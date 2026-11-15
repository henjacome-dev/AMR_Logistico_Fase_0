#!/usr/bin/env python3
"""A3 Etapa 4.2: A* precalculado + Mamdani v03 + CNN + supervisor en Gazebo.

Prueba real en simulacion, NO en robot fisico. No cambia archivos tracked
ni el FIS. Aborta si detecta otro publicador de /cmd_vel.

Uso:
 source tools/activar_entorno.sh
 ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 python3 -u tools/A3_E42_integracion_gazebo.py
"""
from __future__ import annotations

import collections
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
FIS = ROOT / 'matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis'
SHA = '67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903'
VENV = Path.home() / '.venvs/amr-a2-r18/bin/python'
TIMEOUT_RUTA = 180.0
RANGOS = ('/rango_frontal', '/rango_izq', '/rango_der')


def rutas_disponibles():
    salida = []
    for n in (1, 2):
        archivo = ROOT / f'resultados/F12_ruta_{n}.csv'
        with archivo.open(newline='', encoding='utf-8-sig') as f:
            puntos = [(float(x['x_m']), float(x['y_m'])) for x in csv.DictReader(f)]
        assert len(puntos) >= 2
        assert all(math.dist(a, b) <= 0.15001 for a, b in zip(puntos, puntos[1:]))
        salida.append((n, archivo, puntos))
    return salida


def main():
    raiz_git = subprocess.run(
        ['git', '-C', str(ROOT), 'rev-parse', '--show-toplevel'],
        capture_output=True,
        text=True,
        check=True
    ).stdout.strip()
    assert Path(raiz_git).resolve() == ROOT, (
        'El script no pertenece al repositorio de integracion'
    )
    assert FIS.is_file() and hashlib.sha256(FIS.read_bytes()).hexdigest() == SHA, 'FIS v03 no coincide'
    assert VENV.is_file(), 'No existe el entorno virtual amr-a2-r18'
    assert os.getenv('ROS_DOMAIN_ID') == '173', 'Se requiere dominio ROS 173 de Gazebo'
    assert os.getenv('ROS_LOCALHOST_ONLY') == '1', 'Se requiere ROS_LOCALHOST_ONLY=1'

    fecha = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    carpeta = ROOT / 'resultados/A3_E42_gazebo_integral' / fecha
    carpeta.mkdir(parents=True, exist_ok=False)
    logs = {}
    procesos = {}
    auditor = None
    error = None
    resumen = {'aprobado_nominal': False, 'evasion_observada': False, 'fis_sha256': SHA,
               'ruta': None, 'error': None}
    traza = []
    ultimo_reporte = time.monotonic()
    inicio_navegacion = None

    datos = {
        'pose': None, 'pose_t': None, 'lidar_frente': None, 'lidar_t': None,
        'rangos': {topico: None for topico in RANGOS},
        'num_rangos': collections.Counter(),
        'num_cnn': 0, 'num_prop_ruta': 0, 'num_prop_difuso': 0,
        'num_prop_difuso_positiva': 0, 'num_estado_difuso_activo': 0,
        'comandos': 0, 'max_v': 0.0, 'max_w': 0.0,
        'modos': collections.Counter(), 'motivos': collections.Counter(),
        'ultimo_modo': None,
    }

    rclpy.init()
    try:
        auditor = Node('auditor_etapa4_gazebo')
        assert auditor.count_publishers('/cmd_vel') == 0, (
            'Existe otro controlador en /cmd_vel. No se inicia la prueba.'
        )
        for topico in ('/cmd_vel_ruta', '/cmd_vel_difuso', '/estado_difuso',
                       '/estacion_cnn_info', *RANGOS):
            assert auditor.count_publishers(topico) == 0, f'Publicador previo en {topico}'

        # Esperar descubrimiento DDS antes de validar Gazebo.
        controlador = False
        sus_robots = []

        for _ in range(80):
            sus_robots = auditor.get_subscriptions_info_by_topic('/cmd_vel')

            if (
                len(sus_robots) == 1
                and sus_robots[0].node_name == 'turtlebot3_diff_drive'
            ):
                controlador = True
                break

            rclpy.spin_once(auditor, timeout_sec=0.10)

        assert controlador, (
            'Controlador Gazebo no disponible tras espera DDS: '
            + str([s.node_name for s in sus_robots])
        )

        print('[OK] Controlador Gazebo detectado mediante DDS', flush=True)

        def cb_odom(msg):
            p = msg.pose.pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                datos['pose'] = (float(p.x), float(p.y))
                datos['pose_t'] = time.monotonic()

        def cb_scan(msg):
            # El frente esta centrado en 0 rad, no en el minimo de 360 grados.
            vals = []
            for i, dist in enumerate(msg.ranges):
                a = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                               math.cos(msg.angle_min + i * msg.angle_increment))
                if abs(a) > math.radians(25):
                    continue
                if math.isinf(dist) and dist > 0:
                    dist = msg.range_max
                if math.isfinite(dist) and msg.range_min <= dist <= msg.range_max:
                    vals.append(float(dist))
            datos['lidar_frente'] = min(vals) if vals else None
            datos['lidar_t'] = time.monotonic()

        def cb_rango(topico, msg):
            datos['rangos'][topico] = float(msg.range)
            datos['num_rangos'][topico] += 1

        def cb_estado(msg):
            try:
                obj = json.loads(msg.data)
                modo = obj['modo']
                motivo = obj['motivo']
            except (ValueError, KeyError, TypeError):
                return
            datos['modos'][modo] += 1
            datos['motivos'][motivo] += 1
            if datos['ultimo_modo'] != modo:
                print(f'[MODO] {datos["ultimo_modo"]} -> {modo}: {motivo}', flush=True)
                datos['ultimo_modo'] = modo
            if inicio_navegacion is not None:
                traza.append({
                    'tiempo_s': round(time.monotonic() - inicio_navegacion, 3),
                    'x_m': '' if datos['pose'] is None else round(datos['pose'][0], 4),
                    'y_m': '' if datos['pose'] is None else round(datos['pose'][1], 4),
                    'modo': modo, 'motivo': motivo,
                    'v_m_s': obj.get('v', ''), 'omega_rad_s': obj.get('w', ''),
                    'frontal_range_m': datos['rangos']['/rango_frontal'],
                    'izq_range_m': datos['rangos']['/rango_izq'],
                    'der_range_m': datos['rangos']['/rango_der'],
                })

        def cb_cmd(msg):
            datos['comandos'] += 1
            datos['max_v'] = max(datos['max_v'], abs(float(msg.linear.x)))
            datos['max_w'] = max(datos['max_w'], abs(float(msg.angular.z)))

        def cb_prop_ruta(msg):
            datos['num_prop_ruta'] += 1

        def cb_prop_difuso(msg):
            datos['num_prop_difuso'] += 1
            if float(msg.linear.x) > 0:
                datos['num_prop_difuso_positiva'] += 1

        def cb_estado_difuso(msg):
            if msg.data == 'ACTIVO':
                datos['num_estado_difuso_activo'] += 1

        def cb_cnn(msg):
            try:
                json.loads(msg.data)
                datos['num_cnn'] += 1
            except ValueError:
                pass

        subs = [
            auditor.create_subscription(Odometry, '/odom', cb_odom, qos_profile_sensor_data),
            auditor.create_subscription(LaserScan, '/scan', cb_scan, qos_profile_sensor_data),
            auditor.create_subscription(String, '/estado_supervisor', cb_estado, 10),
            auditor.create_subscription(String, '/estado_difuso', cb_estado_difuso, 10),
            auditor.create_subscription(String, '/estacion_cnn_info', cb_cnn, 10),
            auditor.create_subscription(Twist, '/cmd_vel', cb_cmd, 10),
            auditor.create_subscription(Twist, '/cmd_vel_ruta', cb_prop_ruta, 10),
            auditor.create_subscription(Twist, '/cmd_vel_difuso', cb_prop_difuso, 10),
        ]
        for topico in RANGOS:
            subs.append(auditor.create_subscription(
                Range, topico, lambda msg, t=topico: cb_rango(t, msg),
                qos_profile_sensor_data))

        print('=== ETAPA 4.2: PREVUELO GAZEBO ===', flush=True)
        tope = time.monotonic() + 10.0
        while time.monotonic() < tope:
            rclpy.spin_once(auditor, timeout_sec=.10)
            if (datos['pose'] is not None and datos['lidar_frente'] is not None
                    and time.monotonic() - datos['pose_t'] < 1.5
                    and time.monotonic() - datos['lidar_t'] < 1.5):
                break
        assert datos['pose'] is not None and datos['lidar_frente'] is not None, (
            'Gazebo sin /odom o /scan. Iniciar almacen_f1.launch.py en Terminal 1'
        )
        assert datos['lidar_frente'] >= 0.35, 'Obstaculo frontal critico al inicio'
        rutas = rutas_disponibles()
        candidatos = sorted(((math.dist(datos['pose'], pts[0]), n, csv_path, pts)
                            for n, csv_path, pts in rutas), key=lambda x: x[0])
        d0, numero, original, puntos = candidatos[0]
        print(f'Pose inicial: {datos["pose"]} | frontal: {datos["lidar_frente"]:.3f} m')
        print(f'Ruta mas cercana: {numero} | error de inicio: {d0:.3f} m')
        assert d0 <= .25, 'El robot no coincide con el inicio de rutas 1 o 2; no se movera'
        # Se copia la ruta para que el seguidor escriba un CSV de log distinto
        # del F13_ruta_1_log.csv preexistente y modificado.
        ruta_prueba = carpeta / f'F12_ruta_{numero}_E42_{fecha}.csv'
        shutil.copy2(original, ruta_prueba)
        resumen['ruta'] = str(original.relative_to(ROOT))
        resumen['inicio'] = datos['pose']
        resumen['meta'] = puntos[-1]
        print('[OK] Prevuelo completo. Archivo historico F13 no sera sobrescrito.')

        def iniciar(nombre, interprete, script, extra=()):
            log = (carpeta / f'{nombre}.log').open('w', encoding='utf-8')
            logs[nombre] = log
            p = subprocess.Popen([str(interprete), '-u', str(ROOT / script), *map(str, extra)],
                                 cwd=ROOT, env={**os.environ, 'PYTHONUNBUFFERED': '1'},
                                 stdout=log, stderr=subprocess.STDOUT)
            procesos[nombre] = p
            print(f'[INICIADO] {nombre} PID={p.pid}', flush=True)

        print('\n=== ACTIVAR ARQUITECTURA INTEGRAL ===', flush=True)
        iniciar('emulador', sys.executable, 'tools/emulador_sensores_f1.py')
        iniciar('supervisor', sys.executable, 'tools/supervisor_u3s7_cnn.py',
                ('--ros-args', '-p', 'use_sim_time:=true'))
        iniciar('mamdani_v03', VENV, 'tools/A3_nodo_mamdani_v03.py',
                ('--ros', '--ros-args', '-p', 'use_sim_time:=true'))
        iniciar('cnn', sys.executable, 'tools/reconocedor_estacion.py')

        listo = False
        tope = time.monotonic() + 25.0
        while time.monotonic() < tope:
            rclpy.spin_once(auditor, timeout_sec=.08)
            assert all(p.poll() is None for p in procesos.values()), (
                'Un subsistema termino antes de iniciar la ruta'
            )
            listo = (
                auditor.count_publishers('/cmd_vel') == 1
                and auditor.count_publishers('/cmd_vel_difuso') == 1
                and auditor.count_publishers('/estado_difuso') == 1
                and auditor.count_publishers('/estacion_cnn_info') == 1
                and all(auditor.count_publishers(t) == 1 for t in RANGOS)
                and all(datos['num_rangos'][t] >= 3 for t in RANGOS)
                and datos['num_cnn'] >= 2
                and datos['num_estado_difuso_activo'] >= 2
                and datos['num_prop_difuso_positiva'] >= 2
                and datos['modos']['DETENIDO'] >= 2
            )
            if listo:
                break
        assert listo, ('Los subsistemas no estan listos. Revisar logs de la carpeta de evidencia')
        assert datos['max_v'] == 0, 'Movimiento inesperado sin propuesta de ruta'
        print('[OK] Mamdani v03, CNN, emulador y supervisor activos')
        print('[OK] Unico publicador /cmd_vel; detenido antes de la ruta')

        print(f'\n=== NAVEGACION A* RUTA {numero} ===', flush=True)
        iniciar('seguidor', sys.executable, 'tools/seguidor_ruta_f13.py',
                ('--ruta', ruta_prueba.relative_to(ROOT),
                 '--ros-args', '-p', 'use_sim_time:=true'))
        inicio_navegacion = time.monotonic()
        ultimo_reporte = inicio_navegacion
        tope = inicio_navegacion + TIMEOUT_RUTA
        while time.monotonic() < tope:
            rclpy.spin_once(auditor, timeout_sec=.08)
            if any(procesos[n].poll() is not None for n in
                   ('emulador', 'supervisor', 'mamdani_v03', 'cnn')):
                raise RuntimeError('Un subsistema critico finalizo durante la ruta')
            if auditor.count_publishers('/cmd_vel') != 1:
                raise RuntimeError('Numero inesperado de publicadores /cmd_vel')
            if time.monotonic() - ultimo_reporte >= 8:
                print(f'[AVANCE] pose={datos["pose"]} '
                      f'modos={dict(datos["modos"])} '
                      f'CNN={datos["num_cnn"]} '
                      f'difuso_activo={datos["num_estado_difuso_activo"]}', flush=True)
                ultimo_reporte = time.monotonic()
            if procesos['seguidor'].poll() is not None:
                break
        if procesos['seguidor'].poll() is None:
            raise TimeoutError('El seguidor supero el tiempo limite de la prueba')
        for _ in range(8):
            rclpy.spin_once(auditor, timeout_sec=.08)

        error_final = math.dist(datos['pose'], puntos[-1])
        resumen.update({
            'codigo_seguidor': procesos['seguidor'].returncode,
            'posicion_final': datos['pose'], 'error_final_m': round(error_final, 4),
            'duracion_observada_s': round(time.monotonic() - inicio_navegacion, 3),
            'inferencias_cnn': datos['num_cnn'],
            'propuestas_ruta': datos['num_prop_ruta'],
            'propuestas_mamdani': datos['num_prop_difuso'],
            'propuestas_mamdani_positivas': datos['num_prop_difuso_positiva'],
            'estados_mamdani_activo': datos['num_estado_difuso_activo'],
            'comandos_a_gazebo': datos['comandos'],
            'max_v_m_s': round(datos['max_v'], 5),
            'max_w_rad_s': round(datos['max_w'], 5),
            'modos': dict(datos['modos']), 'motivos': dict(datos['motivos']),
            'evasion_observada': datos['modos']['DIFUSO'] > 0,
        })
        print('\n=== RESULTADO INTEGRAL ===')
        print(json.dumps(resumen, indent=2, ensure_ascii=False), flush=True)

        assert procesos['seguidor'].returncode == 0, 'El seguidor no termino normalmente'
        assert error_final <= .15, 'Error final superior a 0.15 m'
        assert datos['modos']['RUTA'] > 0, 'No se observo modo RUTA'
        assert datos['num_prop_difuso_positiva'] > 0, 'Mamdani no propuso velocidad'
        assert datos['num_prop_ruta'] > 0, 'No hubo propuestas de A*'
        assert datos['num_cnn'] > 0, 'CNN no produjo inferencias'
        assert datos['max_v'] <= .1201, 'Velocidad lineal sobre el limite'
        assert datos['max_w'] <= .7001, 'Velocidad angular sobre el limite'
        assert hashlib.sha256(FIS.read_bytes()).hexdigest() == SHA, 'FIS modificado'

        resumen['aprobado_nominal'] = True
        print('\n[OK] INTEGRACION NOMINAL A* + MAMDANI V03 + CNN + SUPERVISOR', flush=True)
        if resumen['evasion_observada']:
            print('[OK] CAMBIO REAL A MODO DIFUSO OBSERVADO EN GAZEBO', flush=True)
        else:
            print('[PENDIENTE] La trayectoria no activo DIFUSO; se requiere '
                  'escenario controlado de obstaculo para acreditar ese cambio.', flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
        print(f'\n[ERROR / INTERRUPCION] {error}', flush=True)
    finally:
        print('\n=== PARADA SEGURA DE ESTA PRUEBA ===', flush=True)
        # Se detiene el seguidor primero: no quedan propuestas de ruta.
        for nombre in ('seguidor', 'cnn', 'mamdani_v03', 'emulador', 'supervisor'):
            p = procesos.get(nombre)
            if p is None:
                continue
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:
                    p.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    p.terminate()
                    try:
                        p.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        p.kill()
                        p.wait(timeout=3)
            print(f'[DETENIDO] {nombre}: salida={p.returncode}', flush=True)
        if auditor is not None:
            parada = auditor.create_publisher(Twist, '/cmd_vel', 10)
            for _ in range(12):
                parada.publish(Twist())
                rclpy.spin_once(auditor, timeout_sec=.045)
            auditor.destroy_publisher(parada)
            auditor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in logs.values():
            handle.close()
        # El seguidor historico escribe resultados/F13_<ruta de prueba>_log.csv;
        # adjuntamos una copia sin alterar el archivo de origen.
        if resumen.get('ruta') is not None:
            ruta_archivo = carpeta / f'F12_ruta_{numero}_E42_{fecha}.csv'
            archivo_trayectoria = ROOT / 'resultados' / (
                'F13_' + ruta_archivo.stem.removeprefix('F12_') + '_log.csv'
            )
            if archivo_trayectoria.is_file():
                shutil.copy2(archivo_trayectoria, carpeta / 'trayectoria_seguidor.csv')
        if traza:
            with (carpeta / 'traza_supervisor.csv').open('w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=list(traza[0]))
                writer.writeheader()
                writer.writerows(traza)
        resumen['error'] = error
        (carpeta / 'resumen.json').write_text(
            json.dumps(resumen, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        print('[EVIDENCIA]', carpeta, flush=True)
        if error:
            for nombre in ('seguidor', 'supervisor', 'mamdani_v03', 'cnn'):
                path = carpeta / f'{nombre}.log'
                if path.is_file():
                    lines = path.read_text(errors='replace').splitlines()
                    print(f'--- {nombre}: ultimas 8 lineas ---', flush=True)
                    print('\n'.join(lines[-8:]), flush=True)
        print('=== FIN ETAPA 4.2 ===', flush=True)
    return 0 if error is None and resumen['aprobado_nominal'] else 1


if __name__ == '__main__':
    sys.exit(main())
