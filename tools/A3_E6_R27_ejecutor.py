#!/usr/bin/env python3
"""Etapa 4.3.41: misiones A* con rescate de alineacion angular en Gazebo ROS 2 Humble.

--self-test: SOLO logica, sin ROS.
--preflight-ros: SOLO lectura de /odom, /scan, /camera y grafo ROS.
--ejecutar --autorizar-movimiento: mueve TurtleBot3 en Gazebo, despues de
verificar el origen exacto de la orden y un unico supervisor /cmd_vel.

No altera las rutas F12 ni el Mamdani v03. Uso exclusivo de simulacion;
no ejecutar conectado a robots fisicos u otros receptores de /cmd_vel.
"""
from __future__ import annotations

import argparse
from collections import deque, Counter
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from A3_E43_gestor_misiones import (  # type: ignore
    PUNTOS, FIS, FIS_SHA, catalogo, planificar,
)

VENV = Path.home() / '.venvs/amr-a2-r18/bin/python'
OUT = ROOT / 'resultados/A3_E6_R27_pilotos'
TOL_INICIO = 0.25
TOL_LLEGADA = 0.15
RADIO_CNN = 0.40
EDAD_SENSORES = 1.5
EDAD_CNN_NS = 2_000_000_000
TIMEOUT_TRAMO_S = 330.0
TIMEOUT_CNN_S = 18.0


def comprobar_integridad():
    assert FIS.is_file(), f'No existe FIS: {FIS}'
    assert hashlib.sha256(FIS.read_bytes()).hexdigest() == FIS_SHA, 'FIS v03 alterado'
    assert len(FIS_SHA) == 64
    repo = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--show-toplevel'],
                          check=True, text=True, capture_output=True).stdout.strip()
    assert Path(repo).resolve() == ROOT, 'Directorio de trabajo Git distinto'


def pos_valida(p):
    return p is not None and len(p) == 2 and all(math.isfinite(float(v)) for v in p)


class ConfirmadorCNN:
    """Tres frames CNN distintos, recientes y concordantes cerca de la meta."""
    def __init__(self, destino):
        assert destino in PUNTOS and destino != 'ROBOT'
        self.destino = destino
        self.frames = deque(maxlen=3)
        self.ultimo_frame = -1
        self.confirmado = False

    def recibir(self, obj, pose, edad_odom_s, ahora_ns):
        if self.confirmado:
            return True
        try:
            frame = obj['frame_id']
            etiqueta = str(obj['estacion']).upper().strip()
            clase = str(obj['clase_top1']).upper().strip()
            conf = float(obj['confianza'])
            valido = (isinstance(frame, int) and not isinstance(frame, bool)
                      and frame > self.ultimo_frame
                      and 0 <= ahora_ns - frame <= EDAD_CNN_NS
                      and etiqueta == self.destino and clase == self.destino
                      and math.isfinite(conf) and .80 <= conf <= 1.0
                      and pos_valida(pose) and edad_odom_s <= EDAD_SENSORES
                      and math.dist(pose, PUNTOS[self.destino]) <= RADIO_CNN)
        except (KeyError, ValueError, TypeError, OverflowError):
            valido = False
        if not valido:
            self.frames.clear()
            return False
        self.ultimo_frame = frame
        self.frames.append(frame)
        self.confirmado = len(self.frames) >= 3
        return self.confirmado


def autopueba():
    comprobar_integridad()
    grafo = catalogo()
    contador = 0

    def ok(expr, descripcion):
        nonlocal contador
        assert expr, descripcion
        contador += 1
        print(f'[OK {contador:02d}] {descripcion}', flush=True)

    plan = planificar(grafo, 'ROBOT', ['RECEPCION', 'ALMACENAMIENTO', 'EMPAQUE', 'DESPACHO'])
    ok(len(plan['etapas']) == 4, 'Cuatro misiones planificadas')
    ok(all(math.dist(et['puntos'][0], PUNTOS[et['origen']]) < .001
           for et in plan['etapas']), 'Cada tramo empieza en origen declarado')
    ok(all(math.dist(et['puntos'][-1], PUNTOS[et['destino']]) < .001
           for et in plan['etapas']), 'Cada tramo finaliza en destino declarado')
    ok(all(math.dist(a, b) <= .15001 for et in plan['etapas']
           for a, b in zip(et['puntos'], et['puntos'][1:])), 'Continuidad de waypoints')
    ok(all(math.dist(plan['etapas'][i]['puntos'][-1],
                     plan['etapas'][i+1]['puntos'][0]) < 1e-5 for i in range(3)),
       'Continuidad entre misiones consecutivas')
    ok(len(planificar(grafo, 'EMPAQUE', ['RECEPCION', 'DESPACHO'])['etapas']) == 2,
       'Orden de estaciones arbitrario')
    c = ConfirmadorCNN('RECEPCION')
    punto = PUNTOS['RECEPCION']
    instante = 100_000_000_000

    def mensaje(frame, etiqueta='RECEPCION', clase='RECEPCION', confianza=.99):
        return {'frame_id': frame, 'estacion': etiqueta,
                'clase_top1': clase, 'confianza': confianza}

    ok(not c.recibir(mensaje(instante), (-3., -2.8), .1, instante + 1000),
       'CNN fuera del radio de destino no confirma')
    ok(not c.recibir(mensaje(instante+100), punto, .1, instante+200),
       'Una prediccion correcta no confirma')
    ok(not c.recibir(mensaje(instante+100), punto, .1, instante+300),
       'Frame repetido no suma votos')
    ok(not c.recibir(mensaje(instante+200, etiqueta='EMPAQUE'), punto, .1,
                     instante+300), 'Prediccion incorrecta reinicia votos')
    ok(not c.recibir(mensaje(instante+300, confianza=.79), punto, .1,
                     instante+400), 'Confianza bajo umbral rechazada')
    ok(not c.recibir(mensaje(instante+400), punto, .1,
                     instante+EDAD_CNN_NS+600), 'Frame CNN caducado rechazado')
    ok(not c.recibir(mensaje(instante+500), punto, 2.0,
                     instante+600), 'Odometria caducada bloquea confirmacion')
    ok(not c.recibir(mensaje(instante+600), punto, .1,
                     instante+700), 'Voto CNN 1 de 3')
    ok(not c.recibir(mensaje(instante+700), punto, .1,
                     instante+800), 'Voto CNN 2 de 3')
    ok(c.recibir(mensaje(instante+800), punto, .1,
                 instante+900), 'Voto CNN 3 de 3 confirma llegada')
    ok(c.confirmado, 'Confirmacion CNN conservada')
    ok(math.dist((-3.975, -2.373), PUNTOS['RECEPCION']) > TOL_INICIO,
       'Posicion del ensayo anterior NO autoriza salida de RECEPCION')
    ok(hashlib.sha256(FIS.read_bytes()).hexdigest() == FIS_SHA,
       'SHA256 del FIS v03 intacto')
    assert contador == 19
    print('[OK] 19/19 pruebas del ejecutor aprobadas')
    print('[SEGURIDAD] Sin ROS, Gazebo ni publicaciones /cmd_vel')


from A3_E438_alineacion_visual_gazebo import (
    PANELES, RADIUS_INICIO_M, TOL_YAW_RAD, TIMEOUT_GIRO,
    error_yaw, orden_giro, autoprueba as autoprueba_alineacion,
)


class Auditor:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        self.rclpy = rclpy
        self.node = Node('auditor_e432_misiones')
        self.pose = None
        self.t_pose = 0.0
        self.frontal = None
        self.t_scan = 0.0
        self.num_cnn = 0
        self.num_difuso = 0
        self.modos = Counter()
        self.estado_reciente = None
        self.t_estado = 0.0
        self.confirmador = None
        self.inicio_confirmacion = None
        self.frame_mas_reciente = None
        self.count_movimiento = 0
        self.subs = [
            self.node.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data),
            self.node.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data),
            self.node.create_subscription(String, '/estacion_cnn_info', self._cnn, 10),
            self.node.create_subscription(String, '/estado_supervisor', self._estado, 10),
            self.node.create_subscription(String, '/estado_difuso', self._difuso, 10),
            self.node.create_subscription(Twist, '/cmd_vel', self._cmd, 10),
        ]

    def _odom(self, msg):
        p = msg.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.pose = (float(p.x), float(p.y))
            self.t_pose = time.monotonic()

    def _scan(self, msg):
        valores = []
        for i, d in enumerate(msg.ranges):
            ang = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                             math.cos(msg.angle_min + i * msg.angle_increment))
            if abs(ang) > math.radians(25):
                continue
            if math.isinf(d) and d > 0:
                d = msg.range_max
            if math.isfinite(d) and msg.range_min <= d <= msg.range_max:
                valores.append(float(d))
        self.frontal = min(valores) if valores else None
        self.t_scan = time.monotonic()

    def _cnn(self, msg):
        try:
            info = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.num_cnn += 1
        if self.confirmador is not None:
            self.confirmador.recibir(info, self.pose, time.monotonic() - self.t_pose,
                                    time.monotonic_ns())

    def _estado(self, msg):
        try:
            estado = json.loads(msg.data)
            modo = estado['modo']
        except (ValueError, TypeError, KeyError):
            return
        self.modos[modo] += 1
        self.estado_reciente = estado
        self.t_estado = time.monotonic()

    def _difuso(self, msg):
        if msg.data == 'ACTIVO':
            self.num_difuso += 1

    def _cmd(self, msg):
        if abs(float(msg.linear.x)) > 1e-5 or abs(float(msg.angular.z)) > 1e-5:
            self.count_movimiento += 1

    def spin(self, espera=.10):
        self.rclpy.spin_once(self.node, timeout_sec=espera)

    def espera_gazebo(self):
        inicio = time.monotonic()
        while time.monotonic() - inicio < 12:
            self.spin()
            subs = self.node.get_subscriptions_info_by_topic('/cmd_vel')
            nombres = [s.node_name for s in subs]
            robot = (
                nombres.count('turtlebot3_diff_drive') == 1
                and nombres.count(self.node.get_name()) == 1
                and len(nombres) == 2
            )
            sensores = (self.node.count_publishers('/odom') >= 1
                        and self.node.count_publishers('/scan') >= 1
                        and self.node.count_publishers('/camera/image_raw') >= 1
                        and self.node.count_publishers('/clock') >= 1)
            if robot and sensores and self.sensores_frescos():
                return
        raise RuntimeError('Gazebo, controlador o sensores no detectados a tiempo')

    def sensores_frescos(self):
        ahora = time.monotonic()
        return (pos_valida(self.pose) and self.frontal is not None
                and ahora - self.t_pose < EDAD_SENSORES
                and ahora - self.t_scan < EDAD_SENSORES)

    def validar_origen(self, origen):
        assert self.sensores_frescos(), 'Sensores ausentes o caducados'
        d = math.dist(self.pose, PUNTOS[origen])
        assert d <= TOL_INICIO, (f'Posicion actual {self.pose} no coincide con {origen} '
                                  f'{PUNTOS[origen]}; error {d:.3f} m > {TOL_INICIO} m')
        assert self.frontal >= .35, f'Obstaculo frontal critico: {self.frontal:.3f} m'
        return d

    def cerrar(self):
        self.node.destroy_node()


class AuditorIntegrado(Auditor):
    """Extiende el auditor original sin reemplazar sus validaciones."""
    def __init__(self):
        self.yaw = None
        self.t_yaw = 0.0
        self.en_giro = False
        self.max_lineal_giro = 0.0
        self.max_angular_giro = 0.0
        super().__init__()
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.sub_yaw = self.node.create_subscription(
            Odometry, '/odom', self._odom_yaw, qos_profile_sensor_data)

    def _odom_yaw(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                              1 - 2*(q.y*q.y + q.z*q.z))
        self.t_yaw = time.monotonic()

    def _cmd(self, msg):
        super()._cmd(msg)
        if self.en_giro:
            self.max_lineal_giro = max(self.max_lineal_giro, abs(float(msg.linear.x)))
            self.max_angular_giro = max(self.max_angular_giro, abs(float(msg.angular.z)))

    def postura_fresca(self):
        return (self.sensores_frescos() and self.yaw is not None
                and time.monotonic() - self.t_yaw < EDAD_SENSORES)

    def validar_panel(self, meta):
        assert self.postura_fresca(), 'Orientacion o sensores caducados'
        d = math.dist(self.pose, PUNTOS[meta])
        assert d <= RADIUS_INICIO_M, (
            f'No girar: error meta {d:.3f} m > {RADIUS_INICIO_M} m')
        assert self.frontal >= .35, 'No girar: obstaculo frontal critico'
        return d


def alinear_en_gestor(a, meta, vigilar, carpeta, numero):
    """Giro v=0 mediante /cmd_vel_ruta, nunca publica directamente /cmd_vel.

    El llamante debe haber detenido al seguidor Y a Mamdani, manteniendo
    supervisor, emulador y CNN. Si /cmd_vel_difuso persiste, BLOQUEA.
    """
    from geometry_msgs.msg import Twist
    assert meta in PANELES
    a.validar_panel(meta)
    assert a.node.count_publishers('/cmd_vel') == 1, 'Supervisor no es unico publicador'
    assert a.node.count_publishers('/cmd_vel_difuso') == 0, 'Mamdani aun activo; giro bloqueado'
    assert a.node.count_publishers('/cmd_vel_ruta') == 0, 'Hay un seguidor de ruta activo'
    assert a.node.count_publishers('/estacion_cnn_info') == 1, 'CNN duplicada o ausente'
    assert a.estado_reciente and a.estado_reciente.get('modo') == 'DETENIDO', (
        'Supervisor no esta detenido antes del giro')

    pub = None
    a.max_lineal_giro = 0.0
    a.max_angular_giro = 0.0
    yaw_inicial = math.degrees(a.yaw)
    error_inicial = math.degrees(error_yaw(a.pose, a.yaw, meta))
    muestras = []
    try:
        pub = a.node.create_publisher(Twist, '/cmd_vel_ruta', 10)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and pub.get_subscription_count() < 1:
            vigilar()
        assert pub.get_subscription_count() >= 1, 'Supervisor no escucha propuestas de giro'

        a.en_giro = True
        estable = 0
        hubo_ruta = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < TIMEOUT_GIRO:
            vigilar()
            a.validar_panel(meta)
            assert a.node.count_publishers('/cmd_vel_difuso') == 0, 'Mamdani aparecio durante giro'
            assert a.node.count_publishers('/cmd_vel_ruta') == 1, 'Propuesta de ruta duplicada'
            assert a.max_lineal_giro < .001, 'Velocidad lineal no autorizada durante giro'
            modo = (a.estado_reciente or {}).get('modo', '?')
            assert modo != 'DIFUSO', 'Supervisor pide DIFUSO: alineacion cancelada'
            err = error_yaw(a.pose, a.yaw, meta)
            w = orden_giro(err)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = w
            pub.publish(msg)
            muestras.append((round(time.monotonic()-t0, 3),
                             round(math.degrees(a.yaw), 3),
                             round(math.degrees(err), 3),
                             round(w, 4), modo))
            if abs(err) <= TOL_YAW_RAD:
                estable += 1
                if estable >= 5:
                    break
            else:
                estable = 0
            if modo == 'RUTA':
                hubo_ruta = True
            if (time.monotonic() - t0 > 2.0 and not hubo_ruta
                    and abs(err) > TOL_YAW_RAD):
                raise RuntimeError('Supervisor no autorizo modo RUTA para giro')
        assert estable >= 5, 'No se logro orientacion estable en el tiempo limite'
    finally:
        # Anular propuesta ANTES de abandonar; en caso de error, parada garantizada
        if pub is not None:
            for _ in range(8):
                pub.publish(Twist())
                a.spin(.06)
            a.node.destroy_publisher(pub)
        a.en_giro = False

    t_stop = time.monotonic() + 3
    detenido = False
    while time.monotonic() < t_stop:
        vigilar()
        estado = a.estado_reciente or {}
        if (estado.get('modo') == 'DETENIDO'
                and abs(float(estado.get('v', 1))) <= 1e-5
                and abs(float(estado.get('w', 1))) <= 1e-5
                and time.monotonic() - a.t_estado < .4):
            detenido = True
            break
    assert detenido, 'No se confirmo parada efectiva despues del giro'
    a.validar_panel(meta)
    error_final = math.degrees(error_yaw(a.pose, a.yaw, meta))
    assert abs(math.radians(error_final)) <= TOL_YAW_RAD, 'Error angular final fuera de tolerancia'
    assert a.max_lineal_giro < .001, 'Se observo movimiento lineal en giro'

    csv_salida = carpeta / f'alineacion_{numero:02d}_{meta.lower()}.csv'
    with csv_salida.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(('t_s', 'yaw_deg', 'error_deg', 'w_propuesto_rad_s', 'modo_supervisor'))
        writer.writerows(muestras)
    print(f'[OK] Alineacion {meta}: yaw={math.degrees(a.yaw):+.2f} deg; '
          f'error panel={error_final:+.2f} deg; v=0', flush=True)
    return {
        'aplicada': True, 'yaw_inicial_deg': round(yaw_inicial, 3),
        'error_angular_inicial_deg': round(error_inicial, 3),
        'yaw_final_deg': round(math.degrees(a.yaw), 3),
        'error_angular_final_deg': round(error_final, 3),
        'max_lineal_giro_m_s': round(a.max_lineal_giro, 6),
        'max_angular_giro_rad_s': round(a.max_angular_giro, 6),
        'registro_giro': str(csv_salida.relative_to(ROOT)),
    }


def comprobar_dominio():
    assert os.getenv('ROS_DOMAIN_ID') == '173', 'Se exige ROS_DOMAIN_ID=173 (Gazebo)'
    assert os.getenv('ROS_LOCALHOST_ONLY') == '1', 'Se exige ROS_LOCALHOST_ONLY=1'


def preparar_misiones(desde, orden):
    grafo = catalogo()
    plan = planificar(grafo, desde, orden)
    if any(et['sin_desplazamiento'] for et in plan['etapas']):
        raise ValueError('Evitar destinos repetidos; no deben generar tramo vacio')
    return plan


def prevuelo(desde, orden, guardar=True):
    import rclpy
    comprobar_integridad()
    comprobar_dominio()
    plan = preparar_misiones(desde, orden)
    carpeta = None
    if guardar:
        carpeta = OUT / ('prevuelo_' + dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        carpeta.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    auditor = None
    estado = {'modo': 'PREVUELO_SIN_MOVIMIENTO', 'origen': desde, 'destinos': orden,
              'aprobado': False, 'pose': None, 'error': None}
    try:
        auditor = Auditor()
        auditor.espera_gazebo()
        assert auditor.node.count_publishers('/cmd_vel') == 0, (
            'Otro nodo publica /cmd_vel; abortar y apagar controladores')
        assert auditor.node.count_publishers('/cmd_vel_ruta') == 0, 'Otro seguidor activo'
        assert auditor.node.count_publishers('/cmd_vel_difuso') == 0, 'Otro controlador difuso activo'
        assert auditor.node.count_publishers('/estacion_cnn_info') == 0, 'CNN duplicada'
        d = auditor.validar_origen(desde)
        estado.update({'aprobado': True, 'pose': auditor.pose, 'error_inicio_m': round(d,4),
                       'frontal_m': round(auditor.frontal,4)})
        print(f'[OK] Gazebo: odom={auditor.pose}; LiDAR={auditor.frontal:.3f} m')
        print(f'[OK] Origen {desde} validado; error {d:.3f} m')
        print(f'[OK] {len(plan["etapas"])} misiones configuradas; /cmd_vel sin publicadores')
    except Exception as e:
        estado['error'] = f'{type(e).__name__}: {e}'
        print('[BLOQUEADO]', estado['error'])
    finally:
        if auditor is not None:
            estado['pose'] = auditor.pose
            auditor.cerrar()
        if rclpy.ok():
            rclpy.shutdown()
        if carpeta is not None:
            (carpeta / 'resumen.json').write_text(json.dumps(estado, indent=2) + '\n')
            print('[EVIDENCIA]', carpeta)
    print('[SEGURIDAD] No se publico /cmd_vel ni se movio el robot')
    return estado['aprobado']


def escribir_ruta(etapa, carpeta):
    archivo = carpeta / f"F12_E432_{carpeta.name}_mision_{etapa['numero']:02d}_{etapa['destino'].lower()}.csv"
    with archivo.open('w', encoding='utf-8', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(['x_m', 'y_m'])
        w.writerows(etapa['puntos'])
    return archivo


def ejecutar(desde, orden):
    import rclpy
    comprobar_integridad()
    comprobar_dominio()
    assert VENV.is_file(), f'Falta entorno Mamdani: {VENV}'
    plan = preparar_misiones(desde, orden)
    fecha = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    carpeta = OUT / ('ejecucion_' + fecha)
    carpeta.mkdir(parents=True, exist_ok=False)
    logs = {}
    procesos = {}
    resultado = {'aprobado': False, 'origen': desde, 'destinos': orden, 'etapas': [],
                 'fis_sha256': FIS_SHA, 'error': None}
    auditor = None
    rclpy.init()

    def iniciar(nombre, interprete, script, argumentos=()):
        candidato = nombre
        i_log = 0
        while (carpeta / f'{candidato}.log').exists():
            i_log += 1
            candidato = f'{nombre}_reinicio_{i_log:02d}'
        log = (carpeta / f'{candidato}.log').open('w', encoding='utf-8')
        logs[candidato] = log
        p = subprocess.Popen([str(interprete), '-u', str(ROOT / script), *map(str, argumentos)],
                             cwd=ROOT, env={**os.environ, 'PYTHONUNBUFFERED': '1'},
                             stdout=log, stderr=subprocess.STDOUT)
        procesos[nombre] = p
        print(f'[INICIADO] {nombre} PID={p.pid}', flush=True)
        return p

    def detener(nombre):
        p = procesos.pop(nombre, None)
        if p is None:
            return
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
                    p.wait()
        print(f'[DETENIDO] {nombre} salida={p.returncode}', flush=True)

    def vigilar():
        auditor.spin(.08)
        for nombre in ('supervisor', 'mamdani', 'cnn', 'emulador'):
            p = procesos.get(nombre)
            if p is not None and p.poll() is not None:
                raise RuntimeError(f'Subsistema {nombre} termino inesperadamente')
        if auditor.node.count_publishers('/cmd_vel') != 1:
            raise RuntimeError('El supervisor no es el unico publicador /cmd_vel')
        if not auditor.sensores_frescos():
            raise RuntimeError('Se perdieron sensores Gazebo frescos')

    try:
        auditor = AuditorIntegrado()
        auditor.espera_gazebo()
        assert auditor.node.count_publishers('/cmd_vel') == 0, 'Otro supervisor activo'
        assert auditor.node.count_publishers('/cmd_vel_ruta') == 0, 'Otro seguidor activo'
        assert auditor.node.count_publishers('/cmd_vel_difuso') == 0, 'Otro Mamdani activo'
        assert auditor.node.count_publishers('/estacion_cnn_info') == 0, 'Otra CNN activa'
        d = auditor.validar_origen(desde)
        print(f'[PREVUELO] {desde}: pose={auditor.pose} error={d:.3f} m')
        iniciar('emulador', sys.executable, 'tools/emulador_sensores_f1.py')
        iniciar('supervisor', sys.executable, 'tools/A3_E6_R27_supervisor.py',
                ('--ros-args', '-p', 'use_sim_time:=true'))
        iniciar('mamdani', VENV, 'tools/A3_nodo_mamdani_v03.py',
                ('--ros', '--ros-args', '-p', 'use_sim_time:=true'))
        iniciar('cnn', sys.executable, 'tools/reconocedor_estacion.py')
        tope = time.monotonic() + 30
        listo = False
        while time.monotonic() < tope:
            auditor.spin()
            if any(p.poll() is not None for p in procesos.values()):
                raise RuntimeError('Subsistema termino durante el arranque')
            listo = (auditor.node.count_publishers('/cmd_vel') == 1
                     and auditor.node.count_publishers('/cmd_vel_difuso') == 1
                     and auditor.node.count_publishers('/estacion_cnn_info') == 1
                     and auditor.num_cnn >= 2
                     and auditor.num_difuso >= 2
                     and auditor.modos['DETENIDO'] >= 2)
            if listo:
                break
        assert listo, 'Fallo el arranque integrado de los cuatro subsistemas'
        assert auditor.count_movimiento == 0, 'Se detecto movimiento previo al inicio'
        print('[OK] Cuatro subsistemas conectados; robot detenido')

        for etapa in plan['etapas']:
            meta = etapa['destino']
            err = auditor.validar_origen(etapa['origen'])
            print(f'\n=== MISION {etapa["numero"]}: {etapa["origen"]} -> {meta} ===')
            ruta = escribir_ruta(etapa, carpeta)
            nombre = f'seguidor_{etapa["numero"]:02d}'
            auditor.confirmador = ConfirmadorCNN(meta)
            iniciar(nombre, sys.executable, 'tools/A3_E6_R27_seguidor.py',
                    ('--ruta', ruta.relative_to(ROOT), '--ros-args', '-p', 'use_sim_time:=true'))
            tiempo_inicio = time.monotonic()
            while time.monotonic() - tiempo_inicio < TIMEOUT_TRAMO_S:
                vigilar()
                if procesos[nombre].poll() is not None:
                    break
            assert procesos[nombre].poll() is not None, f'Mision {meta}: timeout de ruta'
            codigo = procesos[nombre].returncode
            assert codigo == 0, f'Seguidor {meta} salio con codigo {codigo}'
            err_llegada = math.dist(auditor.pose, PUNTOS[meta])
            assert err_llegada <= TOL_LLEGADA, (
                f'Error final de {meta} {err_llegada:.3f} m > {TOL_LLEGADA} m')
            print(f'[OK] Geometria de llegada a {meta}: {err_llegada:.3f} m')
            detener(nombre)
            # Esperar parada efectiva del supervisor (propuesta de ruta vencida).
            t_stop = time.monotonic() + 3
            while time.monotonic() < t_stop:
                vigilar()
                es_cero = (auditor.estado_reciente is not None
                           and auditor.estado_reciente.get('modo') == 'DETENIDO'
                           and abs(float(auditor.estado_reciente.get('v', 1))) <= 1e-5
                           and abs(float(auditor.estado_reciente.get('w', 1))) <= 1e-5
                           and time.monotonic() - auditor.t_estado < .4)
                if es_cero:
                    break
            assert es_cero, 'No se confirmo parada del supervisor entre misiones'
            # Primero esperar reconocimiento pasivo: no girar si ya funciona.
            alineacion = {'aplicada': False}
            espera_pasiva = time.monotonic() + 4.0
            while not auditor.confirmador.confirmado and time.monotonic() < espera_pasiva:
                vigilar()
            if not auditor.confirmador.confirmado:
                print(f'[CNN] {meta}: no confirmada; iniciar rescate angular', flush=True)
                auditor.confirmador = None  # Eliminar votos acumulados antes del giro
                detener('mamdani')
                t_retirada = time.monotonic() + 6
                while time.monotonic() < t_retirada:
                    vigilar()
                    if auditor.node.count_publishers('/cmd_vel_difuso') == 0:
                        break
                assert auditor.node.count_publishers('/cmd_vel_difuso') == 0, (
                    'Mamdani sigue publicando: alineacion bloqueada')
                t_parada = time.monotonic() + 3.0
                parado = False
                while time.monotonic() < t_parada:
                    vigilar()
                    estado = auditor.estado_reciente or {}
                    parado = (estado.get('modo') == 'DETENIDO'
                              and abs(float(estado.get('v', 1))) <= 1e-5
                              and abs(float(estado.get('w', 1))) <= 1e-5
                              and time.monotonic() - auditor.t_estado < .4)
                    if parado:
                        break
                assert parado, 'Supervisor no detuvo tras suspender Mamdani'
                alineacion = alinear_en_gestor(
                    auditor, meta, vigilar, carpeta, etapa['numero'])
                auditor.confirmador = ConfirmadorCNN(meta)  # 3 frames posteriores al giro
                t_cnn = time.monotonic() + 25.0
                while not auditor.confirmador.confirmado and time.monotonic() < t_cnn:
                    vigilar()
            assert auditor.confirmador.confirmado, (
                f'CNN no confirmo {meta} con 3 frames validos a menos de {RADIO_CNN} m')
            print(f'[OK] CNN {meta}: tres frames recientes; odometria y parada OK', flush=True)
            resultado['etapas'].append({
                'numero': etapa['numero'], 'origen': etapa['origen'], 'destino': meta,
                'codigo_seguidor': codigo, 'error_final_m': round(err_llegada,4),
                'confirmacion_cnn': True, 'alineacion': alineacion,
                'duracion_s': round(time.monotonic()-tiempo_inicio,2),
            })
            auditor.confirmador = None
            # Para el siguiente trayecto, recuperar Mamdani antes de crear seguidor.
            if ('mamdani' not in procesos
                    and etapa['numero'] != plan['etapas'][-1]['numero']):
                anterior = auditor.num_difuso
                iniciar('mamdani', VENV, 'tools/A3_nodo_mamdani_v03.py',
                        ('--ros', '--ros-args', '-p', 'use_sim_time:=true'))
                listo_mamdani = False
                limite_reinicio = time.monotonic() + 20
                contador_cmd_prev = auditor.count_movimiento
                while time.monotonic() < limite_reinicio:
                    vigilar()
                    assert auditor.count_movimiento == contador_cmd_prev, (
                        'Se detecto comando de movimiento durante reinicio de Mamdani')
                    listo_mamdani = (
                        auditor.node.count_publishers('/cmd_vel_difuso') == 1
                        and auditor.num_difuso >= anterior + 2)
                    if listo_mamdani:
                        break
                assert listo_mamdani, 'Mamdani no reanudo mensajes antes del proximo tramo'
                print('[OK] Mamdani reanudado antes de siguiente mision', flush=True)
        resultado['aprobado'] = True
        print(f'[OK] {len(resultado["etapas"])}/{len(plan["etapas"])} MISIONES CONSECUTIVAS COMPLETADAS')
    except (Exception, KeyboardInterrupt) as e:
        resultado['error'] = f'{type(e).__name__}: {e}'
        print('[BLOQUEADO / ERROR]', resultado['error'])
    finally:
        for nombre in [n for n in procesos if n.startswith('seguidor')]:
            detener(nombre)
        # Mantener supervisor vivo durante medio segundo para que publique parada por timeout de ruta.
        if auditor is not None and 'supervisor' in procesos:
            t = time.monotonic() + .65
            while time.monotonic() < t:
                auditor.spin(.08)
        for nombre in ('cnn', 'mamdani', 'emulador', 'supervisor'):
            detener(nombre)
        if auditor is not None:
            # Publicar ceros de emergencia SOLO despues de detener el supervisor.
            from geometry_msgs.msg import Twist
            pub = auditor.node.create_publisher(Twist, '/cmd_vel', 10)
            for _ in range(12):
                pub.publish(Twist())
                auditor.spin(.05)
            auditor.node.destroy_publisher(pub)
            resultado['pose_final'] = auditor.pose
            resultado['modos'] = dict(auditor.modos)
            resultado['cnn_total'] = auditor.num_cnn
            auditor.cerrar()
        if rclpy.ok():
            rclpy.shutdown()
        for f in logs.values():
            f.close()
        resultado['fis_sha256_final'] = hashlib.sha256(FIS.read_bytes()).hexdigest()
        (carpeta / 'resumen.json').write_text(json.dumps(resultado, indent=2, ensure_ascii=False)+'\n')
        print('[EVIDENCIA]', carpeta)
        print('[SEGURIDAD] Nodos finalizados; orden cero emitida al terminar')
    return resultado['aprobado']


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--self-test', action='store_true')
    g.add_argument('--preflight-ros', action='store_true')
    g.add_argument('--ejecutar', action='store_true')
    p.add_argument('--desde', choices=tuple(PUNTOS))
    p.add_argument('--orden', nargs='+', choices=tuple(x for x in PUNTOS if x != 'ROBOT'))
    p.add_argument('--autorizar-movimiento', action='store_true')
    args = p.parse_args()
    if args.self_test:
        autopueba()
        autoprueba_alineacion()
        print("[OK] Ejecutor E441: 19 + 14 pruebas de seguridad")
        return 0
    if not args.desde or not args.orden:
        p.error('Requiere --desde ORIGEN y --orden DESTINO [DESTINO ...]')
    if args.preflight_ros:
        if args.autorizar_movimiento:
            p.error('--preflight-ros nunca acepta autorizar movimiento')
        return 0 if prevuelo(args.desde, args.orden) else 2
    if not args.autorizar_movimiento:
        p.error('--ejecutar requiere --autorizar-movimiento')
    return 0 if ejecutar(args.desde, args.orden) else 2


if __name__ == '__main__':
    sys.exit(main())
