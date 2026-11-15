#!/usr/bin/env python3
"""E4.3.8: corregir SOLO yaw de camara ante un panel de estacion en Gazebo.

No modifica FIS, mundo, modelo CNN ni seguidor. La orden de giro (v=0)
se emite como propuesta /cmd_vel_ruta; el supervisor sigue siendo el UNICO
publicador de /cmd_vel. Durante esta calibracion NO se inicia Mamdani:
si se requiere modo DIFUSO, el supervisor debe detenerse en vez de avanzar.

Uso (dominio 173 y Gazebo abierto):
 python3 tools/A3_E438_alineacion_visual_gazebo.py --self-test
 python3 tools/A3_E438_alineacion_visual_gazebo.py --preflight-ros --destino EMPAQUE
 python3 tools/A3_E438_alineacion_visual_gazebo.py --ejecutar --autorizar-movimiento --destino EMPAQUE

Exclusivo de simulacion. No usar con un robot fisico.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from A3_E432_ejecutor_misiones_ros import (  # type: ignore
    Auditor, ConfirmadorCNN, PUNTOS, comprobar_dominio, comprobar_integridad,
)

# Coordenadas de los paneles verificados en modelos/almacen_f1.world
PANELES = {
    'RECEPCION': (-4.73, -2.8),
    'ALMACENAMIENTO': (-4.73, 2.8),
    'EMPAQUE': (4.73, 2.8),
    'DESPACHO': (4.73, -2.8),
}
RADIUS_INICIO_M = 0.15
RADIUS_CNN_M = 0.40
TOL_YAW_RAD = math.radians(7.0)
MAX_W = 0.35
TIMEOUT_GIRO = 14.0
TIMEOUT_CNN = 25.0
OUT = ROOT / 'resultados/A3_E438_alineacion_visual'


def envolver(a):
    return math.atan2(math.sin(a), math.cos(a))


def angulo_panel(pose, destino):
    px, py = PANELES[destino]
    return math.atan2(py - pose[1], px - pose[0])


def error_yaw(pose, yaw, destino):
    return envolver(angulo_panel(pose, destino) - yaw)


def orden_giro(error):
    if abs(error) <= TOL_YAW_RAD:
        return 0.0
    w = max(-MAX_W, min(MAX_W, 1.0 * error))
    if abs(w) < 0.12:
        w = math.copysign(0.12, w)
    return w


def autoprueba():
    comprobar_integridad()
    n = 0

    def ok(condicion, descripcion):
        nonlocal n
        assert condicion, descripcion
        n += 1
        print(f'[OK {n:02d}] {descripcion}')

    ok(set(PANELES) == set(PUNTOS) - {'ROBOT'}, 'Cuatro paneles conocidos')
    ok(PANELES['EMPAQUE'] == (4.73, 2.8), 'Panel EMPAQUE cotejado con SDF')
    ok(PANELES['DESPACHO'] == (4.73, -2.8), 'Panel DESPACHO cotejado con SDF')
    ok(PANELES['RECEPCION'][0] < 0, 'Panel RECEPCION en pared oeste')
    ok(PANELES['ALMACENAMIENTO'][0] < 0, 'Panel ALMACENAMIENTO en pared oeste')
    muestra = (3.965082772327556, 2.8839121411799513)
    yaw = math.radians(-45.3)
    objetivo = angulo_panel(muestra, 'EMPAQUE')
    err = error_yaw(muestra, yaw, 'EMPAQUE')
    ok(abs(math.degrees(objetivo) + 6.26) < .03, 'Direccion del panel reproduce diagnostico')
    ok(abs(math.degrees(err) - 39.04) < .04, 'Error de yaw reproduce diagnostico')
    ok(0.0 < orden_giro(err) <= MAX_W, 'Control corrige hacia angulo positivo')
    ok(-MAX_W <= orden_giro(-err) < 0, 'Control corrige hacia angulo negativo')
    ok(orden_giro(math.radians(3)) == 0, 'Zona de tolerancia detiene giro')
    ok(abs(envolver(math.radians(179) - math.radians(-179)) - math.radians(-2)) < 1e-9,
       'Error circular sin discontinuidad de 360 grados')
    ok(math.dist(muestra, PUNTOS['EMPAQUE']) < RADIUS_INICIO_M,
       'Posicion de ensayo dentro de radio autorizado')
    ok(RADIUS_INICIO_M < RADIUS_CNN_M, 'Radio de alineacion mas estricto que CNN')
    ok(math.degrees(TOL_YAW_RAD) <= 7.01, 'Tolerancia angular conservadora')
    assert n == 14
    print('[OK] 14/14 pruebas de alineacion aprobadas')
    print('[SEGURIDAD] Sin ROS, sin publicaciones ni movimiento')


class AuditorVisual(Auditor):
    def __init__(self):
        self.yaw = None
        self.t_yaw = 0.0
        self.max_lineal = 0.0
        self.max_angular = 0.0
        self.registro_cnn = []
        self.fase = 'ARRANQUE'
        super().__init__()
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        self.sub_yaw = self.node.create_subscription(
            Odometry, '/odom', self._odom_yaw, qos_profile_sensor_data)

    def _odom_yaw(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.t_yaw = time.monotonic()

    def _cmd(self, msg):
        super()._cmd(msg)
        self.max_lineal = max(self.max_lineal, abs(float(msg.linear.x)))
        self.max_angular = max(self.max_angular, abs(float(msg.angular.z)))

    def _cnn(self, msg):
        super()._cnn(msg)
        try:
            obj = json.loads(msg.data)
            self.registro_cnn.append({
                't_monotonic_s': round(time.monotonic(), 4),
                'fase': self.fase,
                'etiqueta': obj.get('estacion', ''),
                'clase_top1': obj.get('clase_top1', ''),
                'confianza': obj.get('confianza', ''),
                'frame_id': obj.get('frame_id', ''),
                'x_m': '' if self.pose is None else round(self.pose[0], 4),
                'y_m': '' if self.pose is None else round(self.pose[1], 4),
                'yaw_deg': '' if self.yaw is None else round(math.degrees(self.yaw), 3),
            })
        except (TypeError, ValueError, AttributeError):
            pass

    def postura_fresca(self):
        return (self.sensores_frescos() and self.yaw is not None
                and time.monotonic() - self.t_yaw < 1.5)

    def validar_estacion(self, destino):
        assert self.postura_fresca(), 'Odometria/yaw o LiDAR ausentes o caducados'
        distancia = math.dist(self.pose, PUNTOS[destino])
        assert distancia <= RADIUS_INICIO_M, (
            f'Robot fuera de {destino}: error {distancia:.3f} m > {RADIUS_INICIO_M} m')
        assert self.frontal is not None and self.frontal >= .35, (
            'Riesgo frontal: no se autorizan giros')
        return distancia


def verificar_grafo(auditor):
    auditor.espera_gazebo()
    for topico in ('/cmd_vel', '/cmd_vel_ruta', '/cmd_vel_difuso',
                   '/estacion_cnn_info', '/rango_frontal', '/rango_izq', '/rango_der'):
        assert auditor.node.count_publishers(topico) == 0, (
            f'No iniciar: ya existe publicador de {topico}')
    limite = time.monotonic() + 5.0
    while time.monotonic() < limite and not auditor.postura_fresca():
        auditor.spin(.10)
    assert auditor.postura_fresca(), 'No se recibio orientacion reciente'


def preflight(destino):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    comprobar_integridad()
    comprobar_dominio()
    OUT.mkdir(parents=True, exist_ok=True)
    carpeta = OUT / ('prevuelo_' + dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    carpeta.mkdir(exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    a = None
    resultado = {'destino': destino, 'aprobado': False, 'modo': 'SIN_MOVIMIENTO', 'error': None}
    try:
        a = AuditorVisual()
        verificar_grafo(a)
        dist = a.validar_estacion(destino)
        err = error_yaw(a.pose, a.yaw, destino)
        resultado.update({'aprobado': True, 'pose': a.pose,
                          'distancia_meta_m': round(dist, 4),
                          'yaw_deg': round(math.degrees(a.yaw), 3),
                          'direccion_panel_deg': round(math.degrees(angulo_panel(a.pose, destino)),3),
                          'error_angular_deg': round(math.degrees(err),3)})
        print(f'[OK] Gazebo y ROS 2 conectados; /cmd_vel libre')
        print(f'[POSE] {destino}: {a.pose} | error meta={dist:.3f} m')
        print(f'[YAW] robot={math.degrees(a.yaw):.2f} deg | '
              f'panel={math.degrees(angulo_panel(a.pose,destino)):.2f} deg | '
              f'correccion={math.degrees(err):+.2f} deg')
        print('[OK] Prevuelo seguro; no hay publicadores de movimiento previos')
    except Exception as exc:
        resultado['error'] = f'{type(exc).__name__}: {exc}'
        print('[BLOQUEADO]', resultado['error'])
    finally:
        if a is not None:
            a.cerrar()
        if rclpy.ok():
            rclpy.shutdown()
        (carpeta / 'resumen.json').write_text(
            json.dumps(resultado, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print('[EVIDENCIA]', carpeta)
        print('[SEGURIDAD] Solo lectura: no se publico /cmd_vel ni /cmd_vel_ruta')
    return resultado['aprobado']


def ejecutar(destino):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.signals import SignalHandlerOptions
    comprobar_integridad()
    comprobar_dominio()
    carpeta = OUT / ('alineacion_' + dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    carpeta.mkdir(parents=True, exist_ok=False)
    logs = {}
    procesos = {}
    a = None
    pub_ruta = None
    resultado = {'destino': destino, 'aprobado': False, 'alineado': False,
                 'cnn_confirmada': False, 'error': None,
                 'origen_sensores': 'Gazebo /odom /scan /camera/image_raw'}

    def iniciar(nombre, script):
        h = (carpeta / (nombre + '.log')).open('w', encoding='utf-8')
        logs[nombre] = h
        p = subprocess.Popen([sys.executable, '-u', str(ROOT / 'tools' / script),
                              *(['--ros-args', '-p', 'use_sim_time:=true']
                                if nombre == 'supervisor' else [])],
                             cwd=ROOT, env={**os.environ, 'PYTHONUNBUFFERED': '1'},
                             stdout=h, stderr=subprocess.STDOUT)
        procesos[nombre] = p
        print(f'[INICIADO] {nombre} PID={p.pid}', flush=True)

    def vigilar(requiere_ruta=False):
        a.spin(.08)
        assert all(p.poll() is None for p in procesos.values()), 'Subsistema critico finalizo'
        assert a.node.count_publishers('/cmd_vel') == 1, 'Numero inseguro de publicadores /cmd_vel'
        assert a.node.count_publishers('/cmd_vel_difuso') == 0, 'Mamdani ajeno: no iniciar alineacion'
        assert a.postura_fresca(), 'Sensores /odom, yaw o /scan caducados'
        a.validar_estacion(destino)
        assert a.max_lineal < 0.001, 'Se observo movimiento lineal no autorizado'
        if requiere_ruta:
            assert a.node.count_publishers('/cmd_vel_ruta') == 1, 'Publicador de ruta duplicado'

    def enviar_cero():
        if pub_ruta is not None and rclpy.ok():
            for _ in range(6):
                pub_ruta.publish(Twist())
                a.spin(.06)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        a = AuditorVisual()
        verificar_grafo(a)
        d = a.validar_estacion(destino)
        resultado['pose_inicial'] = a.pose
        resultado['error_meta_inicial_m'] = round(d, 4)
        resultado['yaw_inicial_deg'] = round(math.degrees(a.yaw),3)
        resultado['error_angular_inicial_deg'] = round(math.degrees(error_yaw(a.pose,a.yaw,destino)),3)
        print(f'[PREVUELO] {destino}: pose={a.pose} | error={d:.3f} m | '
              f'yaw={math.degrees(a.yaw):+.2f} deg', flush=True)
        # No iniciamos Mamdani en calibracion: si el supervisor solicita DIFUSO,
        # faltara propuesta difusa y se detendra en lugar de avanzar.
        iniciar('emulador', 'emulador_sensores_f1.py')
        iniciar('supervisor', 'supervisor_u3s7_cnn.py')
        iniciar('cnn', 'reconocedor_estacion.py')
        tope = time.monotonic()+30
        listo = False
        while time.monotonic() < tope:
            a.spin(.08)
            assert all(p.poll() is None for p in procesos.values()), 'Fallo subsistema en arranque'
            listo = (a.node.count_publishers('/cmd_vel') == 1
                     and a.node.count_publishers('/estacion_cnn_info') == 1
                     and all(a.node.count_publishers(t) == 1 for t in
                             ('/rango_frontal','/rango_izq','/rango_der'))
                     and a.num_cnn >= 2 and a.modos['DETENIDO'] >= 2)
            if listo:
                break
        assert listo, 'No se conectaron supervisor, emulador y CNN'
        assert a.count_movimiento == 0, 'Hubo movimientos antes de la alineacion'
        print('[OK] Calibracion lista; supervisor detenido antes de giro', flush=True)
        pub_ruta = a.node.create_publisher(Twist, '/cmd_vel_ruta', 10)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and pub_ruta.get_subscription_count() < 1:
            a.spin(.08)
        assert pub_ruta.get_subscription_count() >= 1, 'Supervisor no escucha /cmd_vel_ruta'
        a.fase = 'ALINEACION'
        inicio_giro = time.monotonic()
        estables = 0
        hay_ruta = False
        muestras_giro = []
        while time.monotonic() - inicio_giro < TIMEOUT_GIRO:
            vigilar(requiere_ruta=True)
            err = error_yaw(a.pose, a.yaw, destino)
            w = orden_giro(err)
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = w
            pub_ruta.publish(msg)
            muestras_giro.append((round(time.monotonic()-inicio_giro,3),
                                   round(math.degrees(a.yaw),3),
                                   round(math.degrees(err),3), round(w,4),
                                   '' if a.estado_reciente is None else a.estado_reciente.get('modo','')))
            if abs(err) <= TOL_YAW_RAD:
                estables += 1
                if estables >= 5:
                    break
            else:
                estables = 0
            if a.estado_reciente is not None and a.estado_reciente.get('modo') == 'RUTA':
                hay_ruta = True
            if (time.monotonic() - inicio_giro > 2.0 and not hay_ruta
                    and abs(err) > TOL_YAW_RAD):
                raise RuntimeError('Supervisor no autorizo modo RUTA para giro')
            if (time.monotonic() - inicio_giro > 1.5
                    and a.estado_reciente is not None
                    and a.estado_reciente.get('modo') == 'DIFUSO'):
                raise RuntimeError('Modo DIFUSO activo: giro bloqueado por seguridad')
        assert estables >= 5, 'No se logro orientacion estable dentro de timeout'
        enviar_cero()
        a.node.destroy_publisher(pub_ruta)
        pub_ruta = None
        final_giro = error_yaw(a.pose, a.yaw, destino)
        assert abs(final_giro) <= TOL_YAW_RAD, 'Error angular final excesivo'
        assert a.max_lineal < .001, 'Movimiento lineal observado durante giro'
        resultado.update({'alineado': True, 'yaw_final_deg': round(math.degrees(a.yaw),3),
                          'error_angular_final_deg': round(math.degrees(final_giro),3),
                          'error_posicion_final_m': round(math.dist(a.pose,PUNTOS[destino]),4),
                          'max_velocidad_lineal_m_s': round(a.max_lineal,6),
                          'max_velocidad_angular_rad_s': round(a.max_angular,6)})
        print(f'[OK] Alineacion: yaw={math.degrees(a.yaw):+.2f} deg | '
              f'error panel={math.degrees(final_giro):+.2f} deg | v=0', flush=True)
        with (carpeta/'muestras_giro.csv').open('w', newline='',encoding='utf-8') as f:
            writer=csv.writer(f,lineterminator='\n')
            writer.writerow(['t_s','yaw_deg','error_deg','w_propuesto_rad_s','modo_supervisor'])
            writer.writerows(muestras_giro)
        # La confirmacion comienza SOLO al acreditar parada efectiva.
        a.fase = 'ESPERA_PARADA'
        tstop = time.monotonic()+3
        parado = False
        while time.monotonic() < tstop:
            vigilar()
            estado = a.estado_reciente or {}
            if estado.get('modo') == 'DETENIDO' and abs(float(estado.get('v',1))) <= 1e-5:
                parado=True
                break
        assert parado, 'Supervisor no confirmo parada antes de CNN'
        a.confirmador = ConfirmadorCNN(destino)
        a.fase = 'CONFIRMACION_CNN'
        print(f'[CNN] Esperando tres frames recientes de {destino} con confianza >=80%...',flush=True)
        hasta = time.monotonic()+TIMEOUT_CNN
        while time.monotonic() < hasta and not a.confirmador.confirmado:
            vigilar()
        assert a.confirmador.confirmado, (
            f'CNN no confirmo {destino} tras giro y parada')
        resultado['cnn_confirmada'] = True
        resultado['aprobado'] = True
        print(f'[OK] CNN {destino}: tres fotogramas validos tras alineacion',flush=True)
    except (Exception, KeyboardInterrupt) as exc:
        resultado['error'] = f'{type(exc).__name__}: {exc}'
        print('[BLOQUEADO / ERROR]',resultado['error'],flush=True)
    finally:
        if a is not None and rclpy.ok():
            enviar_cero()
            if pub_ruta is not None:
                a.node.destroy_publisher(pub_ruta)
                pub_ruta = None
            # La propuesta expira y supervisor continua mandando cero.
            t = time.monotonic()+0.65
            while time.monotonic() < t:
                a.spin(.08)
        for nombre in ('cnn','emulador','supervisor'):
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
                        p.wait()
            print(f'[DETENIDO] {nombre} codigo={p.returncode}',flush=True)
        if a is not None:
            if rclpy.ok():
                from geometry_msgs.msg import Twist
                emergencia = a.node.create_publisher(Twist,'/cmd_vel',10)
                for _ in range(12):
                    emergencia.publish(Twist())
                    a.spin(.05)
                a.node.destroy_publisher(emergencia)
            resultado['pose_final'] = a.pose
            resultado['cnn_total'] = a.num_cnn
            resultado['modos'] = dict(a.modos)
            resultado['max_velocidad_lineal_m_s'] = round(a.max_lineal,6)
            resultado['max_velocidad_angular_rad_s'] = round(a.max_angular,6)
            if a.registro_cnn:
                with (carpeta/'fotogramas_cnn.csv').open('w',newline='',encoding='utf-8') as f:
                    writer = csv.DictWriter(f,fieldnames=list(a.registro_cnn[0]),lineterminator='\n')
                    writer.writeheader()
                    writer.writerows(a.registro_cnn)
            a.cerrar()
        if rclpy.ok():
            rclpy.shutdown()
        for h in logs.values():
            h.close()
        (carpeta/'resumen.json').write_text(
            json.dumps(resultado,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('[EVIDENCIA]',carpeta,flush=True)
        print('[SEGURIDAD] Giro sin desplazamiento; propuesta anulada; /cmd_vel final en cero',flush=True)
    return resultado['aprobado']


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    modo=p.add_mutually_exclusive_group(required=True)
    modo.add_argument('--self-test',action='store_true')
    modo.add_argument('--preflight-ros',action='store_true')
    modo.add_argument('--ejecutar',action='store_true')
    p.add_argument('--destino',choices=tuple(PANELES),default='EMPAQUE')
    p.add_argument('--autorizar-movimiento',action='store_true')
    args=p.parse_args()
    if args.self_test:
        autoprueba()
        return 0
    if args.preflight_ros:
        if args.autorizar_movimiento:
            p.error('El prevuelo no permite autorizacion de movimiento')
        return 0 if preflight(args.destino) else 2
    if not args.autorizar_movimiento:
        p.error('La alineacion requiere --autorizar-movimiento')
    return 0 if ejecutar(args.destino) else 2


if __name__=='__main__':
    sys.exit(main())
