#!/usr/bin/env python3
"""F1.3.3: pruebas de seguridad aisladas del controlador supervisor_f13.py.

No controla el robot. Remapea /cmd_vel, /cmd_vel_ruta, /odom y /scan a
/f133/* exclusivamente para un proceso supervisor de pruebas.
Requiere ROS 2 Humble, Gazebo abierto y reloj /clock en marcha.

Uso desde la raiz del repositorio:
  source tools/activar_entorno.sh
  python3 tools/prueba_seguridad_f133.py --preflight
  python3 tools/prueba_seguridad_f133.py

Genera resultados/F133_seguridad_<fecha>/resumen.csv y supervisor.log.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PREFIJO = '/f133'
REMAPPINGS = [
    ('/cmd_vel', f'{PREFIJO}/cmd_vel'),
    ('/cmd_vel_ruta', f'{PREFIJO}/cmd_vel_ruta'),
    ('/odom', f'{PREFIJO}/odom'),
    ('/scan', f'{PREFIJO}/scan'),
]


class Banco(Node):
    def __init__(self):
        super().__init__('banco_seguridad_f133')
        self.pub_odom = self.create_publisher(Odometry, f'{PREFIJO}/odom', qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, f'{PREFIJO}/scan', qos_profile_sensor_data)
        self.pub_prop = self.create_publisher(Twist, f'{PREFIJO}/cmd_vel_ruta', 10)
        self.recibidos = []
        self.sub = self.create_subscription(Twist, f'{PREFIJO}/cmd_vel', self.cb_vel, 10)

    def cb_vel(self, msg):
        self.recibidos.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def enviar(self, odom=True, scan=True, comando=None, obstaculo=3.0, scan_invalido=False):
        if odom:
            msg = Odometry()
            msg.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(msg)
        if scan:
            msg = LaserScan()
            msg.angle_min = -math.pi
            msg.angle_increment = 2 * math.pi / 360
            msg.angle_max = msg.angle_min + 359 * msg.angle_increment
            msg.range_min = 0.05
            msg.range_max = 3.5
            msg.ranges = [float('nan') if scan_invalido else 3.0] * 360
            if not scan_invalido:
                for i in range(360):
                    if abs(msg.angle_min + i * msg.angle_increment) <= math.radians(10):
                        msg.ranges[i] = obstaculo
            self.pub_scan.publish(msg)
        if comando is not None:
            self.pub_prop.publish(comando)

    def secuencia(self, duracion, *, odom=True, scan=True, comando=None,
                  obstaculo=3.0, scan_invalido=False):
        inicio = time.monotonic()
        comienzo = len(self.recibidos)
        ultimo = 0.0
        while time.monotonic() - inicio < duracion:
            ahora = time.monotonic()
            if ahora - ultimo >= 0.05:
                self.enviar(odom, scan, comando, obstaculo, scan_invalido)
                ultimo = ahora
            rclpy.spin_once(self, timeout_sec=0.025)
        return [x for x in self.recibidos[comienzo:] if x[0] >= inicio + max(0, duracion - 0.33)]


def twist(v=0.08, w=0.0, lateral=0.0):
    x = Twist()
    x.linear.x = v
    x.linear.y = lateral
    x.angular.z = w
    return x


def evaluar(muestras, esperado):
    if not muestras:
        return False, 'sin mensajes /f133/cmd_vel; revisar /clock, QoS y proceso supervisor'
    if esperado == 'cero':
        ok = all(abs(v) <= 1e-6 and abs(w) <= 1e-6 for _, v, w in muestras)
    elif esperado == 'avance':
        ok = any(0.005 < v <= 0.12001 and abs(w) <= 0.70001 for _, v, w in muestras)
    elif esperado == 'limitado':
        ok = (all(-1e-6 <= v <= 0.12001 and abs(w) <= 0.70001 for _, v, w in muestras)
              and any(abs(v - 0.12) < 0.001 and abs(w - 0.70) < 0.001 for _, v, w in muestras))
    else:
        raise ValueError(esperado)
    ult = muestras[-1]
    return ok, f'v_final={ult[1]:.4f}, w_final={ult[2]:.4f}; muestras={len(muestras)}'


def guardar(ruta, filas):
    with ruta.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=['prueba', 'esperado', 'resultado', 'detalle'])
        w.writeheader()
        w.writerows(filas)


def main():
    p = argparse.ArgumentParser(description='F1.3.3 — pruebas de parada segura aisladas')
    p.add_argument('--repo', type=Path, default=Path.cwd())
    p.add_argument('--preflight', action='store_true')
    args = p.parse_args()
    raiz = args.repo.expanduser().resolve()
    supervisor = raiz / 'tools' / 'supervisor_f13.py'
    if not supervisor.is_file():
        raise FileNotFoundError(f'No se encontro {supervisor}')

    rclpy.init()
    nodo = Banco()
    proceso = None
    log = None
    filas = []
    salida = None
    try:
        if any(x.node_name == 'supervisor_f133_test' for x in nodo.get_publishers_info_by_topic(f'{PREFIJO}/cmd_vel')):
            raise RuntimeError('Ya existe un supervisor de pruebas en /f133/cmd_vel')
        # El banco no puede operar sin reloj de simulacion y no toca /cmd_vel.
        limite = time.monotonic() + 4
        reloj = False
        while time.monotonic() < limite:
            rclpy.spin_once(nodo, timeout_sec=0.1)
            if nodo.get_publishers_info_by_topic('/clock'):
                reloj = True
                break
        if not reloj:
            raise RuntimeError('No hay /clock. Inicia Gazebo Classic y despausa la simulacion')
        if args.preflight:
            print('[OK] supervisor_f13.py presente; /clock visible; topicos de salida aislados')
            print('[OK] Preflight: no se ha enviado movimiento al robot')
            return

        salida = raiz / 'resultados' / f'F133_seguridad_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        salida.mkdir(parents=True, exist_ok=False)
        log = (salida / 'supervisor.log').open('w', encoding='utf-8')
        orden = [sys.executable, str(supervisor), '--ros-args',
                 '-r', '__node:=supervisor_f133_test']
        for origen, destino in REMAPPINGS:
            orden.extend(['-r', f'{origen}:={destino}'])
        orden.extend(['-p', 'use_sim_time:=true'])
        proceso = subprocess.Popen(orden, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=raiz)
        # Dar tiempo al descubrimiento DDS y verificar que se apliquen todos los remaps.
        limite = time.monotonic() + 12
        listo = False
        while time.monotonic() < limite:
            rclpy.spin_once(nodo, timeout_sec=0.08)
            if proceso.poll() is not None:
                raise RuntimeError(f'El supervisor terminó prematuramente: exit={proceso.returncode}')
            publi = nodo.get_publishers_info_by_topic(f'{PREFIJO}/cmd_vel')
            otros = nodo.get_publishers_info_by_topic('/cmd_vel')
            desvio = any(x.node_name == 'supervisor_f133_test' for x in otros)
            if desvio:
                raise RuntimeError('FALLO CRITICO: supervisor_f133_test publica sobre /cmd_vel real')
            if (any(x.node_name == 'supervisor_f133_test' for x in publi)
                and nodo.pub_prop.get_subscription_count() > 0
                and nodo.pub_scan.get_subscription_count() > 0
                and nodo.pub_odom.get_subscription_count() > 0):
                listo = True
                break
        if not listo:
            raise RuntimeError('Sin conexiones DDS /f133; verificar supervisor.log y Gazebo')
        print('[OK] Aislamiento comprobado: la salida del test es /f133/cmd_vel')

        casos = [
            ('S01_sin_propuesta', 'cero', dict(comando=None), 0.8),
            ('S02_comando_valido', 'avance', dict(comando=twist(0.08, 0.2)), 0.9),
            ('S03_limites_velocidad', 'limitado', dict(comando=twist(0.50, 3.0)), 0.9),
            ('S04_obstaculo_frontal', 'cero', dict(comando=twist(), obstaculo=0.20), 0.9),
            ('S05_lidar_invalido', 'cero', dict(comando=twist(), scan_invalido=True), 0.9),
            ('S06_twist_nan', 'cero', dict(comando=twist(float('nan'))), 0.9),
            ('S07_comando_reversa', 'cero', dict(comando=twist(-0.05)), 0.9),
            ('S08_movimiento_lateral', 'cero', dict(comando=twist(0.08, lateral=0.02)), 0.9),
        ]
        for nombre, esperado, opciones, tiempo_espera in casos:
            m = nodo.secuencia(tiempo_espera, **opciones)
            ok, detalle = evaluar(m, esperado)
            filas.append(dict(prueba=nombre, esperado=esperado,
                              resultado='OK' if ok else 'FALLO', detalle=detalle))
            print(f"[{'OK' if ok else 'FALLO'}] {nombre}: {detalle}", flush=True)

        # Expiracion de propuesta: sensores validos, sin comandos por >0.4s
        nodo.secuencia(0.5, comando=twist())
        m = nodo.secuencia(1.1, comando=None)
        ok, detalle = evaluar(m, 'cero')
        filas.append(dict(prueba='S09_comando_caducado', esperado='cero',
                          resultado='OK' if ok else 'FALLO', detalle=detalle))
        print(f"[{'OK' if ok else 'FALLO'}] S09_comando_caducado: {detalle}", flush=True)

        # Caducidad de scan; mantener odom y comandos actualizados.
        nodo.secuencia(0.6, comando=twist())
        m = nodo.secuencia(2.0, scan=False, comando=twist())
        ok, detalle = evaluar(m, 'cero')
        filas.append(dict(prueba='S10_lidar_caducado', esperado='cero',
                          resultado='OK' if ok else 'FALLO', detalle=detalle))
        print(f"[{'OK' if ok else 'FALLO'}] S10_lidar_caducado: {detalle}", flush=True)

        # Caducidad de odometria; mantener scan y comandos actualizados.
        nodo.secuencia(0.6, comando=twist())
        m = nodo.secuencia(2.0, odom=False, comando=twist())
        ok, detalle = evaluar(m, 'cero')
        filas.append(dict(prueba='S11_odom_caducada', esperado='cero',
                          resultado='OK' if ok else 'FALLO', detalle=detalle))
        print(f"[{'OK' if ok else 'FALLO'}] S11_odom_caducada: {detalle}", flush=True)

        # Interrupcion controlada del supervisor, bajo topicos aislados.
        # Exigir que antes del SIGINT exista un comando positivo demostrado.
        prueba_marcha = nodo.secuencia(0.7, comando=twist())
        estaba_moviendose, detalle_previo = evaluar(prueba_marcha, 'avance')
        inicio = time.monotonic()
        pos = len(nodo.recibidos)
        proceso.send_signal(signal.SIGINT)
        while time.monotonic() - inicio < 1.4 and proceso.poll() is None:
            rclpy.spin_once(nodo, timeout_sec=0.05)
        final = nodo.recibidos[pos:]
        ok = (estaba_moviendose and proceso.poll() is not None and
              any(abs(v) < 1e-6 and abs(w) < 1e-6 for _, v, w in final))
        detalle = (f'pre_SIGINT={estaba_moviendose}; exit={proceso.poll()}, '
                   f'muestras_post_SIGINT={len(final)}; {detalle_previo}')
        filas.append(dict(prueba='S12_interrupcion_ctrl_c', esperado='cero_final',
                          resultado='OK' if ok else 'FALLO', detalle=detalle))
        print(f"[{'OK' if ok else 'FALLO'}] S12_interrupcion_ctrl_c: {detalle}", flush=True)
    except Exception as exc:
        filas.append(dict(prueba='ERROR_INFRAESTRUCTURA', esperado='sin_errors',
                          resultado='FALLO', detalle=str(exc)))
        print('[ERROR]', exc, file=sys.stderr)
    finally:
        if proceso is not None and proceso.poll() is None:
            proceso.send_signal(signal.SIGTERM)
            try:
                proceso.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proceso.kill()
                proceso.wait(timeout=3)
        if log is not None:
            log.close()
        if salida is not None:
            guardar(salida / 'resumen.csv', filas)
            print('[EVIDENCIA]', salida / 'resumen.csv')
            print('[EVIDENCIA]', salida / 'supervisor.log')
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not filas or any(fila['resultado'] != 'OK' for fila in filas):
        raise SystemExit(2)
    print(f'[FINAL] {len(filas)}/{len(filas)} controles superados. Solo validacion logica ROS aislada.')
    print('[NOTA] No demuestra ausencia de colisiones ni certifica seguridad fisica.')


if __name__ == '__main__':
    main()
