#!/usr/bin/env python3
"""F1.3 - Seguimiento de waypoints A* para TurtleBot3 (ROS 2 Humble).

Uso:
    python3 tools/seguidor_ruta_f13.py --validar
    python3 tools/seguidor_ruta_f13.py --ruta resultados/F12_ruta_1.csv --ros-args -p use_sim_time:=true

Este prototipo propone velocidades en /cmd_vel_ruta; solo el supervisor publica /cmd_vel.
No replanifica ni evita obstaculos: detiene su propuesta si detecta riesgo.
Los CSV deben utilizar coordenadas x_m,y_m del mismo marco que /odom.
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

VMAX = 0.12                   # m/s (< 0.20 m/s del contrato del proyecto)
WMAX = 0.70                   # rad/s (< 1.50 rad/s del contrato del proyecto)
TOL_WAYPOINT = 0.060         # m
TOL_FINAL = 0.080            # m
MAX_SEPARACION = 0.15        # m entre puntos contiguos del CSV
TOL_INICIO = 0.25            # m entre odometria y primer waypoint
DISTANCIA_FRENADO = 0.35    # m, umbral provisional: requiere calibracion
FOV_FRONTAL = math.radians(25)
DATOS_MAX_EDAD = 1.5        # s de tiempo monotono real
ESPERA_INICIAL = 15.0       # s
TIEMPO_RUTA_MAX = 720.0     # s (primera ruta corta)


def distancia(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


def leer_ruta(path):
    if not path.is_file():
        raise FileNotFoundError(f"No existe el CSV: {path}")
    with path.open(newline="", encoding="utf-8-sig") as archivo:
        lector = csv.DictReader(archivo)
        if lector.fieldnames is None or not {'x_m', 'y_m'}.issubset(lector.fieldnames):
            raise ValueError("El CSV debe tener cabeceras x_m,y_m")
        puntos = []
        for numero, fila in enumerate(lector, start=2):
            x, y = float(fila['x_m']), float(fila['y_m'])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError(f"Coordenadas invalidas, fila {numero}")
            puntos.append((x, y))
    if len(puntos) < 2:
        raise ValueError("La ruta debe tener al menos dos puntos")
    for i in range(1, len(puntos)):
        if distancia(puntos[i-1], puntos[i]) > MAX_SEPARACION:
            raise ValueError(f"Salto mayor que {MAX_SEPARACION} m entre filas {i+1} y {i+2}")
    return puntos


def yaw_cuaternion(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description="Seguidor F1.3 para ROS 2 Humble")
    parser.add_argument('--ruta', default='resultados/F12_ruta_1.csv')
    parser.add_argument('--validar', action='store_true', help='Validar el CSV sin mover el robot ni cargar ROS')
    args, argumentos_ros = parser.parse_known_args()
    ruta_archivo = Path(args.ruta)
    ruta = leer_ruta(ruta_archivo)
    longitud = sum(distancia(a, b) for a, b in zip(ruta[:-1], ruta[1:]))
    print(f"[RUTA] {ruta_archivo} | {len(ruta)} puntos | {longitud:.3f} m")
    print(f"[INICIO] {ruta[0]} | [META] {ruta[-1]}")
    if args.validar:
        print('[OK] Formato y continuidad del CSV validados. No se envio movimiento.')
        return

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class Seguidor(Node):
        def __init__(self):
            super().__init__('seguidor_ruta_f13', automatically_declare_parameters_from_overrides=True)
            self.ruta = ruta
            self.indice = 0
            self.pose = None
            self.odom_t = None
            self.scan_t = None
            self.dist_frente = None
            self.inicio = time.monotonic()
            self.inicio_ruta = None
            self.terminado = False
            self.exitoso = False
            self.motivo = ''
            self.registros = []
            self.salida = Path('/home/enry/AMR_Logistico_Fase_0/resultados/F1_A_cierre_20261106/seguimiento_orden_A.csv')
            self.pub = self.create_publisher(Twist, '/cmd_vel_ruta', 10)
            self.sub_odom = self.create_subscription(
                Odometry, '/odom', self.cb_odom, qos_profile_sensor_data)
            self.sub_scan = self.create_subscription(
                LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
            self.timer = self.create_timer(0.10, self.control)
            self.get_logger().info('Esperando /odom, /scan y supervisor suscrito a /cmd_vel_ruta.')

        def cb_odom(self, msg):
            pose = msg.pose.pose
            self.pose = (pose.position.x, pose.position.y, yaw_cuaternion(pose.orientation))
            self.odom_t = time.monotonic()

        def cb_scan(self, msg):
            lecturas = []
            for i, valor in enumerate(msg.ranges):
                angulo = normalizar(msg.angle_min + i * msg.angle_increment)
                if abs(angulo) > FOV_FRONTAL:
                    continue
                if math.isinf(valor) and valor > 0 and math.isfinite(msg.range_max):
                    valor = msg.range_max
                if math.isfinite(valor) and msg.range_min <= valor <= msg.range_max:
                    lecturas.append(valor)
            self.dist_frente = min(lecturas) if lecturas else None
            self.scan_t = time.monotonic()

        def parar(self):
            self.pub.publish(Twist())

        def finalizar(self, exito, motivo):
            self.parar()
            self.exitoso = exito
            self.motivo = motivo
            self.terminado = True
            (self.get_logger().info if exito else self.get_logger().error)(motivo)

        def control(self):
            if self.terminado:
                return
            ahora = time.monotonic()
            if self.pose is None or self.odom_t is None or self.scan_t is None or self.pub.get_subscription_count() == 0:
                self.parar()
                if ahora - self.inicio > ESPERA_INICIAL:
                    self.finalizar(False, 'Tiempo agotado esperando sensores o supervisor de velocidades.')
                return
            if ahora - self.odom_t > DATOS_MAX_EDAD or ahora - self.scan_t > DATOS_MAX_EDAD:
                self.finalizar(False, 'Datos de odometria/LiDAR vencidos: parada de seguridad.')
                return
            if self.dist_frente is None:
                self.finalizar(False, 'LiDAR sin datos frontales validos: parada de seguridad.')
                return
            if self.inicio_ruta is None:
                error = distancia(self.pose[:2], self.ruta[0])
                if error > TOL_INICIO:
                    self.finalizar(False, f'El origen CSV no coincide con /odom (error {error:.3f} m).')
                    return
                self.inicio_ruta = ahora
                self.get_logger().info(f'Inicio aceptado; error: {error:.3f} m')
            if ahora - self.inicio_ruta > TIEMPO_RUTA_MAX:
                self.finalizar(False, 'Tiempo maximo de ruta excedido.')
                return
            x, y, yaw = self.pose
            while self.indice < len(self.ruta) - 1 and distancia((x, y), self.ruta[self.indice]) <= TOL_WAYPOINT:
                self.indice += 1
            objetivo = self.ruta[self.indice]
            d = distancia((x, y), objetivo)
            if self.indice == len(self.ruta) - 1 and d <= TOL_FINAL:
                self.finalizar(True, f'Meta alcanzada; error final {d:.3f} m')
                return
            if self.dist_frente < DISTANCIA_FRENADO:
                self.finalizar(False, f'Obstaculo a {self.dist_frente:.3f} m; se requiere replanificacion.')
                return
            rumbo = math.atan2(objetivo[1] - y, objetivo[0] - x)
            error = normalizar(rumbo - yaw)
            comando = Twist()
            comando.angular.z = max(-WMAX, min(WMAX, 1.5*error))
            if abs(error) < math.radians(25):
                if self.indice == len(self.ruta) - 1:
                    velocidad_base = 0.45 * d
                else:
                    velocidad_base = max(0.075, 0.45 * d)

                comando.linear.x = min(VMAX, velocidad_base) * max(
                    0.2, math.cos(error)
                )
            self.pub.publish(comando)
            self.registros.append([round(ahora - self.inicio_ruta, 3), x, y, yaw,
                                   self.indice, objetivo[0], objetivo[1], d,
                                   self.dist_frente, comando.linear.x, comando.angular.z])

        def guardar_log(self):
            self.salida.parent.mkdir(parents=True, exist_ok=True)
            with self.salida.open('w', encoding='utf-8', newline='') as f:
                escritor = csv.writer(f, lineterminator='\n')
                escritor.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad', 'waypoint', 'meta_x_m',
                                   'meta_y_m', 'error_m', 'laser_frontal_m', 'v_m_s', 'w_rad_s'])
                escritor.writerows(self.registros)
            print(f'[LOG] {self.salida} | {len(self.registros)} muestras')

    rclpy.init(args=argumentos_ros)
    nodo = Seguidor()
    try:
        param = nodo.get_parameter_or('use_sim_time', None)
        if param is None or not bool(param.value):
            raise RuntimeError('Se requiere --ros-args -p use_sim_time:=true')
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.2)
    except KeyboardInterrupt:
        print('[PARADA] Cancelado con Ctrl+C')
    finally:
        for _ in range(8):
            nodo.parar()
            time.sleep(0.05)
        nodo.guardar_log()
        resultado = nodo.exitoso
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not resultado:
        sys.exit(2)


if __name__ == '__main__':
    main()
