#!/usr/bin/env python3
"""F1.3: arbitraje unico y parada segura de comandos ROS 2 Humble.

Consume /cmd_vel_ruta (geometry_msgs/Twist), /scan y /odom.
Es el UNICO publicador del proyecto en /cmd_vel durante autonomia.
Prueba de prototipo: no supone evitar obstaculos, solo detenerse.
Ejecutar con --ros-args -p use_sim_time:=true.
"""
import math
import signal
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

VMAX = 0.12
WMAX = 0.70
LIMITE_OBSTACULO = 0.35   # m; provisional hasta ensayos de calibracion
EDAD_SENSORES = 1.5       # s de tiempo real
EDAD_COMANDO = 0.4        # s de tiempo real
ANGULO_FRONTAL = math.radians(25)


class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor_f13', automatically_declare_parameters_from_overrides=True)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_prop = self.create_subscription(Twist, '/cmd_vel_ruta', self.cb_prop, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, qos_profile_sensor_data)
        self.t_prop = self.t_scan = self.t_odom = None
        self.prop = None
        self.dist_frente = None
        self.warn_t = 0.0
        self.timer = self.create_timer(0.1, self.control)
        self.get_logger().info('Supervisor activo: unico publicador de /cmd_vel; espera ruta y sensores.')

    def cb_prop(self, msg):
        self.prop = msg
        self.t_prop = time.monotonic()

    def cb_odom(self, msg):
        pose = msg.pose.pose.position
        if all(math.isfinite(v) for v in [pose.x, pose.y, pose.z]):
            self.t_odom = time.monotonic()

    def cb_scan(self, msg):
        valores = []
        for i, r in enumerate(msg.ranges):
            angulo = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                                math.cos(msg.angle_min + i * msg.angle_increment))
            if abs(angulo) > ANGULO_FRONTAL:
                continue
            if math.isinf(r) and r > 0 and math.isfinite(msg.range_max):
                r = msg.range_max
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                valores.append(r)
        self.dist_frente = min(valores) if valores else None
        self.t_scan = time.monotonic()

    def control(self):
        ahora = time.monotonic()
        motivo = None
        if self.t_scan is None or self.t_odom is None:
            motivo = 'esperando sensores'
        elif ahora - self.t_scan > EDAD_SENSORES or ahora - self.t_odom > EDAD_SENSORES:
            motivo = 'datos de sensores caducados'
        elif self.dist_frente is None or self.dist_frente < LIMITE_OBSTACULO:
            motivo = 'LiDAR invalido u obstaculo frontal'
        elif self.t_prop is None or ahora - self.t_prop > EDAD_COMANDO:
            motivo = 'no hay propuesta reciente de ruta'
        else:
            p = self.prop
            valores = [p.linear.x, p.linear.y, p.linear.z,
                       p.angular.x, p.angular.y, p.angular.z]
            if not all(math.isfinite(v) for v in valores):
                motivo = 'Twist contiene NaN/inf'
            elif p.linear.x < 0 or any(abs(v) > 1e-6 for v in
                    [p.linear.y, p.linear.z, p.angular.x, p.angular.y]):
                motivo = 'Twist con componentes no autorizados'
            else:
                cmd = Twist()
                cmd.linear.x = min(VMAX, p.linear.x)
                cmd.angular.z = max(-WMAX, min(WMAX, p.angular.z))
                self.pub.publish(cmd)
                return
        self.pub.publish(Twist())
        if ahora - self.warn_t > 5.0:
            self.get_logger().warn('Parada segura: ' + motivo)
            self.warn_t = ahora

    def parada(self):
        self.pub.publish(Twist())


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    nodo = Supervisor()
    try:
        param = nodo.get_parameter_or('use_sim_time', None)
        if param is None or not bool(param.value):
            raise RuntimeError('Supervisor: se requiere --ros-args -p use_sim_time:=true')
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            nodo.parada()
            time.sleep(0.05)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
