#!/usr/bin/env python3
import math
import time
import rclpy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

OBJETIVO = math.pi / 2
TOLERANCIA = math.radians(2)
VEL_MAX = 0.32

def normalizar(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_desde_quaternion(q):
    return math.atan2(
        2 * (q.w*q.z + q.x*q.y),
        1 - 2 * (q.y*q.y + q.z*q.z)
    )

rclpy.init()
nodo = rclpy.create_node("prueba_giro_f1")
pub = nodo.create_publisher(Twist, "/cmd_vel", 10)

estado = {"msg": None, "tiempo": 0.0}

def recibir(msg):
    estado["msg"] = msg
    estado["tiempo"] = time.monotonic()

sub = nodo.create_subscription(
    Odometry, "/odom", recibir, 10
)

try:
    print("Esperando odometria y conexion de control...")
    limite = time.monotonic() + 10

    while (estado["msg"] is None or
           pub.get_subscription_count() == 0):
        if time.monotonic() > limite:
            raise RuntimeError("Interfaces no disponibles")
        rclpy.spin_once(nodo, timeout_sec=0.1)

    inicial = estado["msg"].pose.pose
    yaw0 = yaw_desde_quaternion(inicial.orientation)
    x0 = inicial.position.x
    y0 = inicial.position.y

    print(f"Orientacion inicial: {math.degrees(yaw0):.2f} grados")
    print("Iniciando rotacion antihoraria de 90 grados")

    limite = time.monotonic() + 40
    completado = False

    while time.monotonic() < limite:
        rclpy.spin_once(nodo, timeout_sec=0.05)

        if time.monotonic() - estado["tiempo"] > 2:
            raise RuntimeError("Odometria interrumpida")

        pose = estado["msg"].pose.pose
        yaw = yaw_desde_quaternion(pose.orientation)

        girado = normalizar(yaw - yaw0)
        error = OBJETIVO - girado

        if abs(error) <= TOLERANCIA:
            completado = True
            break

        comando = Twist()
        comando.angular.z = math.copysign(
            min(VEL_MAX, max(0.10, abs(error)*0.8)),
            error
        )
        pub.publish(comando)

    if not completado:
        raise RuntimeError("Tiempo maximo excedido")

finally:
    print("Aplicando parada...")

    for _ in range(15):
        pub.publish(Twist())
        rclpy.spin_once(nodo, timeout_sec=0.05)

    if estado["msg"] is not None and "yaw0" in locals():
        final = estado["msg"].pose.pose
        yawf = yaw_desde_quaternion(final.orientation)
        giro = math.degrees(normalizar(yawf-yaw0))
        desplazamiento = math.hypot(
            final.position.x-x0,
            final.position.y-y0
        )

        print(f"Giro medido: {giro:.2f} grados")
        print(f"Error angular: {abs(90-giro):.2f} grados")
        print(f"Desplazamiento lateral total: {desplazamiento:.4f} m")

    nodo.destroy_node()
    rclpy.shutdown()

print("PRUEBA DE ROTACION COMPLETADA")
