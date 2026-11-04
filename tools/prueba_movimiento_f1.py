#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Twist

VELOCIDAD = 0.10
DURACION = 2.0

rclpy.init()
nodo = rclpy.create_node("prueba_movimiento_f1")
pub = nodo.create_publisher(Twist, "/cmd_vel", 10)

try:
    print("Esperando conexion con TurtleBot3...")

    inicio = time.monotonic()
    while pub.get_subscription_count() == 0:
        if time.monotonic() - inicio > 5.0:
            raise RuntimeError(
                "No hay suscriptor en /cmd_vel"
            )
        time.sleep(0.1)

    print("Iniciando avance controlado.")

    comando = Twist()
    comando.linear.x = VELOCIDAD

    inicio = time.monotonic()
    while time.monotonic() - inicio < DURACION:
        pub.publish(comando)
        time.sleep(0.1)

finally:
    print("Enviando orden de parada.")

    for _ in range(10):
        pub.publish(Twist())
        time.sleep(0.1)

    nodo.destroy_node()
    rclpy.shutdown()

print("Prueba finalizada.")
