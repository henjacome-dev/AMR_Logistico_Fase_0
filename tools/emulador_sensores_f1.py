#!/usr/bin/env python3
"""B3 - Emulador de 3 sensores de distancia para el AMR logístico.

ROS 2 Humble. Entrada: /scan (LaserScan).
Salidas: /rango_frontal, /rango_izq, /rango_der (Range).
Sectores de 15 grados, centrados en 0, +45 y -45 grados.
Alcance [0, 2] m; ruido gaussiano sigma = 0.01 m.
No ordena movimiento ni publica en /cmd_vel.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, Range


SECTORES = {
    "/rango_frontal": 0.0,
    "/rango_izq": 45.0,
    "/rango_der": -45.0,
}
AMPLITUD_RAD = math.radians(15.0)
ALCANCE_MAX_M = 2.0
SIGMA_M = 0.01


def diferencia_angular(a, b):
    """Diferencia angular normalizada a [-pi, pi], incluso en +/-180 grados."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def distancia_sector(scan, centro_rad):
    """Distancia minima medida dentro del sector; 2 m si no hay deteccion."""
    if not scan.ranges or not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        return None

    minimo = ALCANCE_MAX_M
    incluye_rayos = False
    for i, lectura in enumerate(scan.ranges):
        angulo = scan.angle_min + i * scan.angle_increment
        if abs(diferencia_angular(angulo, centro_rad)) > AMPLITUD_RAD / 2.0:
            continue
        incluye_rayos = True

        # NaN / +inf: sin lectura util para localizar un obstaculo.
        if math.isnan(lectura) or lectura == math.inf:
            continue
        # -inf o un retorno positivo por debajo del minimo del lidar:
        # tratar conservadoramente como obstaculo muy proximo.
        if lectura == -math.inf or (math.isfinite(lectura) and 0 <= lectura < scan.range_min):
            minimo = 0.0
            continue
        if not math.isfinite(lectura) or lectura < 0:
            continue
        if lectura > scan.range_max:
            continue
        minimo = min(minimo, lectura)

    return minimo if incluye_rayos else None


class EmuladorSensores(Node):
    def __init__(self):
        super().__init__("emulador_sensores")
        self.declare_parameter("semilla", 20261105)
        semilla = self.get_parameter("semilla").value
        self.azar = random.Random(int(semilla))
        self.mensajes = 0

        qos_lidar = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publicadores = {
            topico: self.create_publisher(Range, topico, 10)
            for topico in SECTORES
        }
        self.create_subscription(LaserScan, "/scan", self.recibir_scan, qos_lidar)
        self.get_logger().info(
            "B3 activo: /scan -> /rango_frontal, /rango_izq, /rango_der "
            "(sectores 15 deg; centros 0/+45/-45; max 2 m; sigma 0.01 m)"
        )

    def recibir_scan(self, scan):
        if not scan.ranges or not math.isfinite(scan.angle_min):
            self.get_logger().warn("LaserScan vacio o con angulos invalidos; no se publican rangos")
            return

        resultados = {}
        for topico, centro_grados in SECTORES.items():
            base = distancia_sector(scan, math.radians(centro_grados))
            if base is None:
                self.get_logger().warn(f"No hay rayos para {topico}; no se publica ese rango")
                continue

            # Sin obstaculo a menos de 2 m: lectura saturada en el alcance.
            # Con obstaculo: ruido independiente y truncamiento al intervalo [0, 2].
            lectura = (
                ALCANCE_MAX_M if base >= ALCANCE_MAX_M
                else min(ALCANCE_MAX_M, max(0.0, base + self.azar.gauss(0.0, SIGMA_M)))
            )

            mensaje = Range()
            mensaje.header = scan.header
            mensaje.radiation_type = Range.INFRARED
            mensaje.field_of_view = AMPLITUD_RAD
            mensaje.min_range = 0.0
            mensaje.max_range = ALCANCE_MAX_M
            mensaje.range = float(lectura)
            self.publicadores[topico].publish(mensaje)
            resultados[topico] = lectura

        self.mensajes += 1
        if self.mensajes % 50 == 0 and len(resultados) == 3:
            self.get_logger().info(
                "Rangos: frente=%.3f, izq=%.3f, der=%.3f m"
                % (
                    resultados["/rango_frontal"],
                    resultados["/rango_izq"],
                    resultados["/rango_der"],
                )
            )


def main(args=None):
    rclpy.init(args=args)
    nodo = EmuladorSensores()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
