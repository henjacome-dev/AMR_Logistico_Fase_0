#!/usr/bin/env python3
"""Avance 3: controlador Mamdani v03 oficial para ROS 2 Humble.

Entradas: tres sensor_msgs/Range.
Salidas: /cmd_vel_difuso y /estado_difuso.
Nunca publica directamente en /cmd_vel.
"""

import argparse
import hashlib
import math
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
FIS = ROOT / (
    "matlab/A2_MAMDANI/A2_v03_rubrica/"
    "AMR_Mamdani_A2_v03_rubrica.fis"
)

SHA256_V03 = (
    "67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903"
)

sys.path.insert(0, str(ROOT / "tools"))

from A2_E21_equivalencia_MATLAB_Python import (
    parse_fis,
    build_skfuzzy,
)
from A2_E22_nodo_difuso import BancoRangos, ORDER, TOPICS


def verificar_fis():
    assert FIS.is_file(), f"No existe: {FIS}"
    actual = hashlib.sha256(FIS.read_bytes()).hexdigest()
    assert actual == SHA256_V03, (
        f"FIS v03 distinto de la referencia: {actual}"
    )


class MotorMamdaniV03:
    def __init__(self):
        verificar_fis()

        original = FIS.read_text(encoding="utf-8")

        nombre_v03 = "Name='AMR_Mamdani_A2_v03_rubrica'"
        nombre_compatible = "Name='AMR_Mamdani_A2_v01'"

        assert original.count(nombre_v03) == 1

        # Adaptacion temporal del identificador que exige el
        # analizador historico. No cambia reglas ni membresias.
        with tempfile.TemporaryDirectory(
            prefix="amr_v03_parser_"
        ) as carpeta:
            temporal = Path(carpeta) / "compatible.fis"

            temporal.write_text(
                original.replace(
                    nombre_v03,
                    nombre_compatible,
                    1
                ),
                encoding="utf-8"
            )

            entradas, salidas, reglas = parse_fis(temporal)

        assert len(entradas) == 3
        assert len(salidas) == 2
        assert len(reglas) == 27

        assert [v[0] for v in entradas] == [
            "df", "di", "dd"
        ]
        assert [v[0] for v in salidas] == [
            "v", "omega"
        ]

        self.reglas = reglas

        from skfuzzy import control as ctrl

        sistema = build_skfuzzy(
            entradas, salidas, reglas
        )

        self.sim = ctrl.ControlSystemSimulation(
            sistema,
            clip_to_bounds=False,
            cache=False,
            lenient=False,
        )

        verificar_fis()

    def calcular(self, valores):
        for clave in ORDER:
            valor = float(valores[clave])

            if not math.isfinite(valor) or not 0 <= valor <= 2:
                raise ValueError(
                    f"Sensor invalido {clave}={valor}"
                )

            self.sim.input[clave] = valor

        self.sim.compute()

        v = float(self.sim.output["v"])
        w = float(self.sim.output["omega"])

        if not math.isfinite(v) or not math.isfinite(w):
            raise ValueError(
                "Salida Mamdani no finita"
            )

        if not (
            -1e-8 <= v <= 0.2 + 1e-8
            and -1.5 - 1e-8 <= w <= 1.5 + 1e-8
        ):
            raise ValueError(
                f"Salida fuera de rango: v={v}, w={w}"
            )

        return (
            max(0.0, min(0.2, v)),
            max(-1.5, min(1.5, w)),
        )


def autopueba():
    print("=== MAMDANI V03: VALIDACION AISLADA ===")

    motor = MotorMamdaniV03()

    casos = {
        "PasilloLibre": (1.7, 1.7, 1.7),
        "ObstaculoIzquierdo": (1.7, 0.2, 1.7),
        "ObstaculoDerecho": (1.7, 1.7, 0.2),
        "FrenteIzquierdaLibre": (0.2, 1.7, 0.2),
        "FrenteDerechaLibre": (0.2, 0.2, 1.7),
        "TresLadosCerca": (0.2, 0.2, 0.2),
        "FrenteCercaLadosMedios": (0.2, 0.9, 0.9),
        "FrenteCercaLadosLejos": (0.2, 1.7, 1.7),
        "DistanciaFrontalUmbral": (0.6, 0.9, 0.9),
        "DistanciaLateralUmbral": (1.7, 0.35, 1.7),
    }

    salidas = {}

    for nombre, valores in casos.items():
        entradas = dict(zip(ORDER, valores))
        v, w = motor.calcular(entradas)

        assert 0 <= v <= 0.2
        assert -1.5 <= w <= 1.5

        salidas[nombre] = (v, w)

        print(
            f"[OK] {nombre}: "
            f"v={v:.6f} m/s, "
            f"omega={w:+.6f} rad/s"
        )

    assert len(salidas) == 10
    assert (
        salidas["TresLadosCerca"][0]
        < salidas["PasilloLibre"][0]
    )
    assert salidas["ObstaculoIzquierdo"][1] < -0.1
    assert salidas["ObstaculoDerecho"][1] > 0.1

    print("[OK] 10/10 escenarios numericos")
    print("[OK] 27 reglas Mamdani v03")

    banco = BancoRangos()

    assert banco.obtener(10.0)[0] is None

    for clave in ORDER:
        banco.recibir(
            clave, 1.7, 0.0, 2.0, 10.0
        )

    entradas, estado = banco.obtener(10.1)
    assert estado == "ACTIVO"
    assert entradas is not None

    motor.calcular(entradas)

    banco.recibir(
        "df", float("nan"), 0.0, 2.0, 10.2
    )

    assert banco.obtener(10.2)[0] is None

    banco.recibir(
        "df", 1.7, 0.0, 2.0, 10.3
    )

    assert banco.obtener(11.2)[0] is None

    print("[OK] Sensores ausentes bloqueados")
    print("[OK] Sensores invalidos bloqueados")
    print("[OK] Sensores caducados bloqueados")

    verificar_fis()

    print("[OK] SHA256 del FIS v03 conservado")
    print("=== AUTOPRUEBA MAMDANI V03 APROBADA ===")


def ejecutar_ros(argumentos_ros):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    motor = MotorMamdaniV03()

    class NodoMamdaniV03(Node):
        def __init__(self):
            super().__init__("controlador_mamdani_v03")

            self.banco = BancoRangos()

            self.pub = self.create_publisher(
                Twist, "/cmd_vel_difuso", 10
            )

            self.pub_estado = self.create_publisher(
                String, "/estado_difuso", 10
            )

            self.subs = []

            for clave, topico in TOPICS.items():
                sub = self.create_subscription(
                    Range,
                    topico,
                    lambda msg, k=clave: self.recibir(
                        k, msg
                    ),
                    10,
                )
                self.subs.append(sub)

            self.timer = self.create_timer(
                0.2, self.control
            )

            self.get_logger().info(
                "Mamdani v03 oficial activo. "
                "Solo publica propuestas difusas."
            )

        def recibir(self, clave, msg):
            self.banco.recibir(
                clave,
                float(msg.range),
                float(msg.min_range),
                float(msg.max_range),
                time.monotonic(),
            )

        def control(self):
            entradas, estado = self.banco.obtener(
                time.monotonic()
            )

            comando = Twist()

            if entradas is not None:
                try:
                    v, w = motor.calcular(entradas)
                    comando.linear.x = v
                    comando.angular.z = w
                except Exception as exc:
                    estado = "ERROR_INFERENCIA"
                    self.get_logger().error(str(exc))

            self.pub.publish(comando)
            self.pub_estado.publish(
                String(data=estado)
            )

    rclpy.init(args=argumentos_ros)
    nodo = None

    try:
        nodo = NodoMamdaniV03()
        rclpy.spin(nodo)

    except KeyboardInterrupt:
        pass

    finally:
        if nodo is not None:
            for _ in range(3):
                nodo.pub.publish(Twist())
                nodo.pub_estado.publish(
                    String(data="DETENIDO")
                )
                time.sleep(0.05)

            nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    opciones = parser.add_mutually_exclusive_group(
        required=True
    )
    opciones.add_argument(
        "--self-test", action="store_true"
    )
    opciones.add_argument(
        "--ros", action="store_true"
    )

    args, resto = parser.parse_known_args()

    if args.self_test:
        autopueba()
    else:
        ejecutar_ros(resto)


if __name__ == "__main__":
    main()
