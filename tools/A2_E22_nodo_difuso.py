#!/usr/bin/env python3
"""AMR Logistico - E2.2: nodo de propuestas difusas ROS 2 Humble.

El nodo SOLO publica /cmd_vel_difuso (propuesta) y /estado_difuso.
NUNCA publica /cmd_vel ni modifica el supervisor existente.
Recibe /rango_frontal, /rango_izq y /rango_der (sensor_msgs/Range).
El FIS v01 es una referencia inmutable; el modelo se reconstruye
mediante el parser de R18 ya validado y publicado.

Uso seguro:
  python3 /mnt/c/Users/Henry/Downloads/A2_E22_nodo_difuso.py --instalar-y-probar
  source tools/activar_entorno.sh
  ~/.venvs/amr-a2-r18/bin/python tools/A2_E22_nodo_difuso.py --ros
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time

REPO = (Path(__file__).resolve().parents[1] if Path(__file__).resolve().parent.name == "tools" else Path.home() / "AMR_Logistico_Fase_0")
R18 = REPO / "tools/A2_E21_equivalencia_MATLAB_Python.py"
FIS = REPO / "matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis"
EXPECTED_FIS_BLOB = "6c7dfdd083728d49c64dd1c90a59b19d63222063"
TIMEOUT_S = 0.8  # Segundos reales desde la recepcion, independientes de /clock.
PERIOD_S = 0.2  # 5 Hz, fase experimental.
ORDER = ("df", "di", "dd")
TOPICS = {"df": "/rango_frontal", "di": "/rango_izq", "dd": "/rango_der"}


def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def check_fis() -> None:
    if not FIS.is_file() or git_blob(FIS.read_bytes()) != EXPECTED_FIS_BLOB:
        raise RuntimeError("FIS v01 ausente o distinto de la referencia inmutable")
    if not R18.is_file():
        raise RuntimeError("No existe tools/A2_E21_equivalencia_MATLAB_Python.py")


def import_r18():
    check_fis()
    spec = importlib.util.spec_from_file_location("amr_a2_r18_core", R18)
    if spec is None or spec.loader is None:
        raise RuntimeError("No se puede importar el modulo R18")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class MotorMamdani:
    """Motor scikit-fuzzy con la misma construccion auditada en R18."""

    def __init__(self):
        from skfuzzy import control as ctrl
        self.r18 = import_r18()
        entradas, salidas, reglas = self.r18.parse_fis(FIS)
        self.sistema = self.r18.build_skfuzzy(entradas, salidas, reglas)
        self.sim = ctrl.ControlSystemSimulation(
            self.sistema, clip_to_bounds=False, cache=False, lenient=False
        )

    def calcular(self, valores: dict[str, float]) -> tuple[float, float]:
        for key in ORDER:
            x = float(valores[key])
            if not math.isfinite(x) or not (0.0 <= x <= 2.0):
                raise ValueError(f"Entrada {key} invalida: {x}")
            self.sim.input[key] = x
        self.sim.compute()
        v = float(self.sim.output["v"])
        w = float(self.sim.output["omega"])
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError("Salida Mamdani no finita")
        if not (-1e-9 <= v <= 0.2 + 1e-9 and -1.5 - 1e-9 <= w <= 1.5 + 1e-9):
            raise ValueError(f"Salida fuera de rango: v={v}, omega={w}")
        return min(0.2, max(0.0, v)), min(1.5, max(-1.5, w))


class BancoRangos:
    """Valida el dato recibido y su edad; nunca reutiliza lecturas invalidas."""

    def __init__(self, timeout_s: float = TIMEOUT_S):
        self.timeout_s = float(timeout_s)
        self.lecturas: dict[str, tuple[float, float] | None] = {
            key: None for key in ORDER
        }
        self.motivos = {key: "SIN_DATO" for key in ORDER}

    def recibir(self, key: str, value: float, minimo: float, maximo: float,
                ahora: float) -> None:
        if key not in ORDER:
            raise KeyError(key)
        valid = (all(math.isfinite(x) for x in (value, minimo, maximo, ahora))
                 and minimo >= 0.0 and maximo >= minimo
                 and 0.0 <= value <= 2.0 and minimo <= value <= maximo)
        if valid:
            self.lecturas[key] = (float(value), float(ahora))
            self.motivos[key] = "VALIDO"
        else:
            self.lecturas[key] = None
            self.motivos[key] = "INVALIDO"

    def obtener(self, ahora: float) -> tuple[dict[str, float] | None, str]:
        valores = {}
        for key in ORDER:
            dato = self.lecturas[key]
            if dato is None:
                return None, f"{self.motivos[key]}_{key}"
            valor, instante = dato
            if not math.isfinite(ahora) or instante > ahora + 1e-6 or ahora - instante > self.timeout_s:
                return None, f"CADUCADO_{key}"
            valores[key] = valor
        return valores, "ACTIVO"


def probar() -> None:
    motor = MotorMamdani()
    referencias = motor.r18.load_reference(REPO / motor.r18.CSV_REL)
    validos = 0
    max_v, max_w = 0., 0.
    print("=== E2.2 - PRUEBA AISLADA SIN ROS NI MOVIMIENTO ===", flush=True)
    for fila in referencias:
        v, w = motor.calcular({k: fila[k] for k in ORDER})
        _, err_v, _, cumple_v = motor.r18.compare_value(fila["v"], v, "v")
        _, err_w, _, cumple_w = motor.r18.compare_value(fila["omega"], w, "omega")
        assert cumple_v and cumple_w, f"Caso R18 no conforme: {fila['Escenario']}"
        max_v = max(max_v, err_v or 0.)
        max_w = max(max_w, err_w or 0.)
        validos += 1
    assert validos == 10
    print(f"[OK] Equivalencia 10/10; errores relativos v={max_v:.6f}% omega={max_w:.6f}%")

    banco = BancoRangos()
    assert banco.obtener(10.0)[0] is None, "No debe activar sin sensores"
    for k in ORDER:
        banco.recibir(k, 1.7, 0., 2., 10.)
    valores, estado = banco.obtener(10.1)
    assert estado == "ACTIVO" and valores is not None
    motor.calcular(valores)
    print("[OK] Entradas Range validas: estado ACTIVO")

    banco.recibir("df", float("nan"), 0., 2., 10.15)
    assert banco.obtener(10.2)[0] is None
    banco.recibir("df", float("inf"), 0., 2., 10.2)
    assert banco.obtener(10.2)[0] is None
    banco.recibir("df", 2.3, 0., 2., 10.2)
    assert banco.obtener(10.2)[0] is None
    banco.recibir("df", -0.1, 0., 2., 10.2)
    assert banco.obtener(10.2)[0] is None
    print("[OK] NaN, infinito y distancias fuera de rango bloqueados")

    banco.recibir("df", 1.7, 0., 2., 10.25)
    assert banco.obtener(11.1)[0] is None, "No debe utilizar datos caducados"
    print("[OK] Caducidad de lecturas: fail-safe")
    print("[OK] Los tres sensores son obligatorios; no se reutilizan datos invalidos")
    print("[OK] El nodo no tiene publicador /cmd_vel ni modifica el supervisor")
    print("=== E2.2: AUTOPRUEBA AISLADA 10/10 + FAILSAFE APROBADA ===")


def instalar() -> None:
    print("=== E2.2 - INSTALACION SEGURA Y AUTOPRUEBA ===", flush=True)
    if not (REPO / ".git").is_dir():
        raise RuntimeError("Repositorio ausente")
    branch = subprocess.run(["git", "branch", "--show-current"], cwd=REPO,
                            capture_output=True, text=True, check=True).stdout.strip()
    if branch != "desarrollo-avance2":
        raise RuntimeError(f"Rama inesperada: {branch}")
    check_fis()
    origen = Path(__file__).resolve()
    destino = REPO / "tools/A2_E22_nodo_difuso.py"
    if origen != destino:
        if destino.exists() and destino.read_bytes() != origen.read_bytes():
            raise RuntimeError(f"Ya existe un nodo distinto; no se sobrescribe: {destino}")
        if not destino.exists():
            temporal = destino.with_suffix(".tmp")
            shutil.copyfile(origen, temporal)
            temporal.replace(destino)
    print(f"[OK] Nodo preparado: {destino}", flush=True)
    py = Path.home() / ".venvs/amr-a2-r18/bin/python"
    if not py.is_file():
        raise RuntimeError("Falta entorno E2.1, no se instala nada automaticamente")
    print("[PRUEBA] Sin publicar topicos ni arrancar Gazebo...", flush=True)
    prueba = subprocess.run([str(py), str(destino), "--self-test"], cwd=REPO)
    if prueba.returncode:
        raise RuntimeError("Autoprueba del nodo no aprobada")
    print("[OK] E2.2 listo para prueba ROS 2 aislada. No se hizo git add/commit/push.")


def ejecutar_ros() -> None:
    # Importacion diferida para que --instalar-y-probar no exija iniciar ROS.
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    motor = MotorMamdani()

    class NodoDifuso(Node):
        def __init__(self):
            super().__init__("controlador_difuso_a2")
            self.banco = BancoRangos()
            self.pub = self.create_publisher(Twist, "/cmd_vel_difuso", 10)
            self.pub_estado = self.create_publisher(String, "/estado_difuso", 10)
            self.subs = []
            for key, topic in TOPICS.items():
                self.subs.append(self.create_subscription(
                    Range, topic, lambda msg, k=key: self.recibir(k, msg), 10
                ))
            self.ultimo_estado = None
            self.timer = self.create_timer(PERIOD_S, self.control)
            self.get_logger().info(
                "E2.2 activo (5 Hz). SOLO propuestas en /cmd_vel_difuso; "
                "sin salida /cmd_vel. No existe arbitraje con supervisor."
            )

        def recibir(self, key: str, msg: Range) -> None:
            self.banco.recibir(key, float(msg.range), float(msg.min_range),
                               float(msg.max_range), time.monotonic())

        def control(self) -> None:
            valores, estado = self.banco.obtener(time.monotonic())
            salida = Twist()  # PARADA predeterminada ante invalidos, caducidad o error.
            if valores is not None:
                try:
                    v, w = motor.calcular(valores)
                    salida.linear.x = v
                    salida.angular.z = w
                except Exception as error:
                    estado = "ERROR_INFERENCIA"
                    self.get_logger().error(f"E2.2: {error}")
            self.pub.publish(salida)  # Nunca /cmd_vel.
            self.pub_estado.publish(String(data=estado))
            if estado != self.ultimo_estado:
                self.get_logger().info(f"Estado difuso: {estado}")
                self.ultimo_estado = estado

        def detener_propuesta(self):
            self.pub.publish(Twist())
            self.pub_estado.publish(String(data="DETENIDO"))

    rclpy.init()
    nodo = None
    try:
        nodo = NodoDifuso()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo is not None:
            for _ in range(3):
                nodo.detener_propuesta()
                time.sleep(0.05)
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    p = argparse.ArgumentParser(description="Nodo Mamdani E2.2: solo propuestas")
    grupo = p.add_mutually_exclusive_group(required=True)
    grupo.add_argument("--instalar-y-probar", action="store_true")
    grupo.add_argument("--self-test", action="store_true")
    grupo.add_argument("--ros", action="store_true")
    args, resto = p.parse_known_args()  # deja --ros-args al entorno ROS2
    try:
        if args.instalar_y_probar:
            instalar()
        elif args.self_test:
            probar()
        else:
            # rclpy.init() lee por defecto sys.argv; excluir flags personalizados.
            sys.argv = [sys.argv[0], *resto]
            ejecutar_ros()
        return 0
    except Exception as exc:
        print(f"[ERROR E2.2] {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
