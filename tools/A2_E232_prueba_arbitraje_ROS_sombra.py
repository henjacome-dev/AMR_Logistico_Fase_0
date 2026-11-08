#!/usr/bin/env python3
"""AMR Logistico / E2.3.2: banco ROS 2 para arbitraje en modo sombra.

Ejecucion WSL2:
 python3 /mnt/c/Users/Henry/Downloads/A2_E232_prueba_arbitraje_ROS_sombra.py

Lanza una instancia del arbitro en un DOMINIO ROS 2 AISLADO, publica Range,
Odometry y Twist sinteticos, observa SOLO /cmd_vel_arbitraje_sombra y
/estado_arbitraje_sombra. Nunca publica /cmd_vel, no inicia Gazebo y no toca
supervisor_f13.py ni el FIS. Prueba de comunicaciones: no es integracion
operativa R19 ni implica seguridad fisica.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

REPO = Path.home() / "AMR_Logistico_Fase_0"
VENV_PY = Path.home() / ".venvs/amr-a2-r18/bin/python"
ACTIVACION = REPO / "tools/activar_entorno.sh"
SOMBRA = REPO / "tools/A2_E23_arbitraje_sombra.py"
FIS = REPO / "matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis"
EXPECTED_FIS_BLOB = "6c7dfdd083728d49c64dd1c90a59b19d63222063"
RANGES = {"df": "/rango_frontal", "di": "/rango_izq", "dd": "/rango_der"}
SENSOR_PERIOD = .07
NODE_PERIOD = .10


def require(check: bool, detail: str):
    if not check:
        raise RuntimeError(detail)


def check_repo():
    require((REPO / ".git").is_dir(), f"Repositorio ausente: {REPO}")
    for p in (VENV_PY, ACTIVACION, SOMBRA, FIS):
        require(p.is_file(), f"Falta: {p}")
    data = FIS.read_bytes()
    blob = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
    require(blob == EXPECTED_FIS_BLOB, "El FIS original fue modificado")
    branch = subprocess.run(["git", "branch", "--show-current"], cwd=REPO,
                            capture_output=True, text=True, check=True).stdout.strip()
    require(branch == "desarrollo-avance2", f"Rama inesperada: {branch}")
    # No se sobreescribe el script local de E2.3.1 ni se exige worktree limpio.


def prepare_runtime():
    check_repo()
    if os.environ.get("_A2_E232_RUNTIME") == "1":
        return
    # Asignar dominio antes de crear cualquier nodo o lanzar ROS.
    domain = 200 + os.getpid() % 25
    command = (
        "source " + shlex.quote(str(ACTIVACION)) + " >/dev/null && "
        + "export _A2_E232_RUNTIME=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=" + str(domain)
        + " && exec " + shlex.quote(str(VENV_PY)) + " -u "
        + shlex.quote(str(Path(__file__).resolve())) + " --banco"
    )
    print("[INFO] Activando ROS 2 Humble y Python del proyecto...", flush=True)
    result = subprocess.run(["bash", "-c", command], cwd=REPO, check=False)
    raise SystemExit(result.returncode)


def import_shadow():
    if str(SOMBRA.parent) not in sys.path:
        sys.path.insert(0, str(SOMBRA.parent))
    from A2_E23_arbitraje_sombra import (
        Config, Reading, Proposal, Frame, ShadowArbiter,
    )
    return Config, Reading, Proposal, Frame, ShadowArbiter


def execute_node():
    # Prohibir inicio directo fuera del dominio aislado creado por el banco.
    require(os.environ.get("_A2_E232_RUNTIME") == "1",
            "El nodo debe iniciarse exclusivamente desde el banco aislado")
    require(os.environ.get("ROS_LOCALHOST_ONLY") == "1" and
            200 <= int(os.environ.get("ROS_DOMAIN_ID", "-1")) <= 224,
            "Dominio de prueba no aislado")
    # TODOS los publicadores de este proceso son solo topicos de sombra.
    Config, Reading, Proposal, Frame, ShadowArbiter = import_shadow()
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    class ArbitradorSombraROS(Node):
        def __init__(self):
            super().__init__("arbitro_sombra_e23")
            self.engine = ShadowArbiter(Config())
            self.readings = {k: None for k in RANGES}
            self.odom_received = None
            self.proposals = {"RUTA": None, "DIFUSO": None}
            self.out = self.create_publisher(Twist, "/cmd_vel_arbitraje_sombra", 10)
            self.states = self.create_publisher(String, "/estado_arbitraje_sombra", 10)
            self.subs = []
            for k, topic in RANGES.items():
                self.subs.append(self.create_subscription(
                    Range, topic, lambda msg, key=k: self.on_range(key, msg), 10))
            self.subs.append(self.create_subscription(Odometry, "/odom", self.on_odom, 10))
            self.subs.append(self.create_subscription(
                Twist, "/cmd_vel_ruta", lambda msg: self.on_twist("RUTA", msg), 10))
            self.subs.append(self.create_subscription(
                Twist, "/cmd_vel_difuso", lambda msg: self.on_twist("DIFUSO", msg), 10))
            self.timer = self.create_timer(NODE_PERIOD, self.evaluate)
            self.last_state = None
            self.get_logger().info(
                "E2.3.2 sombra activo: /cmd_vel_arbitraje_sombra, /estado_arbitraje_sombra. "
                "NUNCA publica /cmd_vel.")

        def on_range(self, key, msg):
            now = time.monotonic()
            v, low, high = float(msg.range), float(msg.min_range), float(msg.max_range)
            good = (all(math.isfinite(x) for x in (v, low, high))
                    and 0 <= low <= high <= 2.0 and low <= v <= high)
            self.readings[key] = Reading(v if good else math.nan, now)

        def on_odom(self, msg):
            now = time.monotonic()
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if all(math.isfinite(v) for v in values):
                self.odom_received = now
            else:
                self.odom_received = None

        def on_twist(self, mode, msg):
            now = time.monotonic()
            v, w = float(msg.linear.x), float(msg.angular.z)
            extra = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
            if any(not math.isfinite(x) or abs(x) > 1e-6 for x in extra):
                v, w = math.nan, math.nan
            self.proposals[mode] = Proposal(v, w, now)

        def evaluate(self):
            now = time.monotonic()
            frame = Frame(self.readings["df"], self.readings["di"],
                          self.readings["dd"], self.odom_received,
                          self.proposals["RUTA"], self.proposals["DIFUSO"])
            result = self.engine.decide(frame, now)
            output = Twist()
            output.linear.x = result.v
            output.angular.z = result.w
            self.out.publish(output)
            self.states.publish(String(data=result.mode + "|" + result.reason))
            status = result.mode + "|" + result.reason
            if status != self.last_state:
                self.get_logger().info(status + f" v={result.v:.3f} w={result.w:.3f}")
                self.last_state = status

    rclpy.init()
    node = None
    try:
        node = ArbitradorSombraROS()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            msg = Twist()
            for _ in range(3):
                node.out.publish(msg)
                node.states.publish(String(data="DETENIDO|SALIDA"))
                time.sleep(.025)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_bench():
    check_repo()
    require(os.environ.get("ROS_LOCALHOST_ONLY") == "1", "ROS_LOCALHOST_ONLY no establecido")
    domain = int(os.environ["ROS_DOMAIN_ID"])
    require(200 <= domain <= 224, "Dominio fuera del intervalo de ensayo")

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Range
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    when = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    evidence = REPO / "resultados" / "A2_E232_arbitraje_ros_sombra" / when
    j = 1
    while evidence.exists():
        evidence = evidence.parent / (when + f"_{j:02d}")
        j += 1
    evidence.mkdir(parents=True)
    logfile = evidence / "nodo_sombra.log"
    reportfile = evidence / "resumen.json"
    csvfile = evidence / "observaciones.csv"
    status_events = []
    velocity_events = []
    observations = []
    results = []
    child = None
    handle = None
    node = None
    started = time.monotonic()
    initialized = False
    successful = False

    print("=== E2.3.2 - ARBITRAJE CON ROS 2 REAL EN MODO SOMBRA ===", flush=True)
    print(f"[INFO] ROS_DOMAIN_ID={domain}; ROS_LOCALHOST_ONLY=1", flush=True)
    print("[INFO] Range, Odometry y Twist sinteticos; SIN Gazebo y SIN /cmd_vel", flush=True)
    print("[INFO] Umbrales .60/.80 m provisorios, sujetos a R19", flush=True)

    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = Node("banco_e232_aislado")
        pub_ranges = {k: node.create_publisher(Range, top, 10)
                      for k, top in RANGES.items()}
        pub_odom = node.create_publisher(Odometry, "/odom", 10)
        pub_route = node.create_publisher(Twist, "/cmd_vel_ruta", 10)
        pub_fuzzy = node.create_publisher(Twist, "/cmd_vel_difuso", 10)
        node.create_subscription(String, "/estado_arbitraje_sombra", lambda m: status_events.append((time.monotonic(), m.data)), 10)
        node.create_subscription(Twist, "/cmd_vel_arbitraje_sombra", lambda m: velocity_events.append((time.monotonic(), float(m.linear.x), float(m.angular.z))), 10)

        handle = logfile.open("w", encoding="utf8")
        child = subprocess.Popen([str(VENV_PY), "-u", str(Path(__file__).resolve()), "--nodo"],
                                 cwd=REPO, env=os.environ.copy(), stdout=handle,
                                 stderr=subprocess.STDOUT)
        limit = time.monotonic() + 18.0
        while time.monotonic() < limit:
            rclpy.spin_once(node, timeout_sec=.05)
            if child.poll() is not None:
                raise RuntimeError("El nodo terminó durante el descubrimiento. Revisar nodo_sombra.log")
            if ("arbitro_sombra_e23" in node.get_node_names()
                and all(node.count_subscribers(topic) >= 1 for topic in RANGES.values())
                and node.count_subscribers("/odom") >= 1
                and node.count_subscribers("/cmd_vel_ruta") >= 1
                and node.count_subscribers("/cmd_vel_difuso") >= 1
                and node.count_publishers("/estado_arbitraje_sombra") >= 1
                and node.count_publishers("/cmd_vel_arbitraje_sombra") >= 1):
                break
        else:
            raise RuntimeError("No se descubrieron las ocho interfaces del nodo")
        require(node.count_publishers("/cmd_vel") == 0,
                "Detectado publicador inesperado en /cmd_vel")
        print("[OK] 8 interfaces ROS descubiertas; /cmd_vel sin publicadores", flush=True)
        results.append({"caso": "Interfaces ROS y aislamiento", "aprobado": True})

        def publish_range(name, value):
            msg = Range()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_scan"
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = math.radians(15.)
            msg.min_range = 0.
            msg.max_range = 2.
            msg.range = float(value)
            pub_ranges[name].publish(msg)

        def publish_odom():
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_footprint"
            msg.pose.pose.orientation.w = 1.
            pub_odom.publish(msg)

        def publish_twist(publisher, v, w, *, lateral=0.):
            msg = Twist()
            msg.linear.x = float(v)
            msg.linear.y = float(lateral)
            msg.angular.z = float(w)
            publisher.publish(msg)

        def phase(name, duration, *, ranges=None, odom=True, route=(.10, 0.),
                  fuzzy=(.16, .20), lateral_route=0.):
            start = time.monotonic()
            next_send = start
            while time.monotonic() < start + duration:
                now = time.monotonic()
                if now >= next_send:
                    if ranges is not None:
                        for k, val in ranges.items():
                            publish_range(k, val)
                    if odom:
                        publish_odom()
                    if route is not None:
                        publish_twist(pub_route, *route, lateral=lateral_route)
                    if fuzzy is not None:
                        publish_twist(pub_fuzzy, *fuzzy)
                    next_send = now + SENSOR_PERIOD
                rclpy.spin_once(node, timeout_sec=.035)
                if child.poll() is not None:
                    raise RuntimeError(f"El nodo terminó durante {name}. Consultar {logfile}")
            # Excluir muestras de transición, exigir que las últimas sean estables.
            trim = time.monotonic() - min(.30, duration * .30)
            statuses = [s for t, s in status_events if t >= trim]
            speeds = [(v, w) for t, v, w in velocity_events if t >= trim]
            require(len(statuses) >= 2 and len(speeds) >= 2,
                    f"{name}: no hay suficientes mensajes ROS finales")
            return statuses, speeds

        def check(name, statuses, speeds, mode, reason, v, w, tol=1e-4):
            desired = mode + "|" + reason
            good = (all(s == desired for s in statuses)
                    and all(abs(x - v) <= tol and abs(y - w) <= tol for x,y in speeds))
            results.append({"caso": name, "aprobado": bool(good),
                            "estado": statuses[-1], "v": speeds[-1][0], "omega": speeds[-1][1],
                            "esperado": desired})
            for s in statuses:
                observations.append({"caso": name, "tipo": "estado", "valor": s})
            for x, y in speeds:
                observations.append({"caso": name, "tipo": "twist", "valor": f"v={x:.6f};w={y:.6f}"})
            print(("[OK] " if good else "[ERROR] ") + name + ": " +
                  statuses[-1] + f" / v={speeds[-1][0]:.3f} w={speeds[-1][1]:+.3f}", flush=True)
            require(good, "Fallo: " + name + f". Esperado {desired} ({v},{w})")

        statuses, speeds = phase("inicio sin datos", .85, ranges=None, odom=False, route=None, fuzzy=None)
        check("Sin sensores, parada cero", statuses, speeds, "DETENIDO", "SIN_DATO_df", 0, 0)
        statuses, speeds = phase("ruta libre", 1.10, ranges=dict(df=1.7, di=1.7, dd=1.7))
        check("Pasillo libre: RUTA", statuses, speeds, "RUTA", "PROPUESTA_RUTA_VALIDA", .10, 0)
        statuses, speeds = phase("difuso", 1.10, ranges=dict(df=.55, di=1.7, dd=1.7))
        check("Frontal .55 m: DIFUSO", statuses, speeds, "DIFUSO", "PROPUESTA_DIFUSO_VALIDA", .12, .20)
        statuses, speeds = phase("histeresis", .85, ranges=dict(df=.70, di=1.7, dd=1.7))
        check("Histeresis .70 m: conserva DIFUSO", statuses, speeds, "DIFUSO", "PROPUESTA_DIFUSO_VALIDA", .12, .20)
        statuses, speeds = phase("retorno ruta", .85, ranges=dict(df=.80, di=1.7, dd=1.7))
        check("Frontal .80 m: retorna RUTA", statuses, speeds, "RUTA", "PROPUESTA_RUTA_VALIDA", .10, 0)
        statuses, speeds = phase("limite frontal", .85, ranges=dict(df=.35, di=1.7, dd=1.7))
        check("Parada estricta a .35 m", statuses, speeds, "DETENIDO", "PARADA_FRONTAL_ESTRICTA", 0, 0)
        statuses, speeds = phase("lectura caducada", 1.40, ranges=None, odom=True)
        check("Caducidad de Range: parada", statuses, speeds, "DETENIDO", "CADUCADO_df", 0, 0)
        statuses, speeds = phase("recuperacion", .95, ranges=dict(df=.55, di=1.7, dd=1.7))
        check("Recuperacion con tres Range", statuses, speeds, "DIFUSO", "PROPUESTA_DIFUSO_VALIDA", .12, .20)
        statuses, speeds = phase("nan frontal", .85, ranges=dict(df=float('nan'), di=1.7, dd=1.7))
        check("NaN frontal: parada", statuses, speeds, "DETENIDO", "INVALIDO_df", 0, 0)
        statuses, speeds = phase("propuesta difusa caducada", 1.0,
                                 ranges=dict(df=.55, di=1.7, dd=1.7), fuzzy=None)
        check("Propuesta DIFUSO caducada", statuses, speeds,
              "DETENIDO", "PROPUESTA_DIFUSO_AUSENTE_O_CADUCADA", 0, 0)
        statuses, speeds = phase("odom caducada", 2.0,
                                 ranges=dict(df=1.7, di=1.7, dd=1.7), odom=False)
        check("Odometria caducada", statuses, speeds,
              "DETENIDO", "ODOMETRIA_AUSENTE_O_CADUCADA", 0, 0)
        statuses, speeds = phase("componente prohibido", 1.0,
                                 ranges=dict(df=1.7, di=1.7, dd=1.7), lateral_route=.05)
        check("Twist lateral no autorizado", statuses, speeds,
              "DETENIDO", "PROPUESTA_RUTA_INVALIDA", 0, 0)
        require(node.count_publishers("/cmd_vel") == 0,
                "Se detectó publicación /cmd_vel durante el ensayo")
        print("[OK] Aislamiento confirmado: /cmd_vel sigue sin publicadores", flush=True)
        results.append({"caso": "Ausencia de /cmd_vel al final", "aprobado": True})

        successful = all(item["aprobado"] for item in results)
        require(successful and len(results) == 14, "Recuento de verificaciones inesperado")
        print(f"[RESULTADO] {len(results)}/{len(results)} verificaciones aprobadas", flush=True)
        print("=== E2.3.2: ARBITRAJE ROS 2 EN SOMBRA APROBADO ===", flush=True)
        return 0
    except Exception as exc:
        print("[ERROR E2.3.2] " + str(exc), flush=True)
        return 1
    finally:
        if child is not None:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=4.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=3.)
        if handle is not None:
            handle.close()
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        meta = {
            "aprobado": successful,
            "fecha": dt.datetime.now().isoformat(timespec="seconds"),
            "duracion_s": round(time.monotonic() - started, 3),
            "ros_domain_id": domain,
            "fis_blob": EXPECTED_FIS_BLOB,
            "alcance": "ROS 2 real con mensajes sinteticos, sin Gazebo, sin /cmd_vel; NO acredita R19 operativo.",
            "umbrales": {"stop_m": .35, "enter_fuzzy_m": .60, "exit_fuzzy_m": .80},
            "tests": results,
            "muestras_estado": len(status_events),
            "muestras_sombra": len(velocity_events),
        }
        reportfile.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf8")
        with csvfile.open("w", newline="", encoding="utf8") as f:
            writer = csv.DictWriter(f, fieldnames=["caso", "tipo", "valor"], lineterminator="\n")
            writer.writeheader()
            writer.writerows(observations)
        print("[EVIDENCIA] " + str(reportfile), flush=True)
        print("[EVIDENCIA] " + str(csvfile), flush=True)
        print("[LOG] " + str(logfile), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--banco", action="store_true", help=argparse.SUPPRESS)
    mode.add_argument("--nodo", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.nodo:
        check_repo()
        execute_node()
        return 0
    if args.banco:
        return run_bench()
    prepare_runtime()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception as exc:
        print("[ERROR E2.3.2] " + str(exc), flush=True)
        sys.exit(1)
