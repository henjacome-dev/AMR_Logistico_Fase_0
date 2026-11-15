#!/usr/bin/env python3
"""A3/E4.1: validacion ROS aislada del arbitraje Ruta/Mamdani v03."""
import collections
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = "67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903"
FIS = ROOT / "matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis"
VENV = Path.home() / ".venvs/amr-a2-r18/bin/python"
CONTROLLER = ROOT / "tools/A3_nodo_mamdani_v03.py"
SAFE_TOPIC = "/cmd_vel_auditoria_e41"

assert os.environ.get("ROS_DOMAIN_ID") == "174", "Usar ROS_DOMAIN_ID=174"
assert os.environ.get("ROS_LOCALHOST_ONLY") == "1", "Usar ROS_LOCALHOST_ONLY=1"
assert subprocess.check_output(["git", "branch", "--show-current"], cwd=ROOT, text=True).strip() == "integracion-final"
assert FIS.is_file() and hashlib.sha256(FIS.read_bytes()).hexdigest() == EXPECTED
assert VENV.is_file() and CONTROLLER.is_file()

SALIDA = ROOT / "resultados/A3_E41_arbitraje_v03" / ("ros_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
SALIDA.mkdir(parents=True, exist_ok=False)

# Ejecuta la clase real SupervisorU3S7 con remapeo SOLO en su proceso;
# se evita el chequeo use_sim_time del ejecutable principal porque este
# banco no tiene un reloj /clock ni Gazebo.
CODIGO_SUP = '''
import rclpy
from tools.supervisor_u3s7_cnn import SupervisorU3S7
rclpy.init(args=["--ros-args", "-r", "/cmd_vel:=/cmd_vel_auditoria_e41"])
n = SupervisorU3S7()
try:
    rclpy.spin(n)
finally:
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
'''

procesos = {}
logs = {}
nodo = None
ok = {}
error = None
estados = []
comandos = []
propuestas = []

try:
    rclpy.init()
    nodo = Node("auditoria_arbitraje_e41")
    assert nodo.count_publishers("/cmd_vel") == 0, "Existe publicador real en este dominio"

    entradas = {
        "df": nodo.create_publisher(Range, "/rango_frontal", 10),
        "di": nodo.create_publisher(Range, "/rango_izq", 10),
        "dd": nodo.create_publisher(Range, "/rango_der", 10),
    }
    pub_scan = nodo.create_publisher(LaserScan, "/scan", 10)
    pub_odom = nodo.create_publisher(Odometry, "/odom", 10)
    pub_ruta = nodo.create_publisher(Twist, "/cmd_vel_ruta", 10)

    subs = [
        nodo.create_subscription(String, "/estado_supervisor", lambda m: estados.append((time.monotonic(), json.loads(m.data))), 10),
        nodo.create_subscription(Twist, SAFE_TOPIC, lambda m: comandos.append((time.monotonic(), float(m.linear.x), float(m.angular.z))), 10),
        nodo.create_subscription(Twist, "/cmd_vel_difuso", lambda m: propuestas.append((time.monotonic(), float(m.linear.x), float(m.angular.z))), 10),
    ]

    def arrancar(nombre, args):
        archivo = (SALIDA / (nombre + ".log")).open("w", encoding="utf-8")
        logs[nombre] = archivo
        procesos[nombre] = subprocess.Popen(args, cwd=ROOT, env={**os.environ, "PYTHONUNBUFFERED": "1"}, stdout=archivo, stderr=subprocess.STDOUT)
        print("[INICIADO]", nombre, "PID", procesos[nombre].pid, flush=True)

    arrancar("supervisor", [sys.executable, "-u", "-c", CODIGO_SUP])
    arrancar("mamdani_v03", [str(VENV), "-u", str(CONTROLLER), "--ros"])

    print("=== DESCUBRIMIENTO ROS 2 AISLADO ===", flush=True)
    plazo = time.monotonic() + 18
    while time.monotonic() < plazo:
        rclpy.spin_once(nodo, timeout_sec=0.06)
        assert all(p.poll() is None for p in procesos.values()), "Un proceso termino antes de iniciar"
        if (nodo.count_publishers(SAFE_TOPIC) == 1
            and nodo.count_publishers("/cmd_vel_difuso") == 1
            and nodo.count_publishers("/estado_supervisor") == 1
            and nodo.count_subscribers("/cmd_vel_ruta") == 1
            and all(nodo.count_subscribers(t) >= 2 for t in ("/rango_frontal", "/rango_izq", "/rango_der"))):
            break
    else:
        raise TimeoutError("No se descubrieron todas las interfaces")
    assert nodo.count_publishers("/cmd_vel") == 0, "Publicador de movimiento NO AISLADO"
    print("[OK] Supervisor remapeado; /cmd_vel tiene 0 publicadores", flush=True)

    def enviar(lecturas, scan_frente, con_ruta, con_sensores):
        if con_sensores:
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = -3.0
            odom.pose.pose.position.y = -2.8
            odom.pose.pose.orientation.w = 1.0
            pub_odom.publish(odom)

            scan = LaserScan()
            scan.header.frame_id = "base_scan"
            scan.angle_min = -math.pi
            scan.angle_increment = 2 * math.pi / 360
            scan.angle_max = math.pi - scan.angle_increment
            scan.range_min = 0.1
            scan.range_max = 3.5
            scan.ranges = [float(scan_frente)] * 360
            pub_scan.publish(scan)

            if lecturas is not None:
                for k, v in lecturas.items():
                    msg = Range()
                    msg.header.frame_id = "base_scan"
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = math.radians(15)
                    msg.min_range = 0.0
                    msg.max_range = 2.0
                    msg.range = float(v)
                    entradas[k].publish(msg)

        if con_ruta:
            msg = Twist()
            msg.linear.x = 0.08
            msg.angular.z = 0.10
            pub_ruta.publish(msg)

    def fase(nombre, modo, lecturas=None, scan=1.7, ruta=True, sensores=True, duracion=1.55):
        comienzo = time.monotonic()
        proximo = comienzo
        while time.monotonic() - comienzo < duracion:
            if time.monotonic() >= proximo:
                enviar(lecturas, scan, ruta, sensores)
                proximo = time.monotonic() + 0.07
            rclpy.spin_once(nodo, timeout_sec=0.035)
            assert all(p.poll() is None for p in procesos.values()), "Proceso finalizado durante " + nombre
            assert nodo.count_publishers("/cmd_vel") == 0, "Salida sin aislamiento"
        corte = comienzo + duracion - 0.40
        e = [x for t, x in estados if t >= corte]
        c = [(v, w) for t, v, w in comandos if t >= corte]
        assert len(e) >= 2 and len(c) >= 2, (nombre, "sin muestras", len(e), len(c))
        assert all(x["modo"] == modo for x in e[-2:]), (nombre, [x["modo"] for x in e[-4:]])
        if modo == "DETENIDO":
            assert all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in c[-2:]), (nombre, c[-2:])
        elif modo == "RUTA":
            assert all(abs(v - 0.08) < 0.002 and abs(w - 0.10) < 0.002 for v, w in c[-2:]), (nombre, c[-2:])
        elif modo == "DIFUSO":
            recientes = [(v,w) for t,v,w in propuestas if t >= corte]
            assert recientes, (nombre, "sin propuestas del motor")
            v_fis, w_fis = recientes[-1]
            v_esperada = min(0.10, v_fis)
            w_esperada = max(-0.70, min(0.70, w_fis))
            assert all(abs(v - v_esperada) < 0.005 and abs(w - w_esperada) < 0.025 for v,w in c[-2:]), (nombre, c[-2:], recientes[-1])
            if nombre == "DIFUSO_LATERAL_IZQ":
                assert w_esperada < -0.1, "No gira alejandose del obstaculo izquierdo"
        assert all(0 <= v <= 0.1201 and abs(w) <= 0.7001 for v,w in c), (nombre, c)
        ok[nombre] = {"modo": modo, "motivo": e[-1]["motivo"], "v": round(c[-1][0], 4), "omega": round(c[-1][1], 4)}
        print(f"[OK {len(ok):02d}] {nombre}: {modo}; v={c[-1][0]:.4f}; omega={c[-1][1]:+.4f}", flush=True)

    print("\n=== ARBITRAJE RUTA / MAMDANI V03 ===", flush=True)
    fase("SIN_RANGE", "DETENIDO", None)
    fase("RUTA_NOMINAL", "RUTA", {"df":1.7, "di":1.7, "dd":1.7})
    fase("DIFUSO_FRONTAL", "DIFUSO", {"df":0.50, "di":1.7, "dd":1.7}, scan=0.50)
    fase("DIFUSO_LATERAL_IZQ", "DIFUSO", {"df":1.7, "di":0.25, "dd":1.7})
    fase("PARADA_CRITICA", "DETENIDO", {"df":0.20, "di":1.7, "dd":1.7}, scan=0.20)
    fase("RECUPERACION", "RUTA", {"df":1.7, "di":1.7, "dd":1.7})
    fase("PROPUESTA_RUTA_AUSENTE", "DETENIDO", {"df":1.7, "di":1.7, "dd":1.7}, ruta=False)
    fase("SENSORES_CADUCADOS", "DETENIDO", None, sensores=False, ruta=True, duracion=2.1)
    print("\n[OK] 8/8 pruebas de arbitraje real ROS 2", flush=True)
    print("[OK] Motor Mamdani v03 y supervisor operativo, sin actuacion", flush=True)

except (Exception, KeyboardInterrupt) as exc:
    error = f"{type(exc).__name__}: {exc}"
    print("[ERROR]", error, flush=True)
finally:
    for nombre in ("mamdani_v03", "supervisor"):
        p = procesos.get(nombre)
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
    for f in logs.values():
        f.close()
    if nodo is not None:
        nodo.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    (SALIDA / "resumen.json").write_text(json.dumps({
        "aprobado": error is None and len(ok) == 8,
        "fis_sha256": EXPECTED,
        "domain": 174,
        "pruebas": ok,
        "error": error,
        "nota": "Prueba con sensores sinteticos; no acredita navegacion ni colisiones en Gazebo"
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("[EVIDENCIA]", SALIDA / "resumen.json", flush=True)

if error:
    for nombre in ("supervisor", "mamdani_v03"):
        p = SALIDA / (nombre + ".log")
        if p.is_file():
            print(f"--- Ultimas lineas de {nombre} ---")
            print("\n".join(p.read_text(errors="replace").splitlines()[-10:]))
    sys.exit(1)
