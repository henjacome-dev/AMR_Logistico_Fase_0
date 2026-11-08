#!/usr/bin/env python3
"""E2.2: prueba de integración ROS 2 sin Gazebo ni movimiento.

Se ejecuta desde Descargas de Windows. Carga ROS 2 y el venv de E2.1,
usa dominio ROS independiente, lanza el nodo de propuestas, publica
sensor_msgs/Range sintéticos y comprueba Twist y estado. No hace git add,
commit, push ni publica /cmd_vel.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

REPO = (Path(__file__).resolve().parents[1] if Path(__file__).resolve().parent.name == "tools" else Path.home() / "AMR_Logistico_Fase_0")
ACTIVACION = REPO / "tools/activar_entorno.sh"
VENVPY = Path.home() / ".venvs/amr-a2-r18/bin/python"
NODO = REPO / "tools/A2_E22_nodo_difuso.py"
FIS = REPO / "matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis"
FIS_BLOB = "6c7dfdd083728d49c64dd1c90a59b19d63222063"
TOPICOS = {"df": "/rango_frontal", "di": "/rango_izq", "dd": "/rango_der"}


def asegurar(condicion, mensaje):
    if not condicion:
        raise RuntimeError(mensaje)


def cargar_entorno():
    asegurar(REPO.joinpath(".git").is_dir(), "Repositorio no encontrado")
    asegurar(ACTIVACION.is_file() and VENVPY.is_file() and NODO.is_file(),
            "Falta entorno ROS 2, venv E2.1 o nodo E2.2")
    contenido = FIS.read_bytes()
    sha = hashlib.sha1(b"blob " + str(len(contenido)).encode() + b"\0" + contenido).hexdigest()
    asegurar(sha == FIS_BLOB, "FIS v01 no coincide con la version inmutable")
    if os.environ.get("_A2_E22_ENTORNO_CARGADO") == "1":
        return
    shell = ("source " + shlex.quote(str(ACTIVACION)) +
             " >/dev/null && export _A2_E22_ENTORNO_CARGADO=1 && exec " +
             shlex.quote(str(VENVPY)) + " " + shlex.quote(str(Path(__file__).resolve())))
    print("[INFO] Activando ROS 2 Humble y entorno Python R18...", flush=True)
    code = subprocess.call(["bash", "-c", shell], cwd=REPO)
    sys.exit(code)


def ejecutar():
    cargar_entorno()
    # Se configura antes de importar/inicializar rclpy. No comunica con el
    # dominio habitual de Gazebo y evita interferir con robots en ejecucion.
    dominio = str(150 + (os.getpid() % 50))
    os.environ["ROS_DOMAIN_ID"] = dominio
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    print("=== E2.2: PRUEBA ROS 2 AISLADA ===", flush=True)
    print(f"[INFO] ROS_DOMAIN_ID={dominio}; ROS_LOCALHOST_ONLY=1", flush=True)
    print("[INFO] Sin Gazebo, sin /cmd_vel y sin modificaciones del supervisor.", flush=True)

    fecha = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    salida = REPO / "resultados" / "A2_E22_prueba_ros_aislada" / fecha
    salida.mkdir(parents=True, exist_ok=False)
    log_node = salida / "nodo_ros.log"
    resumen_path = salida / "resumen.json"
    estados = []
    giros = []
    checks = {}
    proc = None
    nodo = None
    loghandle = None
    rclpy_iniciado = False
    tiempo_inicio = time.monotonic()

    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        rclpy_iniciado = True
        nodo = Node("a2_e22_banco_pruebas")
        pubs = {k: nodo.create_publisher(Range, topic, 10) for k, topic in TOPICOS.items()}

        def estado_cb(msg: String):
            estados.append((time.monotonic(), msg.data))

        def twist_cb(msg: Twist):
            giros.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

        nodo.create_subscription(String, "/estado_difuso", estado_cb, 10)
        nodo.create_subscription(Twist, "/cmd_vel_difuso", twist_cb, 10)

        loghandle = log_node.open("w", encoding="utf-8")
        proc = subprocess.Popen([str(VENVPY), str(NODO), "--ros"], cwd=REPO,
                                env=os.environ.copy(), stdout=loghandle,
                                stderr=subprocess.STDOUT)

        # Descubrimiento de nodo y todas sus interfaces.
        limite = time.monotonic() + 20.0
        while time.monotonic() < limite:
            rclpy.spin_once(nodo, timeout_sec=0.08)
            if proc.poll() is not None:
                raise RuntimeError("El nodo terminó anticipadamente; revisar nodo_ros.log")
            if ("controlador_difuso_a2" in nodo.get_node_names() and
                all(nodo.count_subscribers(p) >= 1 for p in TOPICOS.values()) and
                nodo.count_publishers("/cmd_vel_difuso") >= 1 and
                nodo.count_publishers("/estado_difuso") >= 1):
                break
        else:
            raise RuntimeError("No se descubrieron las cinco interfaces ROS del nodo")

        asegurar(nodo.count_publishers("/cmd_vel") == 0,
                "Existe un publicador en /cmd_vel dentro del dominio aislado")
        checks["interfaces_ros"] = "OK"
        checks["sin_publicador_cmd_vel"] = "OK"
        print("[OK] Nodo y cinco interfaces ROS descubiertos; /cmd_vel sin publicadores.", flush=True)

        def publicar(key: str, value: float):
            msg = Range()
            msg.header.stamp = nodo.get_clock().now().to_msg()
            msg.header.frame_id = "base_scan"
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = math.radians(15.0)
            msg.min_range = 0.0
            msg.max_range = 2.0
            msg.range = float(value)
            pubs[key].publish(msg)

        def fase(nombre, segundos, valores=None):
            comienzo = time.monotonic()
            prox_envio = comienzo
            while time.monotonic() < comienzo + segundos:
                ahora = time.monotonic()
                if valores is not None and ahora >= prox_envio:
                    for key, value in valores.items():
                        publicar(key, value)
                    prox_envio = ahora + 0.065
                rclpy.spin_once(nodo, timeout_sec=0.045)
                if proc.poll() is not None:
                    raise RuntimeError("Nodo ROS finalizó durante " + nombre)
            # Seleccionar únicamente muestras de esta fase, descartando transición inicial.
            corte = comienzo + min(0.45, segundos * 0.40)
            return ([e for t, e in estados if t >= corte],
                    [(v, w) for t, v, w in giros if t >= corte])

        estado, twist = fase("inicio sin sensores", 0.85)
        asegurar("SIN_DATO_df" in estado and twist and
                all(abs(v) < 1e-8 and abs(w) < 1e-8 for v, w in twist),
                "Sin sensores: no se detectó bloqueo y Twist cero")
        checks["sin_sensores"] = "OK"
        print("[OK] Sin sensores: SIN_DATO_df y propuesta cero.", flush=True)

        estado, twist = fase("sensores válidos", 1.65,
                            {"df": 1.7, "di": 1.7, "dd": 1.7})
        asegurar("ACTIVO" in estado and twist and
                any(abs(v - 0.161111111) < 0.002 and abs(w) < 0.01 for v,w in twist),
                "Tres sensores válidos: no se detectó inferencia correcta")
        checks["tres_sensores_activo"] = "OK"
        print("[OK] Tres Range válidos: ACTIVO, v≈0.16111 m/s y omega≈0.", flush=True)

        estado, twist = fase("caducidad", 1.35)
        asegurar(any(e.startswith("CADUCADO_") for e in estado) and twist and
                all(abs(v) < 1e-8 and abs(w) < 1e-8 for v,w in twist[-3:]),
                "Caducidad: no se detectó parada de propuesta")
        checks["caducidad"] = "OK"
        print("[OK] Sensores caducados: propuesta cero.", flush=True)

        estado, twist = fase("recuperación", 1.1,
                            {"df": 1.7, "di": 1.7, "dd": 1.7})
        asegurar("ACTIVO" in estado and
                any(v > 0.10 for v, w in twist), "No se recuperó tras lecturas válidas")
        checks["recuperacion"] = "OK"
        print("[OK] Recuperación con tres lecturas recientes.", flush=True)

        estado, twist = fase("lectura inválida", 1.2,
                            {"df": float("nan"), "di": 1.7, "dd": 1.7})
        asegurar("INVALIDO_df" in estado and twist and
                all(abs(v) < 1e-8 and abs(w) < 1e-8 for v,w in twist[-3:]),
                "NaN frontal: no se detectó bloqueo")
        checks["nan_frontal"] = "OK"
        print("[OK] Range frontal NaN: INVALIDO_df y propuesta cero.", flush=True)

        asegurar(nodo.count_publishers("/cmd_vel") == 0,
                "Detectado publicador inesperado en /cmd_vel")
        checks["aislamiento_confirmado"] = "OK"
        print("[OK] Aislamiento confirmado: sin publicación en /cmd_vel.", flush=True)
        print("=== E2.2: PRUEBA ROS 2 AISLADA APROBADA ===", flush=True)
        ok = True
        return 0
    except Exception as e:
        print("[ERROR E2.2]", e, flush=True)
        ok = False
        return 1
    finally:
        if proc is not None:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
        if loghandle is not None:
            loghandle.close()
        if nodo is not None:
            nodo.destroy_node()
        if rclpy_iniciado and rclpy.ok():
            rclpy.shutdown()
        resumen = {
            "aprobado": bool(locals().get("ok", False)),
            "fecha": dt.datetime.now().isoformat(timespec="seconds"),
            "duracion_s": round(time.monotonic() - tiempo_inicio, 3),
            "ros_domain_id": dominio,
            "fis_git_blob": FIS_BLOB,
            "checks": checks,
            "muestras_estado": len(estados),
            "muestras_cmd_vel_difuso": len(giros),
            "nota": "Prueba aislada: no equivale a integracion con el supervisor ni a navegacion segura.",
        }
        resumen_path.write_text(json.dumps(resumen, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("[EVIDENCIA]", resumen_path, flush=True)
        print("[LOG]", log_node, flush=True)


if __name__ == "__main__":
    try:
        sys.exit(ejecutar())
    except SystemExit:
        raise
    except Exception as e:
        print("[ERROR E2.2]", e, flush=True)
        sys.exit(1)
