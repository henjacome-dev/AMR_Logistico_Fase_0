#!/usr/bin/env python3
"""F1.3.1: ensayos reproducibles para AMR_Logistico_Fase_0.

Requiere ROS 2 Humble + gazebo_ros (Gazebo Classic), TurtleBot3 Waffle Pi,
y los archivos tools/seguidor_ruta_f13.py y tools/supervisor_f13.py.

Uso (desde la raíz del repositorio, con el entorno activado):
    python3 tools/bateria_navegacion_f131.py --repeticiones 1 --timeout 360

No modifica los nodos existentes. Reinicia la entidad waffle_pi entre ensayos,
registra métricas y restaura el archivo histórico F13_ruta_1_log.csv.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


FIELDS = [
    "t_wall_s", "t_sim_s", "x_m", "y_m", "yaw_rad",
    "indice_segmento", "desviacion_ruta_m", "error_meta_m",
    "laser_min_m", "v_cmd_m_s", "w_cmd_rad_s",
]
SUMMARY = [
    "ruta", "repeticion", "estado", "motivo", "retorno_seguidor",
    "muestras", "tiempo_pared_s", "tiempo_sim_s", "recorrido_real_m",
    "longitud_planificada_m", "error_final_m", "desviacion_max_m",
    "desviacion_rms_m", "laser_min_m", "muestras_proximidad",
    "velocidad_max_abs_m_s", "giro_max_abs_rad_s", "cambios_sentido_giro",
    "ruta_csv", "directorio_evidencia",
]
AGREGADO = ["ruta", "ensayos", "exitos", "tasa_exito_pct", "error_final_prom_m",
            "error_final_max_m", "desviacion_max_m", "tiempo_pared_prom_s"]


def leer_ruta(archivo: Path):
    if not archivo.is_file():
        raise FileNotFoundError(f"No existe la ruta: {archivo}")
    with archivo.open(newline="", encoding="utf-8-sig") as f:
        lector = csv.DictReader(f)
        if not {"x_m", "y_m"}.issubset(set(lector.fieldnames or [])):
            raise ValueError(f"CSV {archivo} necesita encabezados x_m,y_m")
        puntos = [(float(row["x_m"]), float(row["y_m"])) for row in lector]
    if len(puntos) < 2:
        raise ValueError(f"Ruta {archivo} con menos de dos puntos")
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in puntos):
        raise ValueError(f"Ruta {archivo} contiene valores no finitos")
    return puntos


def longitud(puntos):
    return sum(math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(puntos, puntos[1:]))


def desviacion_a_polilinea(x, y, puntos):
    """Distancia al segmento más cercano (no sólo al waypoint más cercano)."""
    mejor_d = float("inf")
    mejor_i = 0
    for i, ((ax, ay), (bx, by)) in enumerate(zip(puntos, puntos[1:])):
        dx, dy = bx - ax, by - ay
        len2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / len2)) if len2 else 0.0
        d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        if d < mejor_d:
            mejor_d, mejor_i = d, i
    return mejor_d, mejor_i


class Observador(Node):
    def __init__(self):
        super().__init__("observador_ensayos_f131")
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._vel, 10)
        self.publicador_parada = self.create_publisher(Twist, "/cmd_vel_ruta", 10)
        self.cliente_borrar = self.create_client(DeleteEntity, "/delete_entity")
        self.cliente_crear = self.create_client(SpawnEntity, "/spawn_entity")
        self.ultima_pose = None
        self.ultima_odom_pared = 0.0
        self.ultimo_scan_pared = 0.0
        self.laser_min = None
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.puntos = None
        self.muestras = []
        self.inicio_pared = None

    def _odom(self, msg):
        ahora = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ultima_pose = (float(p.x), float(p.y), float(yaw))
        self.ultima_odom_pared = ahora
        if self.puntos is None or self.inicio_pared is None:
            return
        desviacion, indice = desviacion_a_polilinea(p.x, p.y, self.puntos)
        objetivo = self.puntos[-1]
        t_sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.muestras.append({
            "t_wall_s": round(ahora - self.inicio_pared, 3),
            "t_sim_s": round(t_sim, 4),
            "x_m": round(p.x, 5), "y_m": round(p.y, 5), "yaw_rad": round(yaw, 5),
            "indice_segmento": indice, "desviacion_ruta_m": round(desviacion, 5),
            "error_meta_m": round(math.hypot(p.x - objetivo[0], p.y - objetivo[1]), 5),
            "laser_min_m": "" if self.laser_min is None else round(self.laser_min, 4),
            "v_cmd_m_s": round(self.v_cmd, 4), "w_cmd_rad_s": round(self.w_cmd, 4),
        })

    def _scan(self, msg):
        validos = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        self.laser_min = min(validos) if validos else None
        self.ultimo_scan_pared = time.monotonic()

    def _vel(self, msg):
        self.v_cmd, self.w_cmd = msg.linear.x, msg.angular.z

    def comenzar(self, puntos):
        self.puntos = puntos
        self.muestras = []
        self.inicio_pared = time.monotonic()

    def terminar(self):
        self.puntos = None
        self.inicio_pared = None
        return list(self.muestras)

    def parar_robot(self):
        # El supervisor es el ÚNICO publicador permitido en /cmd_vel.
        # Enviamos ceros al canal que él consume.
        for _ in range(12):
            self.publicador_parada.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.08)


def esperar_condicion(nodo, predicado, segundos, mensaje):
    limite = time.monotonic() + segundos
    while time.monotonic() < limite:
        rclpy.spin_once(nodo, timeout_sec=0.12)
        if predicado():
            return
    raise TimeoutError(mensaje)


def llamar_servicio(nodo, cliente, solicitud, descripcion, segundos=30.0):
    if not cliente.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Falta servicio Gazebo: {descripcion}")
    futuro = cliente.call_async(solicitud)
    esperar_condicion(nodo, futuro.done, segundos, f"Timeout servicio {descripcion}")
    if futuro.exception() is not None:
        raise RuntimeError(f"Error {descripcion}: {futuro.exception()}")
    respuesta = futuro.result()
    if not respuesta.success:
        raise RuntimeError(f"Gazebo rechazó {descripcion}: {respuesta.status_message}")
    return respuesta


def reiniciar_robot(nodo, xml, x, y, yaw, z, entidad):
    nodo.parar_robot()
    borrar = DeleteEntity.Request()
    borrar.name = entidad
    llamar_servicio(nodo, nodo.cliente_borrar, borrar, "borrar " + entidad)
    # Pequeña pausa para que Gazebo cierre plugins y conexiones antiguas.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        rclpy.spin_once(nodo, timeout_sec=0.1)
    crear = SpawnEntity.Request()
    crear.name = entidad
    crear.xml = xml
    crear.robot_namespace = ""
    crear.reference_frame = "world"
    crear.initial_pose.position.x = x
    crear.initial_pose.position.y = y
    crear.initial_pose.position.z = z
    crear.initial_pose.orientation.z = math.sin(yaw / 2.0)
    crear.initial_pose.orientation.w = math.cos(yaw / 2.0)
    instante = time.monotonic()
    llamar_servicio(nodo, nodo.cliente_crear, crear, "crear " + entidad)
    esperar_condicion(
        nodo,
        lambda: nodo.ultima_odom_pared >= instante and nodo.ultimo_scan_pared >= instante
                and nodo.ultima_pose is not None
                and math.hypot(nodo.ultima_pose[0] - x, nodo.ultima_pose[1] - y) < 0.22,
        25.0,
        f"Tras reiniciar, /odom o /scan no coincide con inicio ({x:.3f},{y:.3f}). "
        "Comprobar nombre de entidad, instalación de plugins y modelo SDF.",
    )
    # Permite estabilizar plugins tras el spawn.
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        rclpy.spin_once(nodo, timeout_sec=0.1)


def guardar_csv(archivo, campos, filas):
    with archivo.open("w", newline="", encoding="utf-8") as f:
        escritor = csv.DictWriter(f, fieldnames=campos, lineterminator="\n")
        escritor.writeheader()
        escritor.writerows(filas)


def obtener_metricas(muestras, puntos, limite_proximidad):
    if not muestras:
        return dict(muestras=0, error_final_m=None, desviacion_max_m=None,
                    desviacion_rms_m=None, laser_min_m=None, recorrido_real_m=0.0,
                    tiempo_sim_s=None, muestras_proximidad=0,
                    velocidad_max_abs_m_s=None, giro_max_abs_rad_s=None,
                    cambios_sentido_giro=0)
    desviaciones = [float(s["desviacion_ruta_m"]) for s in muestras]
    lecturas = [float(s["laser_min_m"]) for s in muestras if s["laser_min_m"] != ""]
    recorrido = sum(math.hypot(float(b["x_m"]) - float(a["x_m"]),
                               float(b["y_m"]) - float(a["y_m"]))
                    for a, b in zip(muestras, muestras[1:]))
    t_sim = float(muestras[-1]["t_sim_s"]) - float(muestras[0]["t_sim_s"])
    cambios = 0
    ultimo_signo = 0
    for s in muestras:
        w = float(s["w_cmd_rad_s"])
        signo = 1 if w > 0.05 else (-1 if w < -0.05 else 0)
        if signo and ultimo_signo and signo != ultimo_signo:
            cambios += 1
        if signo:
            ultimo_signo = signo
    return dict(
        muestras=len(muestras), error_final_m=float(muestras[-1]["error_meta_m"]),
        desviacion_max_m=max(desviaciones),
        desviacion_rms_m=math.sqrt(sum(d * d for d in desviaciones) / len(desviaciones)),
        laser_min_m=min(lecturas) if lecturas else None,
        recorrido_real_m=recorrido,
        tiempo_sim_s=t_sim if t_sim >= 0 else None,
        muestras_proximidad=sum(v < limite_proximidad for v in lecturas),
        velocidad_max_abs_m_s=max(abs(float(s["v_cmd_m_s"])) for s in muestras),
        giro_max_abs_rad_s=max(abs(float(s["w_cmd_rad_s"])) for s in muestras),
        cambios_sentido_giro=cambios,
    )


def agregar_resultados(resultados):
    salida = []
    for numero in sorted({r["ruta"] for r in resultados}):
        grupo = [r for r in resultados if r["ruta"] == numero]
        errores = [r["error_final_m"] for r in grupo if r["error_final_m"] is not None]
        desviaciones = [r["desviacion_max_m"] for r in grupo if r["desviacion_max_m"] is not None]
        tiempos = [r["tiempo_pared_s"] for r in grupo]
        n_ok = sum(r["estado"] == "OK" for r in grupo)
        salida.append(dict(
            ruta=numero, ensayos=len(grupo), exitos=n_ok,
            tasa_exito_pct=round(100.0 * n_ok / len(grupo), 1),
            error_final_prom_m=round(sum(errores) / len(errores), 4) if errores else None,
            error_final_max_m=round(max(errores), 4) if errores else None,
            desviacion_max_m=round(max(desviaciones), 4) if desviaciones else None,
            tiempo_pared_prom_s=round(sum(tiempos) / len(tiempos), 3),
        ))
    return salida


def terminar_proceso(proceso, segundos=6.0):
    if proceso is None or proceso.poll() is not None:
        return
    try:
        os.killpg(proceso.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proceso.wait(timeout=segundos)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proceso.pid, signal.SIGTERM)
            proceso.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proceso.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proceso.wait()


def crear_proceso(comando, archivo_log):
    registro = open(archivo_log, "w", encoding="utf-8", buffering=1)
    try:
        p = subprocess.Popen(comando, stdout=registro, stderr=subprocess.STDOUT,
                             start_new_session=True)
    except Exception:
        registro.close()
        raise
    return p, registro


def realizar_ensayo(nodo, repo, ruta, puntos, numero, repeticion, salida,
                    timeout, tolerancia, distancia_alerta, modelo_xml,
                    entidad, z, log_original):
    nombre = f"ruta_{numero:02d}_rep_{repeticion:02d}"
    carpeta = salida / nombre
    carpeta.mkdir(parents=True, exist_ok=False)
    x0, y0 = puntos[0]
    yaw = math.atan2(puntos[1][1] - y0, puntos[1][0] - x0)
    reiniciar_robot(nodo, modelo_xml, x0, y0, yaw, z, entidad)
    # El seguidor F1.3 escribe por defecto este mismo archivo. Evitar registrar
    # por error datos obsoletos de un ensayo anterior.
    if log_original.exists():
        log_original.unlink()
    comando = [sys.executable, str(repo / "tools/seguidor_ruta_f13.py"),
               "--ruta", str(ruta), "--ros-args", "-p", "use_sim_time:=true"]
    proceso = None
    archivo = None
    expiro = False
    inicio = time.monotonic()
    nodo.comenzar(puntos)
    try:
        proceso, archivo = crear_proceso(comando, carpeta / "salida_seguidor.log")
        while proceso.poll() is None:
            rclpy.spin_once(nodo, timeout_sec=0.1)
            if time.monotonic() - inicio > timeout:
                expiro = True
                terminar_proceso(proceso)
                break
        # Recibir las últimas odometrías antes de evaluar.
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    finally:
        terminar_proceso(proceso)
        nodo.parar_robot()
        muestras = nodo.terminar()
        if archivo:
            archivo.close()

    guardar_csv(carpeta / "odometria.csv", FIELDS, muestras)
    if log_original.exists():
        shutil.copy2(log_original, carpeta / "log_seguidor_original.csv")
    m = obtener_metricas(muestras, puntos, distancia_alerta)
    retorno = proceso.returncode if proceso is not None else -1
    if expiro:
        estado, motivo = "FALLO", "Tiempo de ejecución excedido"
    elif retorno != 0:
        estado, motivo = "FALLO", f"Seguidor terminó con código {retorno}"
    elif m["muestras"] < 10:
        estado, motivo = "FALLO", "Odometría insuficiente"
    elif m["error_final_m"] > tolerancia:
        estado, motivo = "FALLO", "Error final excede tolerancia"
    else:
        estado, motivo = "OK", "Meta alcanzada dentro de tolerancia"
    resultado = dict(
        ruta=numero, repeticion=repeticion, estado=estado, motivo=motivo,
        retorno_seguidor=retorno, muestras=m["muestras"],
        tiempo_pared_s=round(time.monotonic() - inicio, 3),
        tiempo_sim_s=None if m["tiempo_sim_s"] is None else round(m["tiempo_sim_s"], 3),
        recorrido_real_m=round(m["recorrido_real_m"], 3),
        longitud_planificada_m=round(longitud(puntos), 3),
        error_final_m=None if m["error_final_m"] is None else round(m["error_final_m"], 4),
        desviacion_max_m=None if m["desviacion_max_m"] is None else round(m["desviacion_max_m"], 4),
        desviacion_rms_m=None if m["desviacion_rms_m"] is None else round(m["desviacion_rms_m"], 4),
        laser_min_m=None if m["laser_min_m"] is None else round(m["laser_min_m"], 4),
        muestras_proximidad=m["muestras_proximidad"],
        velocidad_max_abs_m_s=m["velocidad_max_abs_m_s"],
        giro_max_abs_rad_s=m["giro_max_abs_rad_s"],
        cambios_sentido_giro=m["cambios_sentido_giro"],
        ruta_csv=str(ruta.relative_to(repo)),
        directorio_evidencia=str(carpeta.relative_to(repo)),
    )
    (carpeta / "resultado.json").write_text(json.dumps(resultado, indent=2, ensure_ascii=False) + "\n",
                                            encoding="utf-8")
    return resultado


def argumentos():
    p = argparse.ArgumentParser(description="F1.3.1 - Batería de navegación autónoma AMR")
    p.add_argument("--preflight", action="store_true",
                   help="Verifica rutas, modelo, ROS y Gazebo sin mover el robot")
    p.add_argument("--repeticiones", type=int, default=1)
    p.add_argument("--timeout", type=float, default=360.0,
                   help="Tiempo máximo de reloj real por ensayo [s]")
    p.add_argument("--tolerancia", type=float, default=0.10,
                   help="Error final máximo para declarar éxito [m]")
    p.add_argument("--alerta-laser", type=float, default=0.23,
                   help="Umbral diagnóstico de proximidad, NO detector de colisión [m]")
    p.add_argument("--entidad", default="waffle_pi")
    p.add_argument("--z", type=float, default=0.05)
    p.add_argument("--repo", type=Path, default=Path.cwd(), help="Raíz del proyecto")
    p.add_argument("--rutas", nargs="+", default=[
        "resultados/F12_ruta_1.csv", "resultados/F12_ruta_2.csv", "resultados/F12_ruta_3.csv"])
    return p.parse_args()


def main():
    args = argumentos()
    repo = args.repo.expanduser().resolve()
    if args.repeticiones < 1 or args.timeout <= 0 or args.tolerancia <= 0:
        raise ValueError("Repeticiones, timeout y tolerancia deben ser positivos")
    for fichero in (repo / "tools/seguidor_ruta_f13.py", repo / "tools/supervisor_f13.py"):
        if not fichero.is_file():
            raise FileNotFoundError(f"Falta archivo requerido: {fichero}")
    rutas = [(i + 1, (repo / ruta).resolve()) for i, ruta in enumerate(args.rutas)]
    puntos_por_ruta = {n: leer_ruta(ruta) for n, ruta in rutas}
    sdf = Path(get_package_share_directory("turtlebot3_gazebo")) / "models/turtlebot3_waffle_pi/model.sdf"
    if not sdf.is_file():
        raise FileNotFoundError(f"Falta modelo SDF de Waffle Pi: {sdf}")
    modelo_xml = sdf.read_text(encoding="utf-8")
    salida = repo / "resultados" / f"F131_bateria_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    archivo_log_original = repo / "resultados" / "F13_ruta_1_log.csv"
    existia_log_original = archivo_log_original.exists()
    respaldo = salida / "_respaldo_F13_ruta_1_log_original.csv"
    if not args.preflight:
        salida.mkdir(parents=True, exist_ok=False)
        if existia_log_original:
            shutil.copy2(archivo_log_original, respaldo)

    rclpy.init(args=None)
    nodo = Observador()
    supervisor = None
    supervisor_archivo = None
    resultados = []
    try:
        if "supervisor_f13" in nodo.get_node_names():
            raise RuntimeError("Ya está ejecutándose supervisor_f13: detén ese proceso antes de iniciar la batería")
        for c, nombre in ((nodo.cliente_borrar, "/delete_entity"),
                          (nodo.cliente_crear, "/spawn_entity")):
            if not c.wait_for_service(timeout_sec=8.0):
                raise RuntimeError(f"No existe {nombre}. Mantén Gazebo Classic abierto con el mundo F1")
        esperar_condicion(nodo, lambda: nodo.ultima_odom_pared > 0 and nodo.ultimo_scan_pared > 0,
                          15, "Sin /odom o /scan: no está listo Gazebo/TurtleBot3")
        otros = [p for p in nodo.get_publishers_info_by_topic("/cmd_vel")
                 if p.node_name != "supervisor_f13"]
        if otros:
            nombres = ", ".join(f"/{p.node_name}" for p in otros)
            raise RuntimeError(f"Hay otro publicador /cmd_vel activo: {nombres}. Detén teleoperación")
        if args.preflight:
            for n, archivo_ruta in rutas:
                print(f"[OK] Ruta {n}: {len(puntos_por_ruta[n])} puntos | "
                      f"{longitud(puntos_por_ruta[n]):.3f} m | {archivo_ruta}")
            print("[OK] Servicios /delete_entity y /spawn_entity, odometría, LiDAR, "
                  "modelo SDF y archivos del repositorio encontrados")
            print("[OK] Preflight: no se han ejecutado cambios en Gazebo")
            return
        supervisor, supervisor_archivo = crear_proceso(
            [sys.executable, str(repo / "tools/supervisor_f13.py"),
             "--ros-args", "-p", "use_sim_time:=true"], salida / "supervisor.log")
        esperar_condicion(nodo,
                          lambda: any(p.node_name == "supervisor_f13"
                                      for p in nodo.get_publishers_info_by_topic("/cmd_vel")),
                          12, "El supervisor no publica /cmd_vel. Revisar supervisor.log")
        print(f"[F1.3.1] Guardando evidencias en {salida}", flush=True)
        print(f"[F1.3.1] {len(rutas)} rutas × {args.repeticiones} repeticiones", flush=True)
        for repeticion in range(1, args.repeticiones + 1):
            for n, ruta in rutas:
                print(f"[ENSAYO] Ruta {n} | repetición {repeticion}: {ruta.name}", flush=True)
                resultado = realizar_ensayo(
                    nodo, repo, ruta, puntos_por_ruta[n], n, repeticion, salida,
                    args.timeout, args.tolerancia, args.alerta_laser,
                    modelo_xml, args.entidad, args.z, archivo_log_original)
                resultados.append(resultado)
                guardar_csv(salida / "resumen.csv", SUMMARY, resultados)
                guardar_csv(salida / "resumen_por_ruta.csv", AGREGADO,
                            agregar_resultados(resultados))
                print(f"[{resultado['estado']}] Error final: {resultado['error_final_m']} m; "
                      f"desviación máxima: {resultado['desviacion_max_m']} m; "
                      f"tiempo: {resultado['tiempo_pared_s']} s", flush=True)
        print("[FINAL] ", sum(r["estado"] == "OK" for r in resultados),
              "/", len(resultados), "ensayos satisfactorios.", flush=True)
        print("[FINAL] Archivo:", salida / "resumen.csv", flush=True)
        print("[NOTA] Proximidad LiDAR NO demuestra ausencia/presencia de colisiones.", flush=True)
    except KeyboardInterrupt:
        print("[INTERRUMPIDO] Ensayos detenidos por usuario.", file=sys.stderr)
        raise SystemExit(130)
    finally:
        if not args.preflight:
            try:
                nodo.parar_robot()
            except Exception:
                pass
        terminar_proceso(supervisor)
        if supervisor_archivo:
            supervisor_archivo.close()
        # Restituir el archivo previamente generado por F1.3, sin perder evidencia.
        if not args.preflight:
            if existia_log_original:
                shutil.copy2(respaldo, archivo_log_original)
            elif archivo_log_original.exists():
                archivo_log_original.unlink()
        nodo.destroy_node()
        rclpy.shutdown()
    if any(r["estado"] != "OK" for r in resultados):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
