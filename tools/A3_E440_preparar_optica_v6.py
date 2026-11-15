#!/usr/bin/env python3
"""Prepara variantes reversibles de Waffle Pi con optica de adquisicion v6.

Solo prepara archivos: NO ejecuta ROS, NO mueve el robot, NO modifica
fuentes instaladas, CNN, FIS ni otras variantes ya existentes.

Configuracion v6 del dataset original: HFOV 145°, pitch +12°, RGB 640x360.
Se usa pose del SENSOR SDF (no la del enlace) como en v6; la transformacion
URDF camera_rgb_joint acompana el pitch para representar el eje optico.

Ejecutar desde el repositorio: python3 tools/A3_E440_preparar_optica_v6.py --preparar
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FIS_SHA = "67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903"
FIS = ROOT / "matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis"
PITCH = -math.radians(12)
FOV = math.radians(145)
OUT = ROOT / "modelos/variantes_opticas"
SDF_OUT = OUT / "waffle_pi_optica_v6.sdf"
URDF_OUT = OUT / "waffle_pi_optica_v6.urdf"
LAUNCH_OUT = ROOT / "launch/almacen_f1_optica_v6.launch.py"
DIAG_OUT = ROOT / "launch/almacen_f1_optica_v6_empaque_diag.launch.py"
MANIFEST = OUT / "waffle_pi_optica_v6_manifest.json"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def prepare_sdf(source: bytes) -> bytes:
    root = ET.fromstring(source)
    check(root.tag == "sdf", "Raiz de SDF inesperada")
    links = [x for x in root.iter("link") if x.get("name") == "camera_rgb_optical_frame"]
    check(len(links) == 1, "No hay exactamente un enlace de camara RGB")
    link = links[0]
    link_pose = link.find("pose")
    check(link_pose is not None and link_pose.text, "Pose de enlace de camara ausente")
    lp = [float(n) for n in link_pose.text.split()]
    check(len(lp) == 6 and abs(lp[0] - .076) < .002 and abs(lp[2] - .093) < .002,
          "Pose de camara de origen inesperada")
    check(max(abs(x) for x in lp[3:]) < .001, "El enlace de camara original ya esta rotado")
    sensors = [s for s in link.findall("sensor") if s.get("type") == "camera"]
    check(len(sensors) == 1 and sensors[0].get("name") == "camera", "Sensor RGB ambiguo")
    sensor = sensors[0]
    pose = sensor.find("pose")
    if pose is None:
        pose = ET.SubElement(sensor, "pose")
        v = [0.0] * 6
    else:
        v = [float(n) for n in (pose.text or "").split()]
        check(len(v) == 6 and max(abs(x) for x in v[3:]) < .001,
              "Sensor RGB original rotado inesperadamente")
    v[4] = PITCH
    pose.text = " ".join(f"{n:.12f}" for n in v)
    camera = sensor.find("camera")
    check(camera is not None, "Sin seccion camera")
    hfov = camera.find("horizontal_fov")
    image = camera.find("image")
    check(hfov is not None and image is not None, "FOV o configuracion de imagen ausentes")
    width, height = image.find("width"), image.find("height")
    check(width is not None and height is not None, "Resolucion RGB no declarada")
    hfov.text = f"{FOV:.12f}"
    width.text, height.text = "640", "360"
    rate = sensor.find("update_rate")
    if rate is None:
        rate = ET.SubElement(sensor, "update_rate")
    rate.text = "8"
    active = sensor.find("always_on")
    if active is None:
        active = ET.SubElement(sensor, "always_on")
    active.text = "true"
    vis = sensor.find("visualize")
    if vis is None:
        vis = ET.SubElement(sensor, "visualize")
    vis.text = "false"
    data = ET.tostring(root, encoding="utf-8", xml_declaration=False)
    check(not data.lstrip().startswith(b"<?xml"), "Cabecera XML incompatible con spawn_entity")
    re = ET.fromstring(data)
    check(len(list(re.iter("sensor"))) == len(list(root.iter("sensor"))), "Numero de sensores alterado")
    new_link = next(x for x in re.iter("link") if x.get("name") == "camera_rgb_optical_frame")
    new_sensor = next(s for s in new_link.findall("sensor") if s.get("type") == "camera")
    check(abs(float(new_sensor.find("pose").text.split()[4]) - PITCH) < 1e-9,
          "Pitch SDF erroneo")
    check(abs(float(new_sensor.find("camera/horizontal_fov").text) - FOV) < 1e-9,
          "HFOV SDF erroneo")
    return data


def prepare_urdf(source: bytes) -> bytes:
    # Conservar el XML xacro original; ElementTree pierde estructuras
    # necesarias para resolver ${namespace} al serializar.
    import re
    contenido = source.decode("utf-8")
    ET.fromstring(contenido)
    juntas = list(re.finditer(r"<joint\b[^>]*>.*?</joint>", contenido, re.DOTALL))
    candidatas = [m for m in juntas if '${namespace}camera_rgb_joint' in m.group()]
    check(len(candidatas) == 1, "Articulacion RGB no es unica")
    bloque = candidatas[0].group()
    origen = re.search(r"<origin\b[^>]*/>", bloque, re.DOTALL)
    check(origen is not None, "Sin origin de camera_rgb_joint")
    rpy = re.search(r"\brpy\s*=\s*([\"'])([^\"']+)\1", origen.group())
    check(rpy is not None, "Sin atributo rpy en camera_rgb_joint")
    valores = [float(n) for n in rpy.group(2).split()]
    check(len(valores) == 3 and max(abs(x) for x in valores) < .001,
          "URDF instalado ya inclinado")
    previo = origen.group()
    nuevo_origin = (previo[:rpy.start(2)] + f"0 {PITCH:.12f} 0" +
                    previo[rpy.end(2):])
    nuevo_bloque = bloque.replace(previo, nuevo_origin, 1)
    modificado = (contenido[:candidatas[0].start()] + nuevo_bloque +
                  contenido[candidatas[0].end():])
    arbol = ET.fromstring(modificado)
    objetivo = [j for j in arbol.iter("joint")
                if (j.get("name") or "").endswith("camera_rgb_joint")]
    check(len(objetivo) == 1 and abs(float(objetivo[0].find("origin").get("rpy").split()[1]) - PITCH) < 1e-9,
          "Pitch URDF incorrecto")
    check('${namespace}camera_rgb_optical_joint' in modificado,
          "Se perdio la articulacion optica")
    return modificado.encode("utf-8")


def prepare_launch(original: str, diagnostic=False) -> bytes:
    before_sdf = 'robot_pkg / "models/turtlebot3_waffle_pi/model.sdf"'
    before_urdf = 'descripcion_pkg / "urdf/turtlebot3_waffle_pi.urdf"'
    check(original.count(before_sdf) == original.count(before_urdf) == 1,
          "Launch original no coincide con el esquema esperado")
    launch = original.replace(before_sdf,
                              'raiz / "modelos/variantes_opticas/waffle_pi_optica_v6.sdf"')
    launch = launch.replace(before_urdf,
                            'raiz / "modelos/variantes_opticas/waffle_pi_optica_v6.urdf"')
    if diagnostic:
        original_pose = '    x, y = escenario["robot"]["start_xy_m"]\n    yaw = escenario["robot"]["start_yaw_rad"]'
        check(launch.count(original_pose) == 1, "Pose de launch original inesperada")
        x, y = 3.945, 2.903
        yaw = math.atan2(2.8 - y, 4.73 - x)
        new_pose = ("    # Solo prueba visual: NO es una mision A* completada.\n"
                    f"    x, y = {x}, {y}\n    yaw = {yaw:.12f}")
        launch = launch.replace(original_pose, new_pose, 1)
    check("almacen_f1.world" in launch and "spawn_entity.py" in launch,
          "Launch sin mundo o sin spawn_entity")
    compile(launch, "launch_v6", "exec")
    return launch.encode("utf-8")


def check_write(path: Path, data: bytes):
    if path.exists():
        check(path.is_file() and path.read_bytes() == data,
              f"Existe archivo diferente, NO sobrescrito: {path}")
        print("[OK] Existente identico:", path.relative_to(ROOT))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    print("[CREADO]", path.relative_to(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preparar", action="store_true", required=True)
    parser.parse_args()
    check(FIS.is_file() and sha(FIS.read_bytes()) == FIS_SHA, "FIS v03 no coincide")
    from ament_index_python.packages import get_package_share_directory
    gz = Path(get_package_share_directory("turtlebot3_gazebo"))
    desc = Path(get_package_share_directory("turtlebot3_description"))
    original_paths = {
        "sdf": gz / "models/turtlebot3_waffle_pi/model.sdf",
        "urdf": desc / "urdf/turtlebot3_waffle_pi.urdf",
        "launch": ROOT / "launch/almacen_f1.launch.py",
    }
    check(all(p.is_file() for p in original_paths.values()), "Faltan archivos originales")
    sources = {k: p.read_bytes() for k, p in original_paths.items()}
    results = {
        SDF_OUT: prepare_sdf(sources["sdf"]),
        URDF_OUT: prepare_urdf(sources["urdf"]),
        LAUNCH_OUT: prepare_launch(sources["launch"].decode("utf-8")),
        DIAG_OUT: prepare_launch(sources["launch"].decode("utf-8"), diagnostic=True),
    }
    import tempfile
    with tempfile.TemporaryDirectory(prefix="amr_v6_xacro_") as tmp:
        candidate = Path(tmp) / "waffle_pi_optica_v6.urdf"
        candidate.write_bytes(results[URDF_OUT])
        completed = subprocess.run(["xacro", str(candidate)], capture_output=True,
                                   text=True, timeout=20)
        check(completed.returncode == 0, "xacro no acepta URDF: " + completed.stderr[-1200:])
        ET.fromstring(completed.stdout)
    expected_manifest = {
        "variante": "Optica operacional v6 HFOV 145°, pitch +12°, RGB 640x360",
        "origen_optica": "A2_R22_R23_bateria_integral_CNN_v6_camara_contextual.py",
        "hfov_deg": 145.0,
        "cam_pitch_deg": 12.0,
        "resolucion": [640, 360],
        "fuentes_sha256": {str(original_paths[k]): sha(data) for k, data in sources.items()},
        "archivos_variante_sha256": {str(p.relative_to(ROOT)): sha(data) for p, data in results.items()},
        "fis_sha256": FIS_SHA,
        "nota": "Variante experimental, no valida mision ni R25 por si sola. Originales y CNN no modificados.",
    }
    results[MANIFEST] = (json.dumps(expected_manifest, indent=2,
                                   ensure_ascii=False) + "\n").encode("utf-8")
    print("=== PREPARACION DE OPTICA V6: SIN MOVIMIENTO ===")
    for path, data in results.items():
        check_write(path, data)
    check(all(p.read_bytes() == sources[k] for k, p in original_paths.items()),
          "Una fuente original cambio")
    check(sha(FIS.read_bytes()) == FIS_SHA, "FIS modificado inesperadamente")
    print("[OK] SDF (sin cabecera XML) y URDF (xacro) validos")
    print("[OK] HFOV=145°, pitch=+12°, RGB=640x360")
    print("[OK] Launch general y diagnostico creados; originales intactos")
    print("[OK] FIS v03 intacto")
    print("[SEGURIDAD] Sin Gazebo, ROS, movimiento ni entrenamiento")
    print("[SIGUIENTE] NO ejecutar launch general; primero validar la variante diagnostica")


if __name__ == "__main__":
    main()
