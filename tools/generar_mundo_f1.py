#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/escenario_f1.yaml"
OUTPUT = ROOT / "modelos/almacen_f1.world"

cfg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
info = cfg["world"]
w, h = map(float, info["size_m"])
t = float(info["wall_thickness_m"])

assert w >= 4 and h >= 4
assert 0 < t < 0.5
assert len(cfg["stations"]) == 4

sdf = ET.Element("sdf", version="1.6")
world = ET.SubElement(sdf, "world", name=info["name"])

ET.SubElement(world, "gravity").text = "0 0 -9.81"

physics = ET.SubElement(
    world, "physics", name="default_physics", type="ode"
)
ET.SubElement(physics, "max_step_size").text = "0.001"
ET.SubElement(physics, "real_time_update_rate").text = "1000"

for uri in ["model://ground_plane", "model://sun"]:
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = uri

def crear_modelo(nombre, xyz, forma, dimensiones,
                 color, colision=True):
    modelo = ET.SubElement(world, "model", name=nombre)
    ET.SubElement(modelo, "static").text = "true"
    ET.SubElement(modelo, "pose").text = (
        f"{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0"
    )

    link = ET.SubElement(modelo, "link", name="body")

    elementos = ["visual"]
    if colision:
        elementos.append("collision")

    for tipo in elementos:
        nodo = ET.SubElement(link, tipo, name=tipo)
        geometria = ET.SubElement(nodo, "geometry")
        figura = ET.SubElement(geometria, forma)

        if forma == "box":
            ET.SubElement(figura, "size").text = (
                " ".join(map(str, dimensiones))
            )
        else:
            ET.SubElement(figura, "radius").text = str(
                dimensiones[0]
            )
            ET.SubElement(figura, "length").text = str(
                dimensiones[1]
            )

        if tipo == "visual":
            material = ET.SubElement(nodo, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color

# Paredes perimetrales
altura = 1.6
gris = "0.40 0.43 0.48 1"

for lado, x in [
    ("oeste", -w / 2 + t / 2),
    ("este", w / 2 - t / 2)
]:
    crear_modelo(
        f"pared_{lado}",
        (x, 0, altura / 2),
        "box", (t, h, altura), gris
    )

for lado, y in [
    ("sur", -h / 2 + t / 2),
    ("norte", h / 2 - t / 2)
]:
    crear_modelo(
        f"pared_{lado}",
        (0, y, altura / 2),
        "box", (w, t, altura), gris
    )

# Estanterias
for rack in cfg["racks"]:
    x, y = rack["center_xy_m"]
    sx, sy = rack["size_xy_m"]
    z = rack["height_m"]

    crear_modelo(
        rack["id"], (x, y, z / 2),
        "box", (sx, sy, z),
        "0.55 0.35 0.17 1"
    )

# Obstaculos
for objeto in cfg.get("obstacles", []):
    x, y = objeto["center_xy_m"]
    sx, sy = objeto["size_xy_m"]
    z = objeto["height_m"]

    crear_modelo(
        objeto["id"], (x, y, z / 2),
        "box", (sx, sy, z),
        "0.85 0.44 0.12 1"
    )

# Estaciones: marcadores visuales sin colision
colores = {
    "blue": "0.05 0.25 1 1",
    "red": "1 0.08 0.08 1",
    "green": "0.05 0.85 0.15 1",
    "yellow": "1 0.85 0.05 1"
}

for estacion in cfg["stations"]:
    x, y = estacion["goal_xy_m"]
    color = colores[estacion["marker_color"]]

    crear_modelo(
        "estacion_" + estacion["id"].lower(),
        (x, y, 0.008),
        "cylinder", (0.30, 0.01),
        color, colision=False
    )


# B2 - Paneles verticales de estaciones
# Son elementos visuales sin colision para conservar el mapa.
for numero, estacion in enumerate(cfg["stations"], start=1):
    x, y = estacion["goal_xy_m"]
    lado = -1 if x < 0 else 1
    identificador = estacion["id"].lower()

    # Colocar el panel junto al muro lateral.
    x_panel = lado * (w / 2 - t - 0.12)
    color = colores[estacion["marker_color"]]

    crear_modelo(
        "panel_" + identificador,
        (x_panel, y, 0.60),
        "box",
        (0.04, 0.90, 0.72),
        color,
        colision=False
    )

    # Una, dos, tres o cuatro marcas blancas.
    # Las marcas apuntan hacia el interior del almacen.
    x_frente = x_panel - lado * 0.037

    for indice in range(numero):
        desplazamiento = (indice - (numero - 1) / 2) * 0.18

        crear_modelo(
            f"panel_{identificador}_marca_{indice + 1}",
            (x_frente, y + desplazamiento, 0.62),
            "box",
            (0.02, 0.09, 0.40),
            "1 1 1 1",
            colision=False
        )

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
ET.indent(sdf, space="  ")
ET.ElementTree(sdf).write(
    OUTPUT, encoding="utf-8",
    xml_declaration=True
)

print("F1.1.3: mundo SDF generado")
print(f"Archivo: {OUTPUT}")
print(f"Modelos propios: {len(world.findall('model'))}")
print("Estaciones: 4")
print("Obstaculos de prueba: 2")
