#!/usr/bin/env python3
"""R35: comprobacion estatica de coordenadas, mapas y rutas."""

import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

RAIZ = Path(__file__).resolve().parents[1]

esc = yaml.safe_load(
    (RAIZ / "config/escenario_f1.yaml").read_text()
)
interfaces = yaml.safe_load(
    (RAIZ / "config/interfaces.yaml").read_text()
)
meta = yaml.safe_load(
    (RAIZ / "mapas/almacen_f1_inflado.yaml").read_text()
)

res = float(meta["resolution"])
x0, y0, yaw0 = map(float, meta["origin"])
ancho, alto = map(float, esc["world"]["size_m"])

W = round(ancho / res)
H = round(alto / res)

assert W == 200 and H == 160
assert math.isclose(x0, -ancho / 2)
assert math.isclose(y0, -alto / 2)
assert math.isclose(yaw0, 0.0)

print("[OK] Mapa 200x160, resolucion 0.05 m")
print("[OK] Origen de rejilla (-5,-4), yaw 0")

pgm = (RAIZ / "mapas/almacen_f1_inflado.pgm").read_bytes()
cabecera, dimensiones, maximo, pixeles = pgm.split(b"\n", 3)

assert cabecera == b"P5"
assert dimensiones == b"200 160"
assert maximo == b"255"
assert len(pixeles) == W * H

def celda(x, y):
    col = math.floor((x - x0) / res)
    fila = H - 1 - math.floor((y - y0) / res)
    assert 0 <= fila < H and 0 <= col < W
    return fila, col

def centro(fila, col):
    return (
        x0 + (col + 0.5) * res,
        y0 + (H - fila - 0.5) * res
    )

def libre(fila, col):
    return pixeles[fila * W + col] == 254

# Comprobar correspondencia geometrica mundo-SDF.
mundo = ET.parse(RAIZ / "modelos/almacen_f1.world")
modelos = {
    m.attrib["name"]: m
    for m in mundo.findall("./world/model")
}

for obj in esc["stations"]:
    nombre = f"estacion_{obj['id'].lower()}"
    pose = list(map(
        float,
        modelos[nombre].findtext("pose").split()
    ))
    x, y = obj["goal_xy_m"]
    assert math.isclose(pose[0], x, abs_tol=1e-9)
    assert math.isclose(pose[1], y, abs_tol=1e-9)
    assert libre(*celda(x, y))

print("[OK] Cuatro estaciones: SDF, YAML y mapa coherentes")

for obj in esc["obstacles"]:
    pose = list(map(
        float,
        modelos[obj["id"]].findtext("pose").split()
    ))
    x, y = obj["center_xy_m"]
    assert math.isclose(pose[0], x, abs_tol=1e-9)
    assert math.isclose(pose[1], y, abs_tol=1e-9)

print("[OK] Dos obstaculos: SDF y YAML coherentes")

# Comprobar contrato CSV y signos.
assert interfaces["proposed_csv"]["header"] == ["x_m", "y_m"]
assert interfaces["proposed_csv"]["reference_frame"] == "map"
assert interfaces["robot"]["angular_positive_direction"] == "left_ccw"

print("[OK] CSV x_m,y_m en marco map")
print("[OK] Giro positivo antihorario (REP-103)")

origen_robot = esc["robot"]["start_xy_m"]
metas = [s["goal_xy_m"] for s in esc["stations"]]

carpeta = (
    RAIZ / "resultados/R04_astar_mapa_corregido/resultados"
)

rutas = [
    carpeta / f"F12_ruta_{i}.csv"
    for i in range(1, 5)
] + [
    carpeta / f"F12_orden_{c}.csv"
    for c in "ABC"
]

assert all(p.is_file() for p in rutas)

for indice, archivo in enumerate(rutas):
    with archivo.open(newline="", encoding="utf-8-sig") as f:
        lector = csv.DictReader(f)
        assert lector.fieldnames == ["x_m", "y_m"]
        puntos = [
            (float(r["x_m"]), float(r["y_m"]))
            for r in lector
        ]

    assert len(puntos) >= 2

    celdas = []

    for x, y in puntos:
        assert math.isfinite(x) and math.isfinite(y)
        fila, col = celda(x, y)
        assert libre(fila, col)

        # Las rutas se representan por centros de celdas.
        cx, cy = centro(fila, col)
        assert math.hypot(cx - x, cy - y) < 1e-6
        celdas.append((fila, col))

    for (f1, c1), (f2, c2) in zip(
        celdas[:-1], celdas[1:]
    ):
        df, dc = abs(f2 - f1), abs(c2 - c1)
        assert df <= 1 and dc <= 1

        # Una diagonal no debe cortar esquinas ocupadas.
        if df == 1 and dc == 1:
            assert libre(f1, c2) and libre(f2, c1)

    if indice < 4:
        esperado_inicio = (
            origen_robot if indice == 0 else metas[indice - 1]
        )
        esperado_fin = metas[indice]

        assert math.dist(
            puntos[0], esperado_inicio
        ) <= res

        assert math.dist(
            puntos[-1], esperado_fin
        ) <= res
    else:
        assert math.dist(
            puntos[0], origen_robot
        ) <= res

    print(
        f"[OK] {archivo.name}: {len(puntos)} puntos; "
        "sin celdas ocupadas ni cortes diagonales"
    )

print("[R35] COHERENCIA ESTATICA SATISFACTORIA")
print("[ALCANCE] Ensayo estatico: no comprueba TF en ROS 2")
print("[REFERENCIA] Validacion TF independiente en docs/F1_R35_marcos_tf.md")
