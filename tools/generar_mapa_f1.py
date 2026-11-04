#!/usr/bin/env python3
from pathlib import Path
from collections import deque
from math import ceil
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/escenario_f1.yaml"
DESTINO = ROOT / "mapas"

c = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

w, h = map(float, c["world"]["size_m"])
t = float(c["world"]["wall_thickness_m"])
res = float(c["map"]["resolution_m"])
radio = float(c["map"]["inflation_radius_m"])

W = round(w / res)
H = round(h / res)

assert abs(W * res - w) < 1e-8
assert abs(H * res - h) < 1e-8
assert res > 0 and radio >= 0

x0, y0 = -w / 2, -h / 2

# Rectangulos ocupados del mundo
rectangulos = []

for obj in c["racks"] + c.get("obstacles", []):
    x, y = map(float, obj["center_xy_m"])
    sx, sy = map(float, obj["size_xy_m"])

    rectangulos.append((
        x - sx / 2, x + sx / 2,
        y - sy / 2, y + sy / 2
    ))

# 254 = libre; 0 = ocupado
base = bytearray([254]) * (W * H)

for fila in range(H):
    y = y0 + (H - fila - 0.5) * res

    for columna in range(W):
        x = x0 + (columna + 0.5) * res

        pared = (
            x <= x0 + t or
            x >= x0 + w - t or
            y <= y0 + t or
            y >= y0 + h - t
        )

        obstaculo = any(
            xa <= x <= xb and ya <= y <= yb
            for xa, xb, ya, yb in rectangulos
        )

        if pared or obstaculo:
            base[fila * W + columna] = 0

# Inflado circular para el planificador A*
inflado = bytearray(base)
alcance = ceil(radio / res)

offsets = [
    (dy, dx)
    for dy in range(-alcance, alcance + 1)
    for dx in range(-alcance, alcance + 1)
    if (dx * res)**2 + (dy * res)**2
       <= radio**2 + 1e-10
]

for indice, valor in enumerate(base):
    if valor != 0:
        continue

    fila, columna = divmod(indice, W)

    for dy, dx in offsets:
        fy = fila + dy
        fx = columna + dx

        if 0 <= fy < H and 0 <= fx < W:
            inflado[fy * W + fx] = 0

def indice_de(xy):
    x, y = map(float, xy)

    col = int((x - x0) / res)
    fila = H - 1 - int((y - y0) / res)

    if not (0 <= fila < H and 0 <= col < W):
        raise ValueError(f"Coordenada fuera del mapa: {xy}")

    return fila * W + col

# Verificar que el origen del robot es navegable
inicio = indice_de(c["robot"]["start_xy_m"])
assert inflado[inicio] == 254, "Inicio bloqueado"

# Analisis de conectividad sobre celdas libres
cola = deque([inicio])
visitados = {inicio}

while cola:
    actual = cola.popleft()
    fila, col = divmod(actual, W)

    for dy, dx in ((1, 0), (-1, 0),
                   (0, 1), (0, -1)):
        fy, fx = fila + dy, col + dx

        if not (0 <= fy < H and 0 <= fx < W):
            continue

        vecino = fy * W + fx

        if inflado[vecino] != 254:
            continue

        if vecino not in visitados:
            visitados.add(vecino)
            cola.append(vecino)

for estacion in c["stations"]:
    indice = indice_de(estacion["goal_xy_m"])

    assert indice in visitados, (
        f"Estacion inaccesible: {estacion['id']}"
    )

    print(f"[OK] Estacion accesible: {estacion['id']}")

# Guardar imagenes PGM y metadatos YAML
DESTINO.mkdir(exist_ok=True)

def guardar(nombre, datos):
    pgm = DESTINO / f"{nombre}.pgm"
    meta = DESTINO / f"{nombre}.yaml"

    encabezado = f"P5\n{W} {H}\n255\n".encode("ascii")
    pgm.write_bytes(encabezado + bytes(datos))

    meta.write_text(
        f"image: {nombre}.pgm\n"
        "mode: trinary\n"
        f"resolution: {res}\n"
        f"origin: [{x0}, {y0}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8"
    )

guardar("almacen_f1", base)
guardar("almacen_f1_inflado", inflado)

print(f"Mapa: {W} x {H} celdas")
print(f"Resolucion: {res} m")
print(f"Radio de inflado: {radio} m")
print(f"Celdas alcanzables: {len(visitados)}")
print("F1.1.4: mapas generados correctamente")
