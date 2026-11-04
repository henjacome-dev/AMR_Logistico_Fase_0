#!/usr/bin/env python3
"""Verificacion independiente de conectividad A* - F1.2."""
import csv
import heapq
import math
import time
from itertools import count
from pathlib import Path

import yaml

RAIZ = Path(__file__).resolve().parent.parent
SALIDA = RAIZ / "resultados"
SALIDA.mkdir(exist_ok=True)

def leer_pgm(ruta):
    with ruta.open("rb") as f:
        def token():
            b = f.read(1)
            while b and (b.isspace() or b == b"#"):
                if b == b"#":
                    f.readline()
                b = f.read(1)
            if not b:
                raise ValueError("Cabecera PGM incompleta")
            valor = bytearray()
            while b and not b.isspace():
                valor.extend(b)
                b = f.read(1)
            return valor.decode("ascii")

        if token() != "P5":
            raise ValueError("Se requiere PGM binario P5")
        w, h = int(token()), int(token())
        if int(token()) != 255:
            raise ValueError("PGM debe ser de 8 bits")
        pixeles = f.read(w * h)
        if len(pixeles) != w * h:
            raise ValueError("PGM incompleto")
    return w, h, pixeles

meta = yaml.safe_load(
    (RAIZ / "mapas/almacen_f1_inflado.yaml").read_text()
)
esc = yaml.safe_load(
    (RAIZ / "config/escenario_f1.yaml").read_text()
)

W, H, pixeles = leer_pgm(RAIZ / "mapas" / meta["image"])
RES = float(meta["resolution"])
OX, OY, ANG = map(float, meta["origin"])

if abs(ANG) > 1e-8:
    raise ValueError("Mapa rotado no soportado en esta prueba")

def celda(x, y):
    c = math.floor((x - OX) / RES)
    r = H - 1 - math.floor((y - OY) / RES)
    return (c, r)

def libre(c, r):
    if not (0 <= c < W and 0 <= r < H):
        return False
    pixel = pixeles[r * W + c]
    ocupacion = (
        pixel / 255.0 if meta.get("negate", 0)
        else 1.0 - pixel / 255.0
    )
    return ocupacion < meta["free_thresh"]

def xy(c, r):
    return (
        OX + (c + 0.5) * RES,
        OY + (H - r - 0.5) * RES
    )

def heuristica(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return RES * (
        max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    )

MOVIMIENTOS = [
    (dx, dy)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    if dx != 0 or dy != 0
]

def astar(inicio, destino):
    if not libre(*inicio) or not libre(*destino):
        raise RuntimeError("Origen o destino no transitable")

    contador = count()
    abiertos = [(heuristica(inicio, destino),
                 next(contador), 0.0, inicio)]
    g = {inicio: 0.0}
    padre = {}

    while abiertos:
        _, _, costo, actual = heapq.heappop(abiertos)

        if costo > g.get(actual, math.inf) + 1e-9:
            continue

        if actual == destino:
            ruta = [actual]
            while ruta[-1] != inicio:
                ruta.append(padre[ruta[-1]])
            return list(reversed(ruta)), costo

        for dx, dy in MOVIMIENTOS:
            c, r = actual
            vecino = (c + dx, r + dy)

            if not libre(*vecino):
                continue

            # Prohibir cortes diagonales entre obstaculos
            if dx != 0 and dy != 0:
                if not (libre(c + dx, r) and libre(c, r + dy)):
                    continue

            nuevo = costo + RES * math.hypot(dx, dy)

            if nuevo + 1e-9 < g.get(vecino, math.inf):
                g[vecino] = nuevo
                padre[vecino] = actual
                prioridad = nuevo + heuristica(vecino, destino)
                heapq.heappush(
                    abiertos,
                    (prioridad, next(contador), nuevo, vecino)
                )

    raise RuntimeError("No existe ruta transitable")

estaciones = {
    e["id"].upper(): tuple(e["goal_xy_m"])
    for e in esc["stations"]
}

orden = [
    "RECEPCION",
    "ALMACENAMIENTO",
    "EMPAQUE",
    "DESPACHO"
]

puntos = [("ROBOT", tuple(esc["robot"]["start_xy_m"]))]
puntos += [(nombre, estaciones[nombre]) for nombre in orden]

resumen = []
print("=== F1.2.2 - CONECTIVIDAD A* ===")
print(f"Mapa: {W} x {H} | Resolucion: {RES} m")

for i in range(len(puntos) - 1):
    nombre_a, pos_a = puntos[i]
    nombre_b, pos_b = puntos[i + 1]

    inicio = celda(*pos_a)
    destino = celda(*pos_b)

    t0 = time.perf_counter()
    ruta, longitud = astar(inicio, destino)
    tiempo_ms = (time.perf_counter() - t0) * 1000

    archivo = SALIDA / f"F12_ruta_{i+1}.csv"
    with archivo.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["x_m", "y_m"])
        for c, r in ruta:
            x, y = xy(c, r)
            writer.writerow([f"{x:.3f}", f"{y:.3f}"])

    resumen.append([
        nombre_a, nombre_b, len(ruta),
        f"{longitud:.3f}", f"{tiempo_ms:.3f}"
    ])

    print(
        f"[OK] {nombre_a} -> {nombre_b} | "
        f"{len(ruta)} celdas | {longitud:.2f} m | "
        f"{tiempo_ms:.2f} ms"
    )

archivo_resumen = SALIDA / "F12_resumen_rutas.csv"
with archivo_resumen.open(
    "w", newline="", encoding="utf-8"
) as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow([
        "origen", "destino", "celdas",
        "longitud_m", "tiempo_calculo_ms"
    ])
    writer.writerows(resumen)

print("\n=== RESULTADO FINAL ===")
print("F1.2.2 APROBADO: 4 rutas conectadas")
print("CSV generados en resultados/")

# === F1.2.3: TRES ORDENES DE NAVEGACION ===

ordenes = {
    "A": [
        "RECEPCION",
        "ALMACENAMIENTO",
        "EMPAQUE",
        "DESPACHO"
    ],
    "B": [
        "RECEPCION",
        "EMPAQUE",
        "ALMACENAMIENTO",
        "DESPACHO"
    ],
    "C": [
        "DESPACHO",
        "EMPAQUE",
        "ALMACENAMIENTO",
        "RECEPCION"
    ]
}

comparacion = []

print("\n=== F1.2.3 - COMPARACION DE ORDENES ===")

for nombre, secuencia in ordenes.items():
    assert len(secuencia) == 4
    assert len(set(secuencia)) == 4

    actual = celda(*esc["robot"]["start_xy_m"])
    ruta_completa = []
    distancia_total = 0.0

    inicio_tiempo = time.perf_counter()

    for estacion in secuencia:
        destino = celda(*estaciones[estacion])
        ruta, longitud = astar(actual, destino)

        # Evitar duplicar celdas de conexion
        if ruta_completa:
            ruta_completa.extend(ruta[1:])
        else:
            ruta_completa.extend(ruta)

        distancia_total += longitud
        actual = destino

    tiempo_ms = (
        time.perf_counter() - inicio_tiempo
    ) * 1000

    archivo = SALIDA / f"F12_orden_{nombre}.csv"

    with archivo.open(
        "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["x_m", "y_m"])

        for c, r in ruta_completa:
            x, y = xy(c, r)
            writer.writerow([f"{x:.3f}", f"{y:.3f}"])

    comparacion.append([
        nombre,
        "-".join(secuencia),
        len(ruta_completa),
        round(distancia_total, 3),
        round(tiempo_ms, 3)
    ])

    print(
        f"[OK] Orden {nombre}: "
        f"{distancia_total:.2f} m | "
        f"{len(ruta_completa)} celdas | "
        f"{tiempo_ms:.2f} ms"
    )

resumen = SALIDA / "F12_comparacion_ordenes.csv"

with resumen.open(
    "w", newline="", encoding="utf-8"
) as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow([
        "orden", "secuencia", "celdas",
        "longitud_m", "tiempo_ms"
    ])
    writer.writerows(comparacion)

mejor = min(comparacion, key=lambda fila: fila[3])

print("\n=== MEJOR ORDEN EVALUADO ===")
print(f"Orden: {mejor[0]}")
print(f"Distancia: {mejor[3]:.3f} m")
print(f"Celdas: {mejor[2]}")

print("\nF1.2.3: COMPARACION COMPLETADA")
