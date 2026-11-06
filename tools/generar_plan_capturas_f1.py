#!/usr/bin/env python3
"""Tarea 1.7: manifiesto de capturas planificadas, no dataset real."""

import csv
import itertools
import random
from collections import Counter
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
DESTINO = RAIZ / "datos/cnn/plan_adquisicion.csv"

ESTACIONES = [
    ("RECEPCION", "recepcion", (18, 4, 3)),
    ("ALMACENAMIENTO", "almacenamiento", (18, 3, 4)),
    ("EMPAQUE", "empaque", (17, 4, 4)),
    ("DESPACHO", "despacho", (17, 4, 4)),
]

DISTANCIAS = [0.30, 0.60, 0.90, 1.20, 1.50]
ANGULOS = [-30, -15, 0, 15, 30]
PARTICIONES = ["train", "val", "test"]

if DESTINO.exists():
    raise SystemExit(
        "[ALTO] El manifiesto ya existe; no se sobrescribira."
    )

filas = []

for indice, (estacion, nombre, cuotas) in enumerate(ESTACIONES):
    posiciones = list(itertools.product(DISTANCIAS, ANGULOS))
    random.Random(20261105 + indice).shuffle(posiciones)

    assert sum(cuotas) == 25

    asignaciones = []
    for particion, cantidad in zip(PARTICIONES, cuotas):
        asignaciones.extend([particion] * cantidad)

    for (distancia, angulo), particion in zip(
        posiciones, asignaciones
    ):
        for toma in range(1, 7):
            codigo_angulo = (
                f"m{abs(angulo):02d}" if angulo < 0
                else f"p{angulo:02d}"
            )
            archivo = (
                f"{nombre}_d{round(distancia*100):03d}"
                f"_a{codigo_angulo}_t{toma:02d}.png"
            )
            ruta = (
                f"datos/cnn/imagenes/{particion}/"
                f"{nombre}/{archivo}"
            )

            filas.append({
                "id": f"IMG_{len(filas)+1:04d}",
                "estacion": estacion,
                "distancia_m": f"{distancia:.2f}",
                "angulo_deg": angulo,
                "toma": toma,
                "particion": particion,
                "ruta_objetivo": ruta,
                "estado": "PLANIFICADA",
            })

assert len(filas) == 600

conteo = Counter(f["particion"] for f in filas)
assert conteo == {"train": 420, "val": 90, "test": 90}

# Comprobar que ninguna posicion de cámara se reparte
# entre entrenamiento, validacion y prueba.
grupos = {}
for fila in filas:
    clave = (
        fila["estacion"],
        fila["distancia_m"],
        fila["angulo_deg"],
    )
    grupos.setdefault(clave, set()).add(fila["particion"])

assert len(grupos) == 100
assert all(len(particiones) == 1 for particiones in grupos.values())

DESTINO.parent.mkdir(parents=True, exist_ok=True)

with DESTINO.open("w", newline="", encoding="utf-8") as f:
    escritor = csv.DictWriter(f, fieldnames=list(filas[0]))
    escritor.writeheader()
    escritor.writerows(filas)

print("[OK] Planificacion: 600 registros, 150 por estacion")
print("[OK] Train: 420 | Val: 90 | Test: 90")
print("[OK] Sin posiciones de camara compartidas entre conjuntos")
print("[INFO] 0 imagenes capturadas; todas estan PLANIFICADAS")
print(f"[OK] Manifiesto: {DESTINO}")
