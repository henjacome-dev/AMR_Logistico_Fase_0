#!/usr/bin/env python3
import math
from pathlib import Path
import yaml

RAIZ = Path(__file__).resolve().parent.parent

def leer_pgm(ruta):
    with ruta.open("rb") as f:
        def token():
            while True:
                b = f.read(1)
                if not b:
                    raise ValueError("Cabecera PGM incompleta")
                if b == b"#":
                    f.readline()
                    continue
                if not b.isspace():
                    break
            valor = bytearray(b)
            while True:
                b = f.read(1)
                if not b or b.isspace():
                    break
                valor.extend(b)
            return bytes(valor)

        if token() != b"P5":
            raise ValueError("Se requiere PGM binario P5")

        ancho = int(token())
        alto = int(token())
        maximo = int(token())

        if maximo != 255:
            raise ValueError("Se requiere PGM de 8 bits")

        datos = f.read(ancho * alto)

        if len(datos) != ancho * alto:
            raise ValueError("Datos PGM incompletos")

    return ancho, alto, datos

meta = yaml.safe_load(
    (RAIZ / "mapas/almacen_f1_inflado.yaml")
    .read_text(encoding="utf-8")
)

escenario = yaml.safe_load(
    (RAIZ / "config/escenario_f1.yaml")
    .read_text(encoding="utf-8")
)

imagen = RAIZ / "mapas" / meta["image"]
ancho, alto, pixeles = leer_pgm(imagen)

res = float(meta["resolution"])
ox, oy, theta = meta["origin"]

if abs(theta) > 1e-8:
    raise ValueError("Mapa rotado: requiere transformación adicional")

def comprobar(nombre, x, y):
    columna = math.floor((x - ox) / res)
    fila_mapa = math.floor((y - oy) / res)
    fila_imagen = alto - 1 - fila_mapa

    if not (0 <= columna < ancho and 0 <= fila_imagen < alto):
        print(f"[ERROR] {nombre}: fuera del mapa")
        return False

    pixel = pixeles[fila_imagen * ancho + columna]
    ocupacion = pixel / 255 if meta.get("negate", 0) else 1 - pixel / 255

    if ocupacion < meta["free_thresh"]:
        estado = "LIBRE"
    elif ocupacion > meta["occupied_thresh"]:
        estado = "OCUPADO"
    else:
        estado = "DESCONOCIDO"

    print(
        f"{nombre}: XY=({x:.2f},{y:.2f}) "
        f"Celda=({columna},{fila_mapa}) "
        f"Pixel={pixel} Estado={estado}"
    )
    return estado == "LIBRE"

print("=== CONFIGURACION ===")
print(f"Mapa: {ancho} x {alto} celdas")
print(f"Resolucion: {res} m/celda")
print(f"Origen: {meta['origin']}")

print("\n=== POSICIONES ===")
x, y = escenario["robot"]["start_xy_m"]
resultados = [comprobar("ROBOT", x, y)]

for estacion in escenario["stations"]:
    x, y = estacion["goal_xy_m"]
    resultados.append(
        comprobar(estacion["id"], x, y)
    )

print("\n=== RESULTADO ===")
if all(resultados) and len(resultados) == 5:
    print("F1.2.1 APROBADO: 5 posiciones libres")
else:
    raise SystemExit(
        "F1.2.1 PENDIENTE: revisar posiciones o mapa"
    )
