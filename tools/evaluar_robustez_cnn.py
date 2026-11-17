"""Evalúa situaciones visuales adversas por separado del test principal."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageEnhance, ImageFilter
import tensorflow as tf


def op_oclusion(im):
    copia = im.copy()
    ImageDraw.Draw(copia).rectangle((120, 65, 520, 295), fill=(87, 83, 78))
    return copia


def entrada(im):
    escala = 224 / max(im.size)
    im = im.resize(tuple(round(v * escala) for v in im.size), Image.Resampling.BILINEAR)
    lienzo = Image.new("RGB", (224, 224))
    lienzo.paste(im, ((224-im.width)//2, (224-im.height)//2))
    return np.asarray(lienzo, dtype=np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifiesto", type=Path)
    parser.add_argument("resultados", type=Path)
    args = parser.parse_args()
    evaluacion = json.loads((args.resultados / "evaluacion.json").read_text(encoding="utf-8"))
    clases = evaluacion["clases"]
    modelo = tf.keras.models.load_model(args.resultados / "modelo.keras", compile=False)
    with args.manifiesto.open(encoding="utf-8-sig", newline="") as archivo:
        filas = [fila for fila in csv.DictReader(archivo) if fila["particion"] == "test"]
    imagenes = [Image.open(args.manifiesto.parent / fila["archivo"]).convert("RGB") for fila in filas]
    etiquetas = np.asarray([clases.index(fila["clase"]) for fila in filas])
    pred_base = modelo.predict(np.stack(list(map(entrada, imagenes))), batch_size=32, verbose=0).argmax(axis=1)
    condiciones = (
        ("oclusion", op_oclusion, "Una caja opaca cubre la región central donde puede quedar el panel."),
        ("oscuridad", lambda im: ImageEnhance.Brightness(im).enhance(.12),
         "El brillo queda al 12 % del original y se reduce el contraste del marcador."),
        ("desenfoque", lambda im: im.filter(ImageFilter.GaussianBlur(20)),
         "El desenfoque elimina bordes del marcador y mezcla su color con el fondo."),
    )
    resultados = []
    elegidos = set()
    for nombre, cambiar, hipotesis in condiciones:
        alteradas = [cambiar(im) for im in imagenes]
        probs = modelo.predict(np.stack(list(map(entrada, alteradas))), batch_size=32, verbose=0)
        pred = probs.argmax(axis=1)
        candidatos = [i for i in np.flatnonzero((pred_base == etiquetas) & (pred != etiquetas))
                      if filas[i]["archivo"] not in elegidos]
        seleccion = None
        if candidatos:
            i = max(candidatos, key=lambda j: probs[j, pred[j]])
            ruta = filas[i]["archivo"]
            elegidos.add(ruta)
            original = f"fallo_{nombre}_original.jpg"
            alterada = f"fallo_{nombre}.jpg"
            imagenes[i].save(args.resultados / original, quality=87)
            alteradas[i].save(args.resultados / alterada, quality=87)
            seleccion = {"ruta": ruta, "real": clases[etiquetas[i]], "predicha": clases[pred[i]],
                         "confianza": float(probs[i, pred[i]]), "hipotesis": hipotesis,
                         "original": original, "perturbada": alterada}
        resultados.append({"condicion": nombre, "aciertos": int(np.sum(pred == etiquetas)),
                           "n": len(filas), "caso": seleccion})
    (args.resultados / "robustez.json").write_text(json.dumps(resultados, ensure_ascii=False, indent=2),
                                                   encoding="utf-8")
    matriz = evaluacion["matriz_confusion"]
    fig, ax = plt.subplots(figsize=(5.2, 4.5), layout="constrained")
    ax.imshow(matriz, cmap="Blues")
    ax.set_xticks(range(4), clases, rotation=29, ha="right", fontsize=8)
    ax.set_yticks(range(4), clases, fontsize=8)
    ax.set(xlabel="Predicción", ylabel="Etiqueta real")
    for i, fila in enumerate(matriz):
        for j, valor in enumerate(fila):
            ax.text(j, i, valor, ha="center", va="center", color="white" if valor > 12 else "black")
    fig.savefig(args.resultados / "matriz.png", dpi=175)


if __name__ == "__main__":
    main()
