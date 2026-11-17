#!/usr/bin/env python3
"""Entrena y evalúa la CNN final a partir del manifiesto de capturas de Gazebo."""
import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import tensorflow as tf


CLASES = ("ALMACENAMIENTO", "DESPACHO", "EMPAQUE", "RECEPCION")
LOTE = 32


def leer_manifiesto(manifiesto):
    particiones = {nombre: [] for nombre in ("train", "val", "test")}
    grupos = defaultdict(set)
    huellas = defaultdict(set)
    with manifiesto.open(encoding="utf-8-sig", newline="") as archivo:
        for fila in csv.DictReader(archivo):
            ruta = (manifiesto.parent / fila["archivo"]).resolve()
            particion = fila["particion"]
            huella = hashlib.sha256(ruta.read_bytes()).hexdigest()
            if huella != fila["sha256"]:
                raise ValueError(f"La imagen no coincide con el manifiesto: {ruta}")
            grupos[fila["grupo"]].add(particion)
            huellas[huella].add(particion)
            particiones[particion].append((str(ruta), CLASES.index(fila["clase"])))
    if any(len(valores) > 1 for valores in (*grupos.values(), *huellas.values())):
        raise ValueError("El manifiesto comparte grupos o imágenes entre particiones")
    if any(set(etiquetas for _, etiquetas in filas) != set(range(4))
           for filas in particiones.values()):
        raise ValueError("Cada partición debe contener las cuatro estaciones")
    return particiones


def dataset(filas, entrenamiento=False):
    rutas, etiquetas = zip(*filas)
    datos = tf.data.Dataset.from_tensor_slices((list(rutas), list(etiquetas)))
    if entrenamiento:
        datos = datos.shuffle(len(filas), seed=451)

    def cargar(ruta, etiqueta):
        rgb = tf.io.decode_png(tf.io.read_file(ruta), channels=3)
        shape = tf.shape(rgb)[:2]
        escala = 224.0 / tf.cast(tf.reduce_max(shape), tf.float32)
        nuevo = tf.cast(tf.round(tf.cast(shape, tf.float32) * escala), tf.int32)
        rgb = tf.image.resize(rgb, nuevo)
        margen = (224 - nuevo) // 2
        rgb = tf.image.pad_to_bounding_box(tf.round(rgb), margen[0], margen[1], 224, 224)
        return rgb, tf.one_hot(etiqueta, len(CLASES))

    return datos.map(cargar, num_parallel_calls=tf.data.AUTOTUNE).batch(LOTE).prefetch(tf.data.AUTOTUNE)


def crear_modelo():
    base = tf.keras.applications.MobileNetV2(input_shape=(224, 224, 3),
                                             include_top=False, weights="imagenet")
    base.trainable = False
    entrada = tf.keras.Input((224, 224, 3))
    aumento = tf.keras.Sequential([
        tf.keras.layers.RandomRotation(15 / 360),
        tf.keras.layers.RandomZoom(.10),
        tf.keras.layers.RandomBrightness(.20, value_range=(0, 255)),
    ])
    x = tf.keras.layers.Rescaling(1 / 127.5, offset=-1)(aumento(entrada))
    x = tf.keras.layers.GlobalAveragePooling2D()(base(x, training=False))
    x = tf.keras.layers.Dense(128, activation="relu")(x)
    x = tf.keras.layers.Dropout(.3)(x)
    salida = tf.keras.layers.Dense(len(CLASES), activation="softmax")(x)
    modelo = tf.keras.Model(entrada, salida)
    modelo.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                   loss="categorical_crossentropy", metrics=["accuracy"])
    return modelo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifiesto", type=Path)
    parser.add_argument("salida", type=Path)
    args = parser.parse_args()
    tf.keras.utils.set_random_seed(451)
    filas = leer_manifiesto(args.manifiesto.resolve())
    args.salida.mkdir(parents=True, exist_ok=False)
    modelo = crear_modelo()
    historial = modelo.fit(
        dataset(filas["train"], entrenamiento=True),
        validation_data=dataset(filas["val"]), epochs=20,
        callbacks=[tf.keras.callbacks.EarlyStopping(
            monitor="val_loss", start_from_epoch=9, patience=4, restore_best_weights=True)],
    ).history
    modelo_path = args.salida / "modelo.keras"
    modelo.save(modelo_path)
    tf.keras.utils.plot_model(modelo, to_file=str(args.salida / "arquitectura.png"),
                              show_shapes=True, show_trainable=True)
    probabilidades = modelo.predict(dataset(filas["test"]), verbose=0)
    reales = [clase for _, clase in filas["test"]]
    predichas = probabilidades.argmax(axis=1).tolist()
    matriz = tf.math.confusion_matrix(reales, predichas, num_classes=4).numpy().tolist()
    metricas = {
        clase: {"precision": matriz[i][i] / max(1, sum(fila[i] for fila in matriz)),
                "recall": matriz[i][i] / sum(matriz[i])}
        for i, clase in enumerate(CLASES)
    }
    errores = [
        {"archivo": str(Path(ruta).relative_to(args.manifiesto.resolve().parent)),
         "real": CLASES[real], "predicho": CLASES[predicha],
         "confianza": float(probs[predicha])}
        for (ruta, real), predicha, probs in zip(filas["test"], predichas, probabilidades)
        if real != predicha
    ]
    reporte = {
        "clases": CLASES, "pesos_iniciales": "imagenet", "lote": LOTE, "epocas_maximas": 20,
        "early_stopping": {"monitor": "val_loss", "patience": 4, "start_from_epoch": 9},
        "manifiesto_sha256": hashlib.sha256(args.manifiesto.read_bytes()).hexdigest(),
        "modelo_sha256": hashlib.sha256(modelo_path.read_bytes()).hexdigest(),
        "exactitud_test": sum(matriz[i][i] for i in range(4)) / len(reales),
        "matriz_confusion": matriz, "metricas_por_estacion": metricas, "errores": errores,
        "historial": historial,
    }
    (args.salida / "evaluacion.json").write_text(json.dumps(reporte, indent=2), encoding="utf-8")
    import matplotlib.pyplot as plt
    figura, ejes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    for eje, metrica in zip(ejes, ("accuracy", "loss")):
        eje.plot(historial[metrica], label="Entrenamiento")
        eje.plot(historial[f"val_{metrica}"], label="Validación")
        eje.set(xlabel="Época", ylabel=metrica)
        eje.legend()
    figura.savefig(args.salida / "curvas.png", dpi=160)


if __name__ == "__main__":
    main()
