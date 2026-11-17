# Entrega final U3S7

## Modelo, datos y evaluación

El reconocedor carga `modelos/modelo_u3s7.keras`. Sus métricas, figuras y
análisis de fallos se guardan en `resultados/cnn_u3s7_20261116/`.
La evaluación de 90 imágenes reservadas utiliza el mismo modelo que el ensayo
del robot. El resumen registra su SHA-256.

Las 600 imágenes originales se publican con un manifiesto que registra pose,
estación, partición y hash de cada archivo. El código de entrenamiento verifica
los hashes y que los grupos de pose no se compartan entre particiones.
La evaluación nominal y las perturbaciones visuales se reportan por separado.

```bash
~/.venvs/amr-cnn/bin/python tools/entrenar_cnn_final.py \
  datos/u3s7/manifiesto.csv resultados/reentrenamiento_u3s7
~/.venvs/amr-cnn/bin/python tools/evaluar_robustez_cnn.py \
  datos/u3s7/manifiesto.csv resultados/reentrenamiento_u3s7
```

Para capturar una serie nueva desde Gazebo, ejecutar el capturador de
`tools/a2_bancos/A2_R22_R23_bateria_integral_CNN_v6_camara_contextual.py` con
`--solo-captura` usando el entorno ROS descrito en la guía.

## Reproducir en Gazebo

Instalar ROS 2 Humble, TurtleBot3 y los entornos Python de la
[guía de configuración](PRUEBAS_FINALES_WSL.md). Desde la raíz, en Ubuntu:

```bash
bash tools/verificar_gazebo_local.sh
bash tools/ejecutar_prueba_u3s7.sh --desvio
AMR_SEMILLA=451 bash tools/ejecutar_prueba_u3s7.sh
```

El ensayo genera un Waffle Pi con sensores de contacto en un mundo aislado.
Se posiciona primero en Recepción y cumple diez órdenes seguidas. Las cajas
se seleccionan al azar entre posiciones libres al costado de cada ruta.
Se conserva el tamaño y la posición de la caja en cada registro. La precisión
visual se comprueba al llegar y los contactos físicos se miden en Gazebo.

En el ensayo de desvío se induce una llegada a Recepción con destino
Almacenamiento. Al confirmar Recepción con visión y odometría se genera otro
CSV con los puntos de la nueva ruta. La prueba es independiente de las diez
órdenes con cajas.

La duración de navegación se evalúa en tiempo de simulación. Se registra
también la duración hasta confirmar la estación, que incluye la percepción y
si corresponde la orientación frente al panel. El límite temporal por ruta
es `1,30 × L / 0,20` segundos. Los ensayos representan el almacén simulado y
las posiciones de cajas descritas, con pasillo transitable.

## Entregables

- Informe IEEE de ocho páginas en `docs/entrega_u3s7/`, editable en Word y en PDF.
- Video de demostración de 1 min 20 s con fotogramas de la cámara del robot.
- [Guía de defensa oral](DEFENSA_U3S7.md).
- [Resumen de las diez órdenes](../resultados/U3S7/secuencia/resumen.json) y [tabla CSV](../resultados/U3S7/tabla_ordenes.csv).
- [Resumen del desvío inducido](../resultados/U3S7/desvio/resumen.json).
- Código, modelo y 600 imágenes con manifiesto en `datos/u3s7/`.

La prueba nominal de CNN dio **89/90** aciertos. La secuencia con semilla 451
confirmó **10/10** destinos, con **cero contactos externos** y **10/10** tiempos
simulados de navegación dentro de la cota. La prueba controlada de desvío
recuperó la orden original en un solo intento. Los tres registros contienen
el mismo SHA-256 del modelo.
