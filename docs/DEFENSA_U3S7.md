# Defensa oral U3S7

## Reparto sugerido

| Integrante | Explicación | Pregunta cruzada |
|---|---|---|
| Ángel Rodríguez | Mostrar el Waffle Pi, los tópicos y los registros de una orden. | ¿Por qué se exige más de una predicción para confirmar la estación? |
| Henry Jácome | Explicar A*, el mapa inflado, las rutas y la cota temporal. | ¿Cómo mide Gazebo el contacto y qué se excluye del conteo? |
| Luis Lluiguipuma | Explicar las tres distancias, el FIS Mamdani y la decisión del supervisor. | ¿Por qué se congela la base MobileNetV2? |
| Francisco Freire | Explicar la captura, el entrenamiento, la matriz y los errores. | ¿Desde qué posición se calcula la nueva ruta cuando la estación no coincide? |

## Demostración

1. Mostrar el almacén y los cuatro paneles. Lanzar el Waffle Pi con la variante óptica v6.
2. Mostrar una orden y la confirmación al final. Señalar /odom, /scan, /camera/image_raw y /estacion_detectada.
3. Abrir el JSON de la secuencia de diez órdenes. Mostrar semilla, posición de cada caja, contactos y comparación temporal por tramo.
4. Abrir el registro del desvío inducido. Explicar cómo tres imágenes y odometría identifican la estación alternativa, y mostrar el CSV nuevo.
5. Presentar matriz, curva y un fallo visual. Terminar con las limitaciones de las pruebas en simulación.

## Preguntas previsibles

**¿Por qué transferencia de aprendizaje?** La base congelada aporta filtros
aprendidos sobre ImageNet. Con 420 imágenes de entrenamiento se ajusta la cabeza
de cuatro clases, que tiene muchos menos parámetros que la red completa.

**¿Por qué Softmax y entropía cruzada?** Las cuatro estaciones son excluyentes.
Softmax produce probabilidades que suman uno y la entropía cruzada penaliza
asignar poca probabilidad a la etiqueta real.

**¿Cómo se evitó la fuga?** Las imágenes de una misma estación, distancia y
ángulo nominal comparten grupo. El evaluador verifica hashes de archivo y que
cada grupo pertenezca a una sola partición.

**¿Por qué una predicción aislada no cierra la orden?** El robot compara el
destino con tres observaciones recientes, de alta confianza y compatibles con
la odometría. Un resultado incierto puede requerir un giro de alineación.

**¿Qué ocurre si llega a otra estación?** Si se confirma otra estación con
odometría y visión, el ejecutor registra la nueva ruta y ejecuta un reintento
que conserva el destino original.

**¿Cómo se mide el tiempo y la colisión?** El reloj de Gazebo mide la fase de
navegación y el registro conserva por separado el tiempo de reloj real hasta
confirmar la estación. La cota se calcula por tramo como 1,30 × L / 0,20. Los
sensores de contacto cubren las geometrías del robot. Los apoyos con el suelo y
los contactos internos no se cuentan como choques externos.

**¿Qué representa la prueba de diez órdenes?** Una secuencia en Gazebo con una
caja por orden elegida mediante semilla común entre posiciones libres al
costado de los trayectos. No se evaluaron pasillos totalmente bloqueados ni
comportamiento en una plataforma física.
