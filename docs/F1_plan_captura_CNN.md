# Tarea 1.7 — Plan de captura de imágenes para CNN

## 1. Objetivo

Planificar la adquisición de imágenes de las cuatro
estaciones del almacén simulado, utilizando la cámara
del TurtleBot3 Waffle Pi.

Este documento es un protocolo de adquisición.
No representa un dataset ya capturado.

## 2. Clases

- RECEPCION: panel azul, I.
- ALMACENAMIENTO: panel rojo, II.
- EMPAQUE: panel verde, III.
- DESPACHO: panel amarillo, IV.

## 3. Matriz de adquisición

Para cada estación:

Distancias (m):
0.30, 0.60, 0.90, 1.20, 1.50.

Ángulos (grados):
-30, -15, 0, +15, +30.

Capturas independientes por combinación: 6.

Total por estación: 5 x 5 x 6 = 150.
Total general planificado: 600 imágenes.

La distancia se mide entre la cámara y el panel.
El ángulo se define respecto a la normal frontal
del panel.

## 4. Procedimiento

1. Iniciar Gazebo con el almacén corregido.
2. Comprobar la publicación de /camera/image_raw.
3. Posicionar el robot frente a la estación.
4. Ajustar distancia y ángulo de observación.
5. Adquirir seis imágenes independientes.
6. Introducir variaciones controladas de iluminación,
   posición o condiciones de fondo cuando sea posible.
7. Evitar capturar ráfagas de fotogramas idénticos.
8. Registrar la posición, el ángulo y la sesión.
9. Inspeccionar visualmente cada imagen.
10. Repetir para las 25 posiciones de cada estación.

No capturar otros identificadores como clase positiva
de la estación objetivo.

## 5. Organización de datos

Archivo maestro planificado:
datos/cnn/plan_adquisicion.csv

Imágenes futuras:
datos/cnn/imagenes/<particion>/<estacion>/<archivo>.png

Particiones:
- train: entrenamiento.
- val: validación.
- test: prueba.

El archivo CSV distingue los registros PLANIFICADA
de imágenes efectivamente capturadas y verificadas.

## 6. División de datos

División global prevista:
- Entrenamiento: 420 imágenes (70 %).
- Validación: 90 imágenes (15 %).
- Prueba: 90 imágenes (15 %).

Las seis capturas de una misma combinación
estación-distancia-ángulo pertenecen siempre
a la misma partición.

Así se evita repartir repeticiones de una posición
de cámara entre entrenamiento y evaluación.

Se utilizará una semilla fija para reproducibilidad.

## 7. Preprocesamiento futuro

- Verificar imágenes válidas y etiquetas.
- Convertir a RGB cuando corresponda.
- Redimensionar a 224 x 224 píxeles.
- Aplicar el preprocesamiento correspondiente
  a la arquitectura CNN seleccionada.
- Aumentación de datos exclusivamente en train.
- Mantener val y test separados del entrenamiento.

El redimensionamiento no debe sobrescribir
irreversiblemente las imágenes originales.

## 8. Control de calidad

Registrar por imagen:
- Identificador único.
- Estación.
- Distancia.
- Ángulo.
- Número de captura.
- Partición asignada.
- Ruta de archivo.
- Estado de adquisición.

Rechazar imágenes ilegibles, corruptas, duplicadas
o con identificación ambigua.

Si se reemplaza una imagen, respetar su partición
para evitar contaminación entre conjuntos.

## 9. Criterios de aceptación del dataset futuro

- Al menos 150 imágenes válidas por estación.
- Cobertura de distancias de 0.30 a 1.50 m.
- Cobertura angular de -30 a +30 grados.
- Etiquetas y metadatos completos.
- División documentada 70/15/15.
- Ausencia de duplicados entre particiones.

## 10. Estado

Tarea 1.7: protocolo de captura definido.

R22: pendiente de adquisición real de imágenes.
R23: pendiente de preparación y entrenamiento CNN.

No se afirma que las 600 imágenes existan todavía.
