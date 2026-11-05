# B4 — Consolidación del mapa y planificación A*

Fecha: 2026-11-05
Proyecto: AMR logístico — ROS 2 Humble y Gazebo Classic.

## 1. Mapa del almacén

- Dimensiones: 10 × 8 m.
- Resolución: 0,05 m/celda.
- Radio de inflado configurado: 0,21 m.
- Origen: (-5, -4) m.
- Generador: tools/generar_mapa_f1.py.
- Mapa base: mapas/almacen_f1.pgm.
- Mapa inflado: mapas/almacen_f1_inflado.pgm.
- Origen de datos: configuración manual, sin SLAM.

La construcción del mapa y su inflado se realizan
mediante el generador Python. MATLAB importa el
mapa inflado mediante binaryOccupancyMap.

## 2. Planificación A* en MATLAB

Archivos principales:
- matlab/F124_MATLAB/F124_validacion_astar.m
- matlab/F124_MATLAB/F124_comparacion_ordenes.m

Planificador: plannerAStarGrid.
Búsqueda diagonal: habilitada.

## 3. Resultados de rutas MATLAB

| Trayecto | Longitud (m) |
|---|---:|
| Robot - Recepción | 1,0000 |
| Recepción - Almacenamiento | 5,5500 |
| Almacenamiento - Empaque | 8,2899 |
| Empaque - Despacho | 5,5500 |

Fuente: F124_resumen_rutas.csv.

## 4. Comparación de órdenes

| Orden | Python (m) | MATLAB (m) | Diferencia (m) |
|---|---:|---:|---:|
| A | 20,449 | 20,3899 | -0,0591 |
| B | 32,817 | 32,6409 | -0,1761 |
| C | 26,449 | 26,3899 | -0,0591 |

La diferencia absoluta máxima es 0,1761 m,
equivalente al 0,537 % de la longitud Python
de la orden B.

La orden A presenta la menor longitud entre
las tres órdenes evaluadas.

Fuentes:
- matlab/F124_MATLAB/resultados/F124_comparacion_final.csv
- matlab/F124_MATLAB/resultados/F124_comparacion_ordenes.csv

## 5. Trazabilidad

La copia del mapa inflado utilizada por MATLAB
coincide con la versión del repositorio principal.

Evidencias disponibles:
- CSV de rutas y órdenes.
- Comparación MATLAB-Python.
- Figura F124_rutas_astar.png.
- Scripts MATLAB utilizados.

## 6. Limitaciones y pendientes

- No se efectuaron nuevos cálculos en esta auditoría.
- La comparación no demuestra trayectorias idénticas.
- La correspondencia geométrica mundo-mapa se
  comprobó en EX-01, con dos advertencias de paso.
- No se ha acreditado la disponibilidad de MATLAB
  y sus toolboxes en la instalación local de Ubuntu.
