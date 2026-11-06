# R04 — Correccion geometrica de pasillos

Fecha: 2026-11-05
Rama: correccion-avance1-r04

## 1. Modificacion

Se conservaron las paredes, los estantes y las cuatro
estaciones del almacen.

Se reposicionaron exclusivamente dos obstaculos:

- caja_prueba_1: de (0.80, 0.00) a (0.00, -0.70) m.
- caja_prueba_2: de (-1.50, 2.65) a (-1.50, 2.25) m.

## 2. Verificacion geometrica

- Mundo SDF y configuracion YAML: coherentes.
- Mapas base e inflado: coherentes celda a celda.
- Dimensiones del mapa: 200 x 160 celdas.
- Resolucion: 0.05 m/celda.
- Inflado: 0.21 m.
- Cuatro estaciones accesibles.
- Tres pasillos sin advertencias de ancho.

Evidencia:
resultados/EX01_auditoria_R04_v2.txt

## 3. Compatibilidad de rutas

Se comprobaron cuatro rutas individuales y tres
ordenes logisticas previamente generadas.

Resultados:
- Siete rutas con cero waypoints ocupados.
- Cero conflictos diagonales bloqueados.

Esta comprobacion acredita compatibilidad estatica,
no una nueva validacion de seguimiento en Gazebo.

## 4. Replanificacion Python A*

Longitudes originales y corregidas:

- Orden A: 20.449 m -> 20.200 m.
- Orden B: 32.817 m -> 32.568 m.
- Orden C: 26.449 m -> 26.200 m.

Las cuatro rutas individuales se recalcularon.
La orden A continua siendo la menor de las
tres alternativas evaluadas.

Evidencias:
resultados/R04_astar_mapa_corregido/

## 5. Trazabilidad

Los resultados F1.2 y F1.3 previamente publicados
corresponden al escenario original.

Se conservaron los CSV originales y las evidencias
de las pruebas de seguimiento y seguridad.

El paquete matlab/F124_MATLAB/ corresponde al
escenario original. Sus resultados no deben
presentarse como calculados sobre el mapa R04.

La actualizacion MATLAB para la geometria corregida
queda pendiente de ejecucion y documentacion.

## 6. Alcance

R04: comprobacion geometrica estatica satisfactoria.

No se ejecutaron nuevamente pruebas dinamicas de
navegacion ni ensayos de seguridad.

La visualizacion del escenario corregido en Gazebo
puede documentarse junto con la revision de R05.
