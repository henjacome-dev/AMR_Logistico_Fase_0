# R11 — Inflado de obstáculos y planificación A* en MATLAB

Fecha: 2026-11-05
Rama: correccion-avance1-r04

## Objetivo

Validar la generación del mapa inflado mediante
inflate(mapa, 0.21) y la planificación A* sobre
el escenario geométrico corregido.

## Método

- MATLAB Online.
- binaryOccupancyMap.
- Resolución: 0.05 m/celda.
- Inflado solicitado: 0.21 m.
- Planificador: plannerAStarGrid.
- Cuatro trayectos y tres órdenes logísticas.

## Comparación de inflado

- Celdas ocupadas Python: 10699.
- Celdas ocupadas MATLAB: 12065.
- Celdas adicionales MATLAB: 1366.
- Celdas exclusivas Python: 0.

El mapa MATLAB resulta más conservador.
Las diferencias se documentan y no se
interpretan automáticamente como un error.

## Resultados A* MATLAB

- Orden A: 20.100 m, 403 nodos.
- Orden B: 32.702 m, 607 nodos.
- Orden C: 26.100 m, 523 nodos.

La orden A es la menor de las tres evaluadas.

Se calcularon satisfactoriamente cuatro rutas
individuales y tres órdenes logísticas.

## Evidencias

Código:
matlab/R11_MATLAB/R11_inflado_astar_corregido.m

Mapas:
matlab/R11_MATLAB/almacen_f1.pgm
matlab/R11_MATLAB/almacen_f1_inflado_python.pgm

Resultados:
matlab/R11_MATLAB/resultados/

Incluyen tablas CSV, trayectorias y visualizaciones.

## Trazabilidad

Los resultados anteriores de matlab/F124_MATLAB/
permanecen intactos y pertenecen al escenario original.

Los resultados R11 corresponden al escenario
geométrico corregido después de R04.

## Dictamen

R11: verificado mediante ejecución en MATLAB Online,
comparación de inflado y planificación A*.

No se repitieron pruebas dinámicas en Gazebo.
