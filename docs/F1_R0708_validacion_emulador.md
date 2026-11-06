# R07 y R08 — Validacion del emulador de sensores

Fecha: 2026-11-05
Entorno: ROS 2 Humble
Metodo: ensayo aislado con LaserScan sintetico.

## Objetivo

Comprobar las orientaciones, los sectores de medicion,
el alcance maximo y el comportamiento estadistico
del ruido del emulador real.

## R07 — Configuracion angular

Sensores:
- Frontal: centro 0 grados.
- Izquierdo: centro +45 grados.
- Derecho: centro -45 grados.

Resultados:
- Apertura sectorial: 15 grados.
- Mediciones a los tres centros: satisfactorias.
- Desplazamientos de +/-7 grados: incluidos.
- Desplazamientos de +/-8 grados: excluidos.
- Sin contaminacion entre sectores.

Estado R07: Verificado mediante ensayo sintetico.

## R08 — Alcance y ruido

- Alcance maximo: 2.0 m.
- Desviacion estandar configurada: 0.01 m.
- Muestras por sensor: 600.
- Total de muestras estadisticas: 1800.
- Semilla de reproducibilidad: 20261105.

Resultados:

| Sensor | Distancia base (m) | Media (m) | Sigma (m) |
|---|---:|---:|---:|
| Frontal | 0.700 | 0.700508 | 0.010093 |
| Izquierdo | 1.100 | 1.100382 | 0.010029 |
| Derecho | 1.400 | 1.399916 | 0.010117 |

Criterios de aceptacion:
- Error absoluto de media <= 0.003 m.
- Sigma muestral entre 0.008 y 0.012 m.

Los tres sensores cumplieron ambos criterios.
La saturacion a 2.0 m fue satisfactoria.

Estado R08: Verificado mediante ensayo sintetico.

## Evidencias

Codigo:
tools/prueba_r0708_emulador.py

Registros:
resultados/R0708_validacion/resumen.csv
resultados/R0708_validacion/muestras.csv
resultados/R0708_validacion/informe.txt

El verificador utiliza el callback real de:
tools/emulador_sensores_f1.py

## Limitaciones

Las pruebas emplearon lecturas LiDAR sinteticas.
No se ejecutaron ensayos de precision fisica,
deteccion dinamica ni movimiento del robot.

Los resultados estadisticos son compatibles con
el ruido configurado; no constituyen una prueba
formal de normalidad de la distribucion.

No se modifico el codigo del emulador.
