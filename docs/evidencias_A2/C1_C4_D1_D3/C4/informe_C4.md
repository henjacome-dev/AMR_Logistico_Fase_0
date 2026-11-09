# C4 — Cierre comparativo del rediseño operacional B3

**Fecha UTC:** 2026-11-09T02:26:39.932944+00:00
**Dictamen:** CIERRE_DESCRIPTIVO_HIBRIDO_ASISTIDO_CON_RESERVAS
**Fuentes:** C1 (diagnóstico), C2 (línea base), C3 (evaluación). Fuente inmediata: `/home/enry/AMR_A2_entrega_local/C3_evaluacion_20261109_021541_UTC/dictamen_C3.json`.

## Objetivo y diseño comparable

Documentar la respuesta del sistema móvil ante un obstáculo insertado dinámicamente a 1 m. B3 v01 representa el sistema inicial (un ensayo detenido), mientras B3 v02 incorpora **retención de evasión, giro preventivo y subobjetivos geométricos temporales** (cinco ensayos). Se utiliza una ventana común de **0,60 m odométricos desde la inserción del obstáculo** para comparar duración y giro. El despeje se refiere al mínimo de la corrida completa, no exclusivamente a esa ventana.

## Matriz antes/después

| Indicador | Antes (v01, n=1) | Después (v02, n=5; media) | Variación |
|---|---:|---:|---:|
| Llegadas | 0/1 | 5/5 | No interpretable como tasa poblacional |
| `COLISION_PROYECTADA` | 1 | 0 registradas | Diferente tamaño muestral |
| Despeje minimo por corrida | 0.2601 m | 0.3133 m | +20.45% |
| Duracion ventana odometrica comun de 0.60 m | 8.7804 s | 10.4563 s | +19.09% |
| Giro absoluto medio en ventana comun | 0.1315 rad/s | 0.1145 rad/s | -12.91% |
| Variacion angular por tiempo en ventana | 0.4647 indice | 0.1534 indice | -67.00% |
| Tiempo A3 DIFUSO en ventana | 16.3140 % | 6.1350 % | -62.39% |
| Cambios muestreados A3 en ventana | 3.0000 eventos | 2.4000 eventos | -20.00% |
| Conmutaciones A3 por corrida completa | 12.0000 eventos | 5.8000 eventos | -51.67% |
| Distancia total recorrida | 1.0000 m | 2.3900 m | +139.00% |

## Interpretación del resultado

- **Llegada y seguridad geométrica observadas:** B3 v02 obtuvo 5/5 llegadas y 0 detenciones preventivas registradas; el mínimo de despeje posterior entre los cinco ensayos fue 0.3124 m, por encima de la barrera de 0.255 m.
- **Despeje:** 0.2601 m antes versus 0.3133 m de media después; diferencia descriptiva de 0.0532 m.
- **Tiempo de recorrido comparable:** 8.7804 s antes versus 10.4563 s después (+19.09%). La maniobra asistida resultó más lenta en esta ventana.
- **Giro absoluto medio:** 0.1315 frente a 0.1145 rad/s (-12.91%). Menor giro medio no prueba por sí solo menor oscilación.

## Reservas obligatorias para el informe docente

1. Comparación descriptiva **no pareada**, con n=1 antes y n=5 después; no se realizó aleatorización.
2. Cambió la **arquitectura de control**: el efecto del supervisor y Mamdani no se puede aislar del desvío y centrado geométricos. No afirmar «Mamdani mejoró un X %».
3. No se midieron directamente los **contactos físicos** de Gazebo; el despeje es geométrico.
4. Se verificó la presencia de seis rosbags, pero **no** su reproducción.
5. La ventana común está igualada por **distancia odométrica**, no implica igualdad de todos los estados iniciales, perturbaciones ni comandos.
6. El experimento R21 sobre FIS v01/v02 es una comparación **numérica separada**; no constituye esta validación operacional.

## Dictamen de cierre C1–C4

**Aceptación descriptiva condicionada del rediseño híbrido asistido**: el sistema posterior completó cinco recorridos donde la ejecución original se detuvo. Se conservan el coste temporal y todas las reservas de atribución causal. Si la rúbrica exige ensayo A/B pareado o prueba exclusiva de lógica difusa, esa exigencia permanece sin acreditar con estos datos.

## Continuidad hacia D1–D7

Utilizar esta matriz como evidencia C4 en el informe de dos páginas, junto a las pruebas B1–B4, la concordancia MATLAB/Python y la CNN preliminar. Conservar la fuente completa y su manifiesto SHA256. La generación de este reporte no publica en GitHub ni entrega en Blackboard.
