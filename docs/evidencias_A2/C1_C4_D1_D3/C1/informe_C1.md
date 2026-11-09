# C1 — Diagnóstico de debilidad operacional (Avance 2)

**Fecha del análisis (UTC):** 2026-11-09T01:58:09.965086+00:00  
**Dictamen:** Debilidad operacional principal observada: **retorno prematuro del supervisor a `RUTA` durante la aparición de un obstáculo dinámico**, con oscilación de modos `RUTA`/`DIFUSO`, comandos de orientación incompatibles y detención por la barrera `COLISION_PROYECTADA`.

## Evidencia del fallo original (B3 v01: un ensayo)

- Estado: `NO_APROBADO`; llegada a meta: **NO**; error restante: **1.0112 m**.
- Despeje mínimo geométrico: **0.2601 m** frente a barrera **0.255 m**; reserva final: **0.0051 m**. Esta cifra es una estimación de geometría de SDF, no un sensor de contacto físico.
- Cambios de modo A3 después del estado inicial: **12**; entradas a `DIFUSO`: **6**; retornos de `DIFUSO` a `RUTA`: **5**.
- En controles registrados, hubo **4** muestras con `RUTA` y `scan ≤ 0.35 m`; en **4** de ellas además `df ≥ 0.80 m` (discrepancia entre sensores).
- La primera ventana `DIFUSO`→última transición cubrió **10.94–17.74 s**; frecuencia aproximada de alternancias posteriores al primer `DIFUSO`: **1.618 Hz**.
- La interrupción fue preventiva; **no se ha demostrado una colisión física**.

## Evidencia posterior, NO pareada (B3 v02: cinco ensayos)

- Llegadas con evasión híbrida asistida: **5/5**, bajo la validación registrada por el ejecutor.
- Error final medio: **0.1446 m**; recorrido medio: **2.3900 m**.
- Despeje mínimo medio por ensayo: **0.3133 m**; mínimo entre los cinco ensayos: **0.3124 m**.
- En B3 v02 se añaden **subobjetivos temporales**, giro asistido y retención del bypass. A3 y Mamdani siguen participando, pero no controlan por sí solos toda la maniobra.

## Diagnóstico causal sustentado

1. La ruta A* inicial no incluía la caja, por lo que su aparición exigió reacción online.
2. Con la caja próxima, se observó alternancia entre `DIFUSO` y `RUTA`; hubo instantes con `scan` crítico y `df` elevado, lo que permite recuperar erróneamente el seguimiento de una ruta bloqueada.
3. En v01 se conservó velocidad positiva de avance y la separación a la caja descendió hasta activar `COLISION_PROYECTADA`.
4. La intervención v02 mantuvo la evasión y añadió control geométrico temporal; **su éxito no aísla qué porcentaje de mejora proviene de la histéresis, Mamdani o la asistencia geométrica**.

## Conclusión y límites de C1

**C1: DIAGNÓSTICO IDENTIFICADO Y CUANTIFICADO.** La prioridad de C2–C4 es definir métricas y evaluar el antes/después de manera comparable; **1 fallo v01 y 5 éxitos v02 no son una comparación operacional pareada**. Tampoco se ha medido el contacto real con Gazebo ni reproducido íntegramente cada rosbag. Los CSV y JSON provienen de los seis directorios originales enumerados en `tabla_C1.csv`.

**Recomendación:** no modificar ni volver a ejecutar por el momento los controladores auditados. Llevar esta causa y sus reservas al cierre del Avance 2 y definir si C2–C4 requiere un experimento controlado adicional.
