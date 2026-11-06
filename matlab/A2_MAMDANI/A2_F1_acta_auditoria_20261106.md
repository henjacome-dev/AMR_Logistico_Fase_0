# Acta de auditoría — Avance 2, Fase 1

**Fecha:** 6 de noviembre de 2026  
**Entorno:** MATLAB Online  
**Repositorio:** `henjacome-dev/AMR_Logistico_Fase_0`  
**Rama:** `desarrollo-avance2`  
**Commit de reparación de evidencias:** `6b46367c16d07540957022e14e0eeade340d7974`

## Evidencia verificada

Conforme a la salida de `A2_F1_verificacion_auditoria.m` mostrada por el estudiante en MATLAB Online:

| Comprobación | Resultado |
|---|---|
| Archivos técnicos básicos | Presentes |
| Tres MAT | Cargados con `load()`; variables internas comprobadas |
| FIS | Tres entradas, dos salidas, 27 reglas y 27 antecedentes distintos |
| CSV | Concordancia con MAT: barrido de 1.331 entradas y diez casos críticos |
| Reproducción `evalfis` del barrido | Error máximo mostrado: 0 |
| Reproducción `evalfis` de casos críticos | Error máximo mostrado: 0 |
| PNG | Nueve imágenes leídas y verificadas |

El error máximo reportado como cero corresponde a los datos evaluados por el script; no significa una prueba de todos los puntos posibles del universo de entrada.

## Condición crítica y limitaciones

Caso de tres obstáculos próximos (`df=di=dd=0,2 m`):

- `v = 0,011815589 m/s`.
- `omega = 0 rad/s`.

**La inferencia Mamdani por sí sola no garantiza parada estricta.** La lógica de seguridad corresponde al supervisor ROS 2 y requiere pruebas separadas.

Esta auditoría no acredita saturación física de actuadores, ejecución integrada en ROS 2, equivalencia con `scikit-fuzzy`, navegación sin colisiones en Gazebo ni robustez en hardware real.

## Dictamen

**Fase 1 validada en diseño numérico, reproducción MATLAB e integridad de evidencias.** El paso posterior es fortalecer y versionar el cierre documental. R15 y R16 quedan validados a nivel de diseño; R17 se mantiene parcial por la validación operacional pendiente. R18–R21 continúan pendientes.

El archivo `AMR_Mamdani_A2_v01.fis` debe conservarse sin modificaciones.
