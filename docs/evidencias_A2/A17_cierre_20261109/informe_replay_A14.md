# A14 — Auditoría y reproducción de rosbags

Fecha UTC: 2026-11-09T18:29:02.804168+00:00

## Política de versiones

FIS v03: **única versión operativa oficial y del Avance 3**. FIS v04: variante de comparación A13, no sustituye v03.

## Auditoría

Dictamen: `A14_30_ROSBAGS_ACTUALES_INTEGROS_AUDITORIA_SQLITE_SHA256`

Las 5 grabaciones B1 se obtuvieron en 5 recorridos adicionales; no se atribuyen retrospectivamente a A12. Las bases de B2–B4 y A13 se validan en sus directorios originales, sin duplicarlas.

| Grupo | Rosbags | Modalidad |
|---|---:|---|
| B1 | 5/5 | Seguimiento de ruta |
| B2 | 5/5 | A* / obstáculo estático, sin actuación DIFUSO observada |
| B3 | 5/5 | Híbrida asistida con DIFUSO actuante |
| B4 | 5/5 | Centrado geométrico asistido, FIS en sombra |
| A13 | 10/10 | Cinco pares A(v03)/B(v04), experimento aislado |

Bases históricas inventariadas: 88 (no se confunden con las 30 nuevas).

## Reproducción ROS aislada

Dictamen: `A14_SEIS_ROSBAGS_REPRODUCIDOS_SIN_ACTUACION_ORIGINAL`

Dominio ROS: `214`, localhost exclusivamente. `/cmd_vel` se remapea a `/a14_replay/cmd_vel`. Gazebo ausente.

| Muestra | /scan recibidos | /odom recibidos | cmd_vel remapeados |
|---|---:|---:|---:|
| B1 | 130 | 756 | 6112 |
| B2 | 180 | 1045 | 336 |
| B3 | 273 | 1598 | 517 |
| B4 | 174 | 1015 | 331 |
| A13_A | 270 | 1578 | 526 |
| A13_B | 278 | 1623 | 538 |

**Nota:** Esta reproducción demuestra recepción en dominio aislado. No es una nueva prueba de navegación ni reenvía comandos al robot.

## Integridad

La carpeta A14 contiene manifiesto SHA256 propio y referencias a las fuentes originales; no modifica ninguna evidencia preexistente.

