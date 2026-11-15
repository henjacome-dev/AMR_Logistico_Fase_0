# Etapa 6 — Cierre técnico de optimización R27

## 1. Objetivo

Optimizar el tiempo de navegación entre nodos del
AMR Logístico, conservando el funcionamiento de
A*, supervisor, Mamdani v03 y MobileNetV2.

## 2. Criterio de aceptación R27

T_navegación <= 1.30 × (L_A* / 0.20)

Por confirmación del docente, se contabiliza
exclusivamente el desplazamiento entre nodos,
incluidos los giros requeridos durante la ruta.

Se excluyen el reconocimiento CNN y la alineación
angular posterior a la llegada.

## 3. Resultados individuales

| Origen | Destino | Navegación (s) | Límite (s) | Margen (s) |
|---|---|---:|---:|---:|
| ROBOT | ALMACENAMIENTO | 39.748 | 42.900 | +3.152 |
| ALMACENAMIENTO | EMPAQUE | 48.237 | 53.619 | +5.382 |
| EMPAQUE | DESPACHO | 31.545 | 36.400 | +4.855 |
| DESPACHO | RECEPCION | 112.416 | 126.419 | +14.003 |

**Resultado: 4/4 tramos aprobados.**

## 4. Resultados consecutivos

| Origen | Destino | Navegación (s) | Límite (s) | Margen (s) |
|---|---|---:|---:|---:|
| RECEPCION | ALMACENAMIENTO | 33.022 | 36.400 | +3.378 |
| ALMACENAMIENTO | EMPAQUE | 48.334 | 53.618 | +5.284 |
| EMPAQUE | DESPACHO | 31.519 | 36.400 | +4.881 |
| DESPACHO | RECEPCION | 111.460 | 126.419 | +14.959 |

**Resultado: 4/4 tramos aprobados.**

Las cuatro misiones consecutivas finalizaron con
reconocimiento CNN y precisión posicional válida.

## 5. Integridad y trazabilidad

- Referencia funcional original: `73793ac`.
- Rama de desarrollo: `integracion-final`.
- Seguidor y supervisor originales sin cambios.
- Candidata R27 desarrollada en archivos separados.
- Velocidad lineal propuesta máxima observada:
  0.185 m/s.
- Auditoría temporal basada en telemetría del
  seguidor y longitudes de planificación A*.

### SHA-256 de la candidata

- `tools/A3_E6_R27_preparar_candidata.py`: `327154f5a507ac5ae6ffc62dc578402b87ba96df4c6eb4733fbfe3adac629644`
- `tools/A3_E6_R27_seguidor.py`: `b17489cb465d0af97a6d4daa1dae9210f3bd8f545ab54c9fdabee876db008c0e`
- `tools/A3_E6_R27_supervisor.py`: `e13008c0000d5ba249aefa468523dfaea89b3b5070767a5c5843c78e408a7c21`
- `tools/A3_E6_R27_ejecutor.py`: `2f7fc0afd0bb24156cdb3961f17047b1d8aed9158848d4d951f0fec5386622fb`

## 6. Evidencias

- `resultados/A3_E6_R27_pilotos/auditoria_consolidada_R27_20261115_164747.json`
- `resultados/A3_E6_R27_pilotos/auditoria_consecutiva_R27_20261115_165037_389637.json`

Los registros originales de ejecución y la
telemetría detallada se conservan localmente.

## 7. Alcance y limitaciones

- Validación realizada en Gazebo con TurtleBot3.
- Ocho tramos de navegación evaluados.
- Una secuencia completa de cuatro misiones.
- No se realizó una segunda prueba de repetibilidad.
- No se demuestra fiabilidad estadística.
- No se realizó validación en un robot físico.
- La velocidad registrada corresponde a la
  propuesta de control, no a una certificación
  independiente de velocidad física.

## 8. Conclusión

La candidata optimizada cumple R27 en los ocho
tramos evaluados y conserva la integración funcional
de las cuatro estaciones.

La Etapa 6 queda técnicamente validada para los
ensayos efectuados, con repetibilidad adicional
expresamente no evaluada.

Pendientes administrativos:
respaldo Git y continuación de pruebas de robustez.
