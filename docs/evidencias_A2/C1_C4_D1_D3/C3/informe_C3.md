# C3 — Evaluación del rediseño operacional: B3

**Fecha UTC:** 2026-11-09T02:15:41.475233+00:00
**Dictamen:** REDISENO_HIBRIDO_ASISTIDO_OBSERVADO_CON_RESERVAS

## Diagnóstico y acción correctiva

En B3 v01, A3 conmutó repetidamente entre `RUTA` y `DIFUSO` ante una caja insertada durante el desplazamiento. La ruta A* original continuó habilitándose aunque el LiDAR mostraba peligro; la protección detuvo el robot antes de llegar a la meta.

B3 v02 combinó retención evasiva, bloqueo preventivo del avance, giro asistido y subobjetivos geométricos temporales. **La intervención es híbrida y compuesta.** A3 y Mamdani participaron, pero el efecto de sus propuestas no puede separarse del guiado geométrico introducido.

## Comparación descriptiva de ensayos

| Indicador | B3 v01 (n=1) | B3 v02 (n=5) |
|---|---:|---:|
| Llegadas | **0/1** | **5/5** |
| Detenciones preventivas `COLISION_PROYECTADA` | 1 | 0 observadas |
| Distancia minima geometrica de corrida (m) | 0.2601 | 0.3133 (media; +20.45%) |
| Tiempo para 0.60m posteriores al spawn (s) | 8.7804 | 10.4563 (media; +19.09%) |
| Giro absoluto medio en ventana (rad/s) | 0.1315 | 0.1145 (media; -12.91%) |
| Indice de variacion angular por tiempo | 0.4647 | 0.1534 (media; -67.00%) |
| Tiempo de A3 en DIFUSO (%) | 16.3140 | 6.1350 (media; -62.39%) |
| Cambios de A3 muestreados en ventana | 3.0000 | 2.4000 (media; -20.00%) |
| Conmutaciones A3 registradas en corrida | 12.0000 | 5.8000 (media; -51.67%) |

El despeje se calcula geométricamente respecto de objetos del SDF. No es un evento de contacto físico.
Los porcentajes se calculan sobre **medias descriptivas**, con muestras no equivalentes.

## Criterios técnicos y alcance

1. **Seguridad geométrica observada:** las cinco ejecuciones asistidas permanecieron por encima del margen de 0.255 m, según su auditoría de trayectorias.
2. **Eficacia observada del sistema:** las cinco ejecuciones asistidas llegaron a la meta y no registraron `COLISION_PROYECTADA`. No significa tasa de éxito poblacional del 100 %.
3. **Coste de la intervención:** comparar la duración de la ventana común y el giro, que pueden mejorar en sentidos opuestos.
4. **No atribuibilidad aislada:** el aporte de subobjetivos geométricos, conmutación asistida y Mamdani no se ha separado con un ensayo de ablación.
5. **Controles y trazabilidad:** C1 y C2 coinciden en la línea base. Se validaron hashes de 18 archivos originales; seis rosbags comprobados solo por presencia.
6. **Limitaciones:** n=1 antes, n=5 después, sin aleatorización ni emparejamiento; no se midieron contactos físicos de Gazebo.

## Dictamen C3

**Aceptación operacional descriptiva con reservas:** el sistema híbrido B3 v02 superó en estas cinco ejecuciones la falla de llegada de B3 v01, conservando la barrera de seguridad. No se demuestra mejora causal aislada del controlador Mamdani ni un experimento A/B estadísticamente controlado.

## Enlace con C4 y entrega

C4 debe presentar en una sola tabla la debilidad inicial, las intervenciones combinadas, el resultado antes/después y sus reservas. Para el informe académico usar el lenguaje «mejora observada del sistema híbrido asistido»; no «optimización demostrada del FIS».

**Fuentes:**
- /home/enry/AMR_A2_entrega_local/C1_auditoria_20261109_015809_UTC/dictamen_C1.json
- /home/enry/AMR_A2_entrega_local/C2_linea_base_20261109_021010_UTC/dictamen_C2.json
- 18 fuentes originales validadas y manifiesto SHA256 anexado.
