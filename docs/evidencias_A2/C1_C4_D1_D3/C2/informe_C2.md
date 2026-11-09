# C2 — Línea base y protocolo de comparabilidad operacional

**Fecha de auditoría (UTC):** 2026-11-09T02:10:10.062068+00:00
**Escenario:** B3, obstáculo dinámico aparecido a ~1 m del robot. 
**Alcance:** 1 ejecución B3 v01 detenida por barrera, 5 ejecuciones B3 v02 con navegación híbrida asistida.

## Métricas de las corridas completas (descriptivas)

| Indicador | B3 v01 (n=1) | B3 v02 (n=5) |
|---|---:|---:|
| Llegadas registradas | 0/1 | 5/5 |
| Interrupción `COLISION_PROYECTADA` | 1 | 0 de 5 |
| Distancia total recorrida (m) | 1.000 | 2.390 (media) |
| Distancia mínima geométrica (m) | 0.2601 | 0.3133 (media por ensayo) |
| Error final a meta (m) | 1.011 | 0.145 (media) |
| Cambios A3 (total en evento JSON) | 12 | 5.80 (media) |

**Advertencia:** los recorridos completos NO tienen el mismo alcance temporal ni espacial: v01 terminó prematuramente; las diferencias globales no representan un efecto causal.

## Ventana descriptiva normalizada: 0,60 m recorridos desde el spawn de la caja

Se interpola el instante en que cada ensayo acumula 0,60 m de desplazamiento **odometrico** desde la inserción real en Gazebo; esto no usa las distancias euclidianas del punto de partida.

| Indicador ventana | B3 v01 | B3 v02: media [mín, máx] |
|---|---:|---:|
| Duración de ventana (s) | 8.78 | 10.46 [10.38, 10.57] |
| Despeje geométrico mínimo (m) | 0.3975 | 0.4746 [0.4720, 0.4806] |
| |v| media ponderada (m/s) | 0.0692 | 0.0570 [0.0564, 0.0573] |
| |ω| medio ponderado (rad/s) | 0.1315 | 0.1145 [0.1115, 0.1173] |
| Σ|Δω| / duración (rad/s², índice discreto) | 0.465 | 0.153 [0.147, 0.164] |
| Tiempo A3 en DIFUSO (%) | 16.31 | 6.13 [3.82, 7.76] |
| Tiempo con velocidad lineal cero (%) | 3.83 | 13.36 [12.84, 14.10] |
| Cambios A3 muestreados (conteo) | 3.00 | 2.40 [2.00, 4.00] |
| Muestras RUTA con scan≤0,35 y df≥0,80 | 0.00 | 0.00 [0.00, 0.00] |
| Comandos de asistencia geométrica | 0.00 | 41.20 [40.00, 43.00] |

### Condiciones compartidas y variables de confusión

- Coinciden el escenario B3, la posición inicial nominal, la meta, la ruta A* inicial de 2 m y el concepto de inserción dinámica a 1 m; las poses efectivas y el ruido de sensor pueden variar.
- La ventana iguala **recorrido odométrico desde la inserción**, no condiciones sensoriales, tiempo, trayectorias ni órdenes de control.
- **Intervención combinada:** v02 introduce retención de evasión, subobjetivos geométricos y giros asistidos; no es un cambio único aislado.
- v01 es **una sola ejecución** y v02 cinco; no hay aleatorización, réplica baseline, asignación pareada ni control cruzado.
- No inferir p-valores, significancia estadística, reducción causal de colisiones ni mayor rendimiento intrínseco de Mamdani.
- El despeje es geométrico según SDF; **no hay medición directa de contactos físicos**. Los rosbags se verifican por existencia, no se reproducen.

## Criterios para C3 y C4

1. **C3:** aceptar los resultados anteriores solo como contraste descriptivo del sistema completo y justificar por qué el arbitraje original no conservaba la evasión.
2. **C3 experimental (opcional, si la rúbrica exige mejora causal):** ejecutar múltiples réplicas v01 y v02 bajo las mismas semillas de ruido, condiciones de spawn y política de parada, sin reducir márgenes de seguridad. Si una variante incorpora subobjetivos, declarar exactamente esa intervención.
3. **C4:** producir tabla antes/después con N, resultados, mecanismo, errores y reservas, sin llamar A/B pareado a este conjunto.
4. Conservar intactos A3, A4, FIS y todos los archivos originales; mantener la evidencia CNN y el cierre documental de D.

## Dictamen C2

**LINEA_BASE_Y_METRICAS_OPERACIONALES_DEFINIDAS_CON_RESERVAS.** Establece una comparación descriptiva reproducible; todavía NO demuestra causalidad ni optimización aislada de la lógica difusa.

Las seis rutas de origen y los hashes SHA256 están en `tabla_C2.csv` y `manifiesto_SHA256.csv`.
