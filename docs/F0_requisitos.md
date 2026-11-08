# F0.1 — Matriz de requisitos

**Leyenda:** `DOC` obligación del enunciado; `PROP` decisión de ingeniería recomendada, no obligación oficial. Estados del Avance 1 auditados al 2026-11-05; R15–R17 actualizados con pruebas MATLAB del 2026-11-06. Verificado = evidencia existente; Parcial = requiere cierre; Pendiente = requisito no satisfecho.

| ID | Tipo | Requisito | Verificación objetiva | Entrega | Estado |
|---|---|---|---|---|---|
| R01 | DOC | Ubuntu 22.04 con ROS 2 Humble | `echo $ROS_DISTRO` indica `humble` | F0 | Verificado |
| R02 | DOC | Gazebo + TurtleBot3 Waffle Pi | Modelo visible y teleoperable | F0/F1 | Verificado |
| R03 | DOC | Mundo de al menos 4×4 m | Dimensiones y captura del mundo | A1 | Verificado |
| R04 | DOC | ≥3 pasillos con ≥1,0 m de ancho | Cotas del mundo | A1 | Verificado |
| R05 | DOC | Cuatro estaciones con marcadores diferenciados | Captura de cuatro paneles | A1 | Verificado |
| R06 | DOC | Tres sensores emulados `sensor_msgs/Range` | Echo de los 3 tópicos | A1 | Verificado |
| R07 | DOC | Sectores LiDAR de 15°: frontal/izq/der | Ensayo sintetico del emulador; ver docs/F1_R0708_validacion_emulador.md | A1 | Verificado |
| R08 | DOC | Alcance 2 m y ruido gaussiano σ=0,01 m | Ensayo sintetico del emulador; ver docs/F1_R0708_validacion_emulador.md | A1 | Verificado |
| R09 | DOC | Odometría y cámara disponibles | `ros2 topic list -t` y visualización | A1 | Verificado |
| R10 | DOC | Mapa manual sin SLAM | `binaryOccupancyMap`, coordenadas documentadas | A1 | Verificado |
| R11 | DOC | Resolución de mapa ≤0,05 m/celda e inflado de 0,21 m | Mapas de 0,05 m y validación de inflate en MATLAB/Python; ver docs/F1_R11_inflado_matlab.md | A1 | Verificado |
| R12 | DOC | Tres órdenes A* distintas | CSV + tabla longitud/celdas/tiempo | A1 | Verificado |
| R13 | DOC | Seguidor proporcional completa ≥1 orden sin obstáculo | Video o registro ROS | A1 | Verificado |
| R14 | DOC | Reporte de 2 páginas + repositorio | Informe entregado en Blackboard según confirmación del estudiante; repositorio y tag `avance1` verificados | A1 | Verificado |
| R15 | DOC | Mamdani 3 entradas de 0–2 m, 2 salidas | `.fis` validado en MATLAB (3 entradas, 2 salidas, 27 reglas); falta integración ROS 2 | A2 | Validado numéricamente; integración pendiente |
| R16 | DOC | Cobertura de 27 combinaciones lingüísticas | Antecedentes únicos 27/27 y validación con `A2_F1_verificacion_auditoria.m` | A2 | Verificado en MATLAB |
| R17 | DOC | Límites 0≤v≤0,20 m/s y −1,5≤ω≤1,5 rad/s | Rango evaluado en 1.331 casos + diez críticos; falta prueba operativa de saturación y supervisor | A2 | Parcial |
| R18 | DOC | Concordancia MATLAB vs scikit-fuzzy en 10 casos | Tabla de error menor al 2 % | A2 | Verificado: 10/10 casos Python vs MATLAB, error <2%; ver docs/A2_R18_concordancia_MATLAB_Python.md |
| R19 | DOC | Supervisor conmuta ruta/difuso a umbrales indicados | Logs de modos y tópicos | A2 | Pendiente |
| R20 | DOC | 4 escenarios de navegación × 5 repeticiones | Rosbags y métricas | A2 | Pendiente |
| R21 | DOC | Rediseño difuso documentado antes/después | Tabla comparativa de métricas | A2 | Pendiente |
| R22 | DOC | Dataset ≥150 imágenes por estación (600 total) | Conteo, etiquetas y metadatos | A2/F | Pendiente |
| R23 | DOC | Adelanto CNN: captura, 224×224, 70/15/15 y primer entrenamiento | Archivos y entrenamiento ejecutado; si falta: −10/100 en A2 | A2 | Pendiente |
| R24 | DOC | CNN MobileNetV2 y cabeza especificada | Resumen/modelo Keras | Final | Pendiente |
| R25 | DOC | Exactitud CNN ≥90 % | Métricas sobre test y matriz 4×4 | Final | Pendiente |
| R26 | DOC | 10 órdenes consecutivas, 0 colisiones | Logs y tabla de órdenes | Final | Pendiente |
| R27 | DOC | Tiempo de orden ≤1,30×(L_A*/0,20) | Comparación por orden | Final | Pendiente |
| R28 | DOC | IEEE 8 páginas, ≥5 referencias, ≥2 de bibliografía de asignatura | Informe verificable | Final | Pendiente |
| R29 | DOC | Video ≤3 min, repositorio tag `final`, defensa grupal | Artefactos y asistencia | Final | Pendiente |
| R30 | DOC | Robot ESP32 2WD en pista ≥4×4 intersecciones | Demostración física | U4 | Pendiente |
| R31 | DOC | micro-ROS + 3 HC-SR04 + IR + TCS34725 | Lecturas y telemetría física | U4 | Pendiente |
| R32 | DOC | eFLL verificado contra MATLAB y replanificación al bloquear aristas | Tablas, pruebas de rutas | U4 | Pendiente |
| R33 | DOC | k-means k=4, MLP 3→8→4, regresión y KPIs | Resultados de experimentos | U4 | Pendiente |
| R34 | PROP | Un único publicador final de `/cmd_vel` en modo autónomo | Gráfico de nodos y prueba de arbitraje | F0/F2 | Verificado |
| R35 | PROP | Convención única de coordenadas, signos y CSV | SDF/YAML/mapa y 7 CSV verificados; TF map-odom identidad probada e integrada; ver docs/F1_R35_marcos_tf.md | F0/F1 | Verificado |

## Hitos de evaluación del proyecto
A1: 4 nov 2026 (20 %); A2: 9 nov 2026 (35 %); Final: 16 nov 2026 (45 %). La fase real de U4 se califica aparte. Las rúbricas internas de A2 son 35/35/30 y de final 30/30/40.
