# Auditoría consolidada — Avance 2 / R19–R23

**Dictamen técnico:** AUDITORIA TECNICA FAVORABLE CON RESERVAS

Verificaciones: **57**, fallos críticos: **0**, alertas: **1**.

**Importante:** esta auditoría lee las evidencias locales, no equivale a entrega académica oficial.

## Carpetas auditadas

- **R19:** `/home/enry/AMR_Logistico_Fase_0/resultados/A2_R19_integral_gazebo/20261107_222843`
- **R20:** `/home/enry/AMR_Logistico_Fase_0/resultados/A2_R20_integral_4x5/20261107_224326`
- **R21:** `/home/enry/AMR_Logistico_Fase_0/resultados/A2_R21_comparacion_Mamdani_AB/20261108_010146`
- **R22_R23:** `/home/enry/AMR_Logistico_Fase_0/resultados/A2_R22_R23_CNN_integral_v6_contexto/20261108_004923`

## Verificaciones

- [x] Repositorio Git disponible
- [x] HEAD es la base auditada E2.3.6A
- [x] Rama desarrollo-avance2
- [x] FIS oficial v01 mantiene integridad de Git
- [ ] Sin cambios PREEXISTENTES en archivos rastreados (no se modifica ninguno)
- [x] Evidencia completa R19 localizada
- [x] Estado R19: prevalidacion experimental aprobada
- [x] Siete transiciones/etapas R19 observadas
- [x] Sin fallos de seguridad declarados por R19
- [x] R19 conserva advertencia: no acreditado formalmente
- [x] Evidencia completa R20 localizada
- [x] R20: resultado experimental 20/20
- [x] 20 filas CSV con estado OK
- [x] Matriz de rutas y repeticiones completa y sin duplicados
- [x] 20 errores de meta <=0.10m
- [x] Conteos positivos por ensayo de /cmd_vel, /odom y /scan
- [x] 20 carpetas rosbag2 con metadatos y bases SQLite presentes (20/20)
- [x] R20 se ejecutó sin comandos DIFUSO; no prueba mejoras de evasión
- [x] Evidencia completa R21 localizada
- [x] R21: resumen_R21.json presente
- [x] R21: comparacion_20_ensayos.csv presente
- [x] R21: resumen_4_escenarios.csv presente
- [x] R21: observaciones_AB.csv presente
- [x] R21: panel_R21.html presente
- [x] R21: informe_R21.md presente
- [x] R21: AMR_Mamdani_A2_v02_experimental.fis presente
- [x] 20 pares, 1.440 inferencias y tabla de 20 comparaciones
- [x] Referencia del FIS v01 coincide
- [x] SHA256 del FIS v02 experimental conforme
- [x] 20 pares únicos por escenario y repetición
- [x] A/B sin violaciones numéricas de parada ni límites
- [x] R21: aumento de variación angular lateral reconocido (resultado mixto)
- [x] Evidencia completa R22/R23 v6 localizada
- [x] Dataset: resumen_dataset.json presente
- [x] Dataset: manifiesto.csv presente
- [x] Dataset: configuracion_optica.json presente
- [x] 600 registros dataset y resumen de adquisición
- [x] 4 clases x 150 imágenes
- [x] Partición 420/90/90
- [x] 100 grupos; ninguna pose compartida entre particiones
- [x] 600 IDs únicos
- [x] 600 PNG íntegros 640x360 (raster y CRC) (600/600)
- [x] SHA256 por imagen correcto y sin duplicados exactos
- [x] Hash del manifiesto coincide con resumen
- [x] Hash de configuracion optica verificado
- [x] R23: reporte_entrenamiento.json presente
- [x] R23: modelo_cnn_a2.keras presente
- [x] R23: reporte_visual.html presente
- [x] R23: matriz_confusion.csv presente
- [x] R23: historial.csv presente
- [x] R23: mosaico_control_calidad.png presente
- [x] CNN preliminar entrenada
- [x] Entrenamiento usó 420/90/90
- [x] Al menos una época completada
- [x] Modelo .keras y hashes de reportes coinciden
- [x] Matriz de confusión 4x4, 90 muestras test
- [x] Exactitud test consistente con matriz de confusión

## Reservas y pendientes

- R19: umbrales 0.60/0.80 m provisionales; contrastar el enunciado y formalizar el lanzamiento operativo.
- R20: la auditoria constata archivos, conteos del CSV y tamaños; NO reproduce los rosbags ni verifica internamente cada evento.
- R20: 4 rutas A* empleadas como escenarios; verificar equivalencia con la rubrica oficial. DIFUSO=0 en 20 ensayos.
- R21 giro_medio: A=0.46687, B=0.42296, cambio=-9.41%.
- R21 variacion: A=47.66207, B=46.79914, cambio=-1.81%.
- R21: cambios positivos y negativos documentados; no existe prueba de mejora de navegación real en Gazebo.
- R23: aciertos test=90/90, accuracy declarada=100.00%; solo dominio sintético.
- R22/R23: auditar visualmente mosaico_control_calidad.png; partición por pose no equivale a validación en otro mundo/cámara.

## Acciones de cierre

1. Revisar visualmente el mosaico de test CNN y las capturas de Gazebo.
2. Confirmar umbrales R19 y definición de escenarios R20 con rúbrica/enunciado.
3. Redactar R21 como comparación numérica con resultado mixto; no afirmar mejora de navegación.
4. Preparar una única publicación GitHub con informe y evidencias ligeras, conservando localmente dataset completo y rosbags.
5. Confirmar requisitos de entrega Blackboard y registrar acuse de entrega.

Los rosbags no fueron copiados ni reanalizados. El índice registra tamaño de sus bases de datos sin calcular SHA256 completo.
