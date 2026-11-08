# Avance 2 — Cierre experimental consolidado (08-11-2026)

**Proyecto:** AMR Logístico; ROS 2 Humble, Gazebo Classic, MATLAB y scikit-fuzzy.
**Rama de referencia:** `desarrollo-avance2`, base anterior a la publicación
`82ba1d98a4201b01e7eaaa42146c163808dcffec`.
**Estado:** auditoría técnica favorable **con reservas**. No equivale a entrega en Blackboard
ni a homologación de parámetros con una rúbrica no aportada a este cierre.

## Resultados por requisito

| ID | Evidencia | Resultado y alcance |
|---|---|---|
| R15–R17 | FIS v01; pruebas MATLAB/ROS y barreras de seguridad | 3 entradas, 2 salidas, 27 reglas; operación experimental probada, launch permanente pendiente |
| R18 | Equivalencia publicada | 10/10 MATLAB–Python; error menor al 2 % |
| R19 | `evidencias_A2/R19/` | Siete etapas en Gazebo; umbrales de ingeniería 0,60/0,80 m sin confirmación oficial; integración de lanzamiento permanente pendiente |
| R20 | `evidencias_A2/R20/` | 4 rutas × 5 repeticiones = 20/20, error final ≤ 0,10 m, 20 rosbags locales; ninguna activación DIFUSO |
| R21 | `evidencias_A2/R21/` | 20 pares, 1.440 inferencias; menor magnitud de giro, variación de giro lateral aumentada; **sin demostrar mejora en navegación real** |
| R22 | `evidencias_A2/R22_R23/` | 600 imágenes RGB PNG (150 × 4), 100 grupos pose, partición 420/90/90, SHA256, sin duplicados exactos; inspección visual pendiente |
| R23 | `evidencias_A2/R22_R23/entrenamiento/` | CNN ligera inicial, 8 épocas; 90/90 aciertos en test sintético (100 %); no equivale a modelo final MobileNetV2 ni prueba fuera de distribución |

## Alcance experimental y reservas

1. **R19:** los umbrales 0,60/0,80 m no están expresados numéricamente en la matriz del repositorio. El banco utilizó un supervisor experimental; no se sustituye `tools/supervisor_f13.py`.
2. **R20:** las cuatro rutas A* constituyen la definición experimental de escenarios. Confirmar con el enunciado docente si se exige diversidad de obstáculos/entornos. Los ensayos no activaron el modo difuso. Rosbags verificados por presencia, estructura y conteos; no se reprodujo cada uno.
3. **R21:** la comparación es numérica pareada bajo entradas idénticas. Los casos con obstáculos laterales empeoran la variación de ω; por tanto no se afirma una mejora global de suavidad ni evasión.
4. **R22–R23:** el dataset sintético se generó con un modelo de cámara temporal de FOV de 145°, no el modelo físico original. Se controló fuga por grupos de pose y duplicados exactos, pero la cifra de exactitud no demuestra generalización a otra cámara, estación o mundo. Revisar el mosaico de prueba antes de la entrega.
5. **Entrega:** la auditoría no representa la publicación de 20 rosbags, los PNG originales o el modelo Keras. Esos archivos requieren un medio de distribución accesible al docente.

## Archivos reproducibles y trazabilidad

- Bancos originales ejecutados: `tools/a2_bancos/`. Se conservan las versiones efectivamente usadas: R19 original, R20 v2 teleport, R21 A/B, CNN v6 contextual y auditor único.
- Resultados seleccionados: `docs/evidencias_A2/`, con índice `manifiesto_publicacion.csv`.
- Auditoría consolidada local: `resultados/A2_auditoria_integral_cierre/20261108_010641`.
- Rosbags R20 conservados en: `resultados/A2_R20_integral_4x5/20261107_224326`; no subidos a Git.
- Dataset R22/R23 original en: `resultados/A2_R22_R23_CNN_integral_v6_contexto/20261108_004923`.
- ZIP externo para entrega: `A2_dataset_600_y_CNN_entrenada.zip` (carpeta local `~/AMR_A2_entrega_local/`; **no incluido en Git**)
- **SHA256 ZIP dataset y modelo:** `9ac794dc4615254c9373f55c17969b495a581b1fb594552892c36d3beeaf7434`

## Verificaciones antes de la entrega académica

- [ ] Verificar visualmente `docs/evidencias_A2/R22_R23/entrenamiento/mosaico_control_calidad.png`.
- [ ] Confirmar umbrales R19 y definición de escenarios R20 con el PDF/rúbrica oficial.
- [ ] Comprobar que el ZIP completo esté accesible para el docente, sin confiar en referencias a rutas locales.
- [ ] Publicar **un solo commit** de esta preparación tras revisar `git diff` y GitHub Actions.
- [ ] Confirmar entregables Blackboard y registrar el envío.

**No se ha realizado un commit, push, tag ni entrega automática por ejecutar el script de preparación.**
