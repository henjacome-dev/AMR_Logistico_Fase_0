# A3 · Actividad 8 — Justificación de tiempos y análisis de fallos CNN

**Proyecto:** AMR Logístico · Unidad 3 / U3S7.
**Estado:** cierre documental basado en registros preexistentes; **sin repetir Gazebo ni reentrenar la CNN**.
**Rama de publicación:** `u3s7-entrega-final-2026-11-16`; versión de entrega congelada por el tag `final`.
**Nota metodológica:** esta auditoría no sustituye las pruebas pendientes de evasión difusa de la actividad 7.

## 1. Evidencia reproducible

- `resultados/U3S7/tabla_ordenes.csv`: longitudes, tiempos, cajas y contactos por orden.
- `resultados/U3S7/secuencia/resumen.json`: eventos, alineaciones, confirmación CNN, reloj de Gazebo y modos.
- `resultados/U3S7/desvio/resumen.json`: ensayo independiente de desvío y replanteamiento de ruta.
- `resultados/cnn_u3s7_20261116/evaluacion.json`: exactitud nominal, matriz de confusión y único fallo nominal.
- `resultados/cnn_u3s7_20261116/robustez.json`: exactitudes bajo tres perturbaciones y casos seleccionados.
- `tools/evaluar_robustez_cnn.py`: definición exacta de las transformaciones visuales.
- `tools/A3_E6_R27_ejecutor.py`: cómputo de los relojes, límites y orientación al panel.
- `docs/entrega_u3s7/Informe_IEEE_U3S7.docx/.pdf`: explicación actualizada y conservada en ocho páginas.

## 2. Definición correcta de la cota temporal

La rúbrica establece por tramo: **t_ref = L_A* / v_máx**, con v_máx = 0,20 m/s, y **t_límite = 1,30 × t_ref**. Es un valor de comparación construido con la longitud A* y la velocidad máxima, no una medición independiente de viaje.

La implementación distingue dos medidas:

1. **`navegacion_sim_s`**: tiempo de navegación desde el inicio del seguimiento hasta la llegada, tomado del reloj de simulación de Gazebo (`/clock`); es el indicador sobre el que el ejecutor calcula `cumple_tiempo`.
2. **`duracion_total_orden_s` / `confirmacion_total_s`**: tiempo de reloj monotónico del sistema huésped hasta confirmar el destino con CNN, incluyendo esperas y, en determinados casos, giro de alineación. Este tiempo está sujeto también al rendimiento de la ejecución de Gazebo/ROS en WSL2.

**No deben restarse ni compararse como si procedieran del mismo reloj físico.** En particular, `confirmacion_total_s − navegacion_sim_s` **no** es una medición válida del sobrecoste puro de inferencia CNN o de giro, porque combina tiempo de pared y simulado.

## 3. Auditoría de las diez órdenes

| Orden | L_A* (m) | Cota (s) | Navegación sim. (s) | Margen sim. (s) | Total pared (s) | Alineación |
|---:|---:|---:|---:|---:|---:|:---:|
| 01 | 5.600 | 36.40 | 33.9 | +2.50 | 35.10 | No |
| 02 | 8.249 | 53.62 | 51.5 | +2.12 | 59.65 | Sí |
| 03 | 5.600 | 36.40 | 33.8 | +2.60 | 34.82 | No |
| 04 | 5.600 | 36.40 | 34.9 | +1.50 | 45.39 | Sí |
| 05 | 8.249 | 53.62 | 51.2 | +2.42 | 52.65 | No |
| 06 | 5.600 | 36.40 | 33.0 | +3.40 | 34.50 | No |
| 07 | 5.600 | 36.40 | 34.8 | +1.60 | 36.18 | No |
| 08 | 8.249 | 53.62 | 49.8 | +3.82 | 58.20 | Sí |
| 09 | 5.600 | 36.40 | 33.9 | +2.50 | 35.59 | No |
| 10 | 5.600 | 36.40 | 35.2 | +1.20 | 45.89 | Sí |

**Resultados confirmados:** 10/10 órdenes con `navegacion_sim_s <= limite_s`; 10/10 confirmaciones CNN; 0 contactos externos en el resumen (268.368 muestras de sensores de contacto). La cota se cumple con margen positivo incluso en el peor tramo. El valor `cumple_tiempo` se deriva explícitamente de la navegación en Gazebo.

**Análisis exploratorio del tiempo integral:** si se coteja el **valor numérico** de la misma cota con el tiempo de pared, 6/10 quedarían por debajo y 4/10 por encima (órdenes 2, 4, 8 y 10). Los cuatro casos registran giro de alineación frente al panel de Empaque, pero ello **no permite cuantificar cuánto del incremento se debe al giro**, pues los relojes son diferentes. Tampoco convierte automáticamente el resultado formal de 10/10 en 6/10. Es un indicador secundario que justifica medir un KPI operativo homogéneo en ensayos futuros.

**Criterio de alcance:** el registro automático de diez órdenes usó cajas de 0,20 m colocadas al costado de rutas practicables (semilla 451). El resumen conserva los estados `RUTA` y `DETENIDO` sin activaciones `DIFUSO` en esta secuencia específica. No se atribuye a ella la evasión Mamdani observada en pruebas manuales distintas.

## 4. Exactitud nominal y fallos de percepción

La matriz de confusión nominal tiene **89 aciertos de 90** imágenes (98,89%). El único fallo del conjunto reservado fue `EMPAQUE → RECEPCION` (`imagenes/test/EMPAQUE/S03_D05_A01_C06.png`) con confianza Softmax **43,74 %**, inferior al umbral de aceptación operativo de 80 %. El argumento de que el panel aparece pequeño y lateral es una **interpretación visual plausible**, no una causalidad demostrada mediante ablación.

Las tres pruebas suplementarias se aplicaron por separado a copias de las **mismas 90 imágenes**; no son tres errores adicionales observados en el test nominal. El script seleccionó para cada perturbación una imagen correctamente clasificada en normalidad que pasó a ser clasificada erróneamente tras la transformación.

| Condición | Aciertos | Exactitud | Ejemplo (real → predicha) | Confianza de la clase incorrecta |
|:---|---:|---:|:---|---:|
| Nominal | 89/90 | 98.89% | EMPAQUE → RECEPCION | 43.74% |
| Oclusion | 39/90 | 43.33% | ALMACENAMIENTO → RECEPCION | 99.62% |
| Oscuridad | 42/90 | 46.67% | ALMACENAMIENTO → DESPACHO | 96.21% |
| Desenfoque | 36/90 | 40.00% | EMPAQUE → DESPACHO | 98.48% |

**Perturbaciones aplicadas (reproducibles en `tools/evaluar_robustez_cnn.py`):**

- **Oclusión:** rectángulo opaco sobre (120,65)–(520,295) px; oculta la zona donde puede hallarse el panel.
- **Oscuridad:** `ImageEnhance.Brightness(...).enhance(0.12)`; brillo al 12 % del original.
- **Desenfoque:** `ImageFilter.GaussianBlur(20)`; pérdida intensa de bordes y detalles finos.

La elevada confianza Softmax de los errores perturbados (99,62 %, 96,21 % y 98,48 %) **no implica calibración ni confiabilidad fuera del dominio nominal**. El umbral de 0,80 por sí solo no eliminaría esos errores: ayudan la exigencia de tres observaciones concordantes y la verificación de proximidad geométrica, pero esas defensas tampoco garantizan inmunidad ante perturbaciones repetidas.

**Interpretación:** los mecanismos visuales descritos son coherentes con las alteraciones diseñadas y las predicciones observadas, pero no establecen un diagnóstico causal único. Para cuantificar robustez operativa deben probarse intensidades graduadas, iluminación variable, nuevas poses/cámaras y, posteriormente, imágenes físicas independientes.

## 5. Conclusiones defendibles ante la rúbrica

1. La métrica **formal de navegación simulada** es 10/10 bajo el 130 % de t_ref; no se afirma que las diez órdenes también cumplan un límite idéntico en reloj real hasta confirmar la estación.
2. Los tiempos completos se informan **sin ocultar** los cuatro tramos que superan numéricamente la cota, pero se identifican como comparación de naturaleza distinta.
3. La CNN superó el mínimo de **90 % nominal** con 98,89 % (89/90), pero su desempeño cayó bajo perturbaciones visuales deliberadamente severas.
4. Existe **un fallo nominal y tres casos de fallos perturbados**; son cuatro ejemplos diferenciados metodológicamente, no cuatro errores independientes del test reservado.
5. La secuencia automática no demuestra evasión `DIFUSO`; esa evidencia se gestionará por separado dentro de la actividad 7 de A3.

## 6. Reproducción y protección de datos

Desde la raíz del repositorio:

```bash
python3 tools/a3_act8_validar_evidencias.py
python3 tools/a3_act6_validar_informe.py
```

El primer script relee las fuentes de U3S7 y reproduce las dos tablas CSV derivadas sin ejecutar el robot ni modificar los datos. El segundo asegura que las tablas, figuras y referencias del informe mantienen el control de integridad anterior. Las imágenes, el JSON de evaluación y las trayectorias originales **no se reemplazan**.

**Huellas de las fuentes originales (para auditoría):**

- `resultados/U3S7/secuencia/resumen.json`: SHA-256 `7754ac02816b609e1333e179351a7e9b461d47cb217f6cc61631db77fa9421f9`
- `resultados/cnn_u3s7_20261116/evaluacion.json`: SHA-256 `d076638e12f3e9a00dcc98d791846ec6046f7b1fdbc5e1a3e8aa61f98a336aeb`
- `resultados/cnn_u3s7_20261116/robustez.json`: SHA-256 `bf8709480fabc1e3313ca7d14dcfdef0e585b004b177a16bd220493ffdfcb84c`
- `resultados/U3S7/tabla_ordenes.csv (LF)`: SHA-256 `8a76b9380cfc7773ce4fe12e6c7eeed358d1e1ef25f4a8550bf38151a3559e1d`
