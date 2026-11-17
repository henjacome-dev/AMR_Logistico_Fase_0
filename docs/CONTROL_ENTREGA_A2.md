# Control de evidencias y entrega — Avance 2 (U2S7)

**Proyecto:** AMR Logístico MEC451. **Fecha oficial A2:** 9 de noviembre de 2026.
**Objeto:** comprobación de reporte técnico (2 páginas), repositorio y demostración, separando resultados técnicos de constancias académicas.

## Auditoría del repositorio (corte ZIP U3S7)

| Elemento | Archivo del repositorio | Estado encontrado |
|---|---|---|
| Diseño Mamdani oficial v03 | `matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis` | Disponible |
| 27 reglas | `matlab/A2_MAMDANI/A2_F1_tabla_27_reglas.csv` | Disponible |
| Funciones y superficies | `matlab/A2_MAMDANI/evidencias/` | Disponibles |
| Equivalencia MATLAB–Python (10 entradas) | `resultados/A2_R18_equivalencia/evidencia_publicada/A2_R18_comparacion_10_casos.csv` | Disponible |
| Escenarios B1–B4 (5 repeticiones por escenario) | `docs/evidencias_A2/ESCENARIOS_B1_B4/` y `docs/evidencias_A2/A17_cierre_20261109/tabla_B{1,2,3,4}_fuente.csv` | Disponibles con reservas metodológicas |
| Rediseño, comparación y limitaciones | `docs/evidencias_A2/C1_C4_D1_D3/D3/informe_D3.md` | Disponible |
| Adelanto neuronal | `docs/evidencias_A2/R22_R23/` | Disponible, modelo preliminar histórico |
| Tag histórico | Acta A17: `avance2` → `dbbac54debea339d9ff9cd14985b66f9ada44c16` | Declarado en acta; verificar Git remoto |
| Reporte A2 final de dos páginas | No hay PDF/DOCX A2 en el ZIP U3S7 | No verificado |
| Grabación o registro de demostración A2 del 9/11 | Único MP4 archivado: `docs/entrega_u3s7/Demostracion_U3S7.mp4` | No verificado para A2 |
| Acuse de Blackboard | No presente en el ZIP | No verificado |

## Comprobación del reporte A2

- [ ] Existe un PDF o DOCX **efectivamente presentado para A2**; conservar original.
- [ ] Tiene exactamente **dos páginas** (comprobar con `pdfinfo archivo.pdf`).
- [ ] Incluye **objetivo** del avance, **diseño/implementación** con diagramas, **resultados** en tablas o gráficas, **problemas y soluciones** y **próximos pasos**.
- [ ] Incluye las **superficies de control Mamdani**, 27 reglas o su cobertura y la **concordancia MATLAB–Python de 10 casos (<2 %)**.
- [ ] Presenta los cuatro escenarios B1–B4 y cinco repeticiones, con límites y reservas reales; evita llamar «evasión Mamdani pura» a ensayos históricos asistidos.
- [ ] El nombre y hash del reporte coinciden con el archivo efectivamente adjuntado en Blackboard.

## Comprobación de demostración

- [ ] Video completo original de A2, o acta/captura de una demostración registrada, identificable con fecha y grupo.
- [ ] Se observan ROS 2 y Gazebo con robot, caja/obstáculo, movimiento y resultado.
- [ ] Se muestra, cuando corresponda, activación de la **salida Mamdani** sobre `/cmd_vel` o trazas equivalentes.
- [ ] Se distingue la demostración del 9/11 de pruebas posteriores U3S7, que solo sirven como evidencia suplementaria.

## Comprobación formal en Blackboard (mantener privada)

- [ ] En la actividad **Avance 2 / U2S7**, se aprecia estado «Entregado/Submitted».
- [ ] Figura **fecha/hora de entrega**.
- [ ] Figuran el **nombre del archivo entregado y su versión**.
- [ ] Existe acuse/número de confirmación, historial de envíos o vista equivalente de Blackboard; opcionalmente calificación/retroalimentación.
- [ ] Capturas y comprobantes académicos guardados **fuera del repositorio público**, por ejemplo `~/AMR_comprobantes_privados/A2/`.

## Resultado de auditoría, por ahora

**Código y resultados experimentales:** documentados ampliamente.
**Entrega formal del reporte:** sin comprobación en este ZIP.
**Demostración A2:** sin comprobante específico en este ZIP.
**Presentación en Blackboard:** no comprobable sin acceso al acuse de la plataforma.

Si el informe original no se encuentra, se puede preparar una **reconstrucción documental posterior**, pero debe etiquetarse como tal; no afirmar retrospectivamente que fue el archivo entregado el 9/11.
