# A1 · Actividad 5 — Trazabilidad de la entrega y del tag `avance1`

**Proyecto:** AMR Logístico, MEC451 — Grupo 1
**Objeto:** comparar el archivo efectivamente enviado a Blackboard el 4 de noviembre de 2026 con el tag `avance1` de GitHub.
**Rama donde se archiva esta auditoría:** `u3s7-entrega-final-2026-11-16`.
**Estado:** verificado a partir de los hashes GitHub y del contenido del ZIP original; se adjunta script para reproducción en Ubuntu.

## 1. Archivos y fechas históricas

- **Blackboard:** entrega de *Proyecto Avance 1* registrada el **4 de noviembre de 2026, 20:37 (UTC−5)**. La actividad muestra `Avance1_AMR_Grupo_1.docx.pdf` y `AMR_Logistico_Fase_1-main.zip` como adjuntos. La recepción está corroborada por un correo automático de Blackboard; los comprobantes se conservan de forma privada.
- **ZIP original:** `AMR_Logistico_Fase_1-main.zip`; 1.006.777 bytes comprimidos; **93 entradas (76 archivos y 17 directorios)**; 1.613.423 bytes de contenido; prueba CRC sin errores.
- **SHA-256 del ZIP:** `398835963816b9cf429ab68283b27340d6481fffa5006b115b6cad4cfc21502b`.
- **Commit original identificado:** [`d1558d7e8845acf453a3b6eee5a1c62af947b3cf`](https://github.com/henjacome-dev/AMR_Logistico_Fase_0/commit/d1558d7e8845acf453a3b6eee5a1c62af947b3cf), creado **4 de noviembre de 2026, 16:14:43 (UTC−5)**. Mensaje: `F1.3: validar seguimiento ROS2 de tres rutas con supervisor de seguridad`.
- **Tag anotado `avance1`:** apunta a [`db4e4b4f214bfe7cbe1e19192879a1890f1bcb02`](https://github.com/henjacome-dev/AMR_Logistico_Fase_0/commit/db4e4b4f214bfe7cbe1e19192879a1890f1bcb02), del **5 de noviembre de 2026, 16:25:48 (UTC−5)**; creación del tag a las **16:25:59**. Mensaje del commit: `F1: cerrar documentacion y README del Avance 1`.

## 2. Resultado de comparación de contenidos

Para cada archivo del ZIP se calculó el SHA-1 de objeto Git (`SHA1('blob ' + tamaño + '\0' + contenido)`). Se cotejó con las entradas `blob` obtenidas del árbol GitHub del tag. Se identificó además un commit anterior con los **76 archivos exactamente idénticos** en rutas y contenido al ZIP original.

| Comparación | ZIP Blackboard | Commit `d1558d7` | Tag `avance1` |
|---|---:|---:|---:|
| Número de archivos | 76 | 76 | 156 |
| Archivos exactos respecto del ZIP | 76 | **76** | **66** |
| Archivos con mismo nombre, pero contenido distinto | — | **0** | **10** |
| Archivos exclusivos de la referencia Git | — | **0** | **80** |
| Archivos del ZIP ausentes en la referencia Git | — | **0** | **0** |

**Resultado:** el ZIP presentado el 4/11 corresponde **exactamente** al contenido del commit `d1558d7`. El tag `avance1` preserva los 76 nombres, pero añade desarrollos posteriores y no representa una copia inalterada de la entrega Blackboard.

## 3. Diez archivos modificados respecto de la entrega original

| Área | Ruta relativa | Criterio de clasificación |
|---|---|---|
| Documentación | `README.md` | Actualización de estado del repositorio |
| Documentación | `docs/F0_arquitectura.md` | Revisión de especificación de arquitectura |
| Documentación | `docs/F0_requisitos.md` | Revisión de requisitos |
| Documentación | `docs/F0_validacion.md` | Revisión de plan o evidencias de validación |
| Mundo Gazebo | `modelos/almacen_f1.world` | Cambio de contenido del escenario |
| Resultados | `resultados/F13_ruta_2_log.csv` | Contenido de registro diferente |
| Resultados | `resultados/F13_ruta_3_log.csv` | Contenido de registro diferente |
| Herramientas | `tools/generar_mundo_f1.py` | Cambio en generador del escenario |
| Control de navegación | `tools/seguidor_ruta_f13.py` | Cambio en código de seguimiento |
| Supervisión | `tools/supervisor_f13.py` | Cambio en código del supervisor |

**Interpretación:** la comparación de hashes demuestra que estos archivos cambiaron, pero por sí sola no demuestra mejora o regresión del comportamiento. Los cambios posteriores no deben atribuírsele al ZIP entregado.

## 4. Archivos añadidos posteriormente en `avance1`

| Carpeta | Archivos nuevos | Contenido destacado |
|---|---:|---|
| `docs/` | 6 | Evidencias de emulador, mapa, navegación integrada, seguridad y comprobaciones |
| `launch/` | 1 | Lanzamiento operativo del almacén |
| `pruebas/` | 1 | Obstáculo SDF de pruebas |
| `resultados/` | 67 | Baterías de seguimiento, odometría, supervisión y ensayos de seguridad |
| `tools/` | 5 | Emulador de sensores y herramientas de repetibilidad, seguridad y recuperación |
| **Total** | **80** | |

El generador posterior `tools/generar_mundo_f1.py` incluye paneles verticales para las estaciones. El tag incorpora también `tools/emulador_sensores_f1.py`. Esos componentes no constan en los 76 archivos del ZIP original.

## 5. Validez y límites de esta auditoría

- **Verificado:** estructura ZIP, integridad CRC, hash SHA-256 del ZIP, SHA-1 Git por archivo y correspondencia con árboles GitHub del commit original y del tag.
- **No verificado por este procedimiento:** funcionamiento de ROS 2/Gazebo, ejecución real de A*, notas del docente, permisos de archivos o metadatos Git dentro del ZIP (el ZIP no incluye `.git`).
- **Criterio histórico:** el commit `d1558d7` es la **referencia exacta del código entregado**; el tag `avance1` es una **consolidación posterior**. Ambos deben preservarse; no mover ni sobrescribir el tag existente.

## 6. Reproducción en Ubuntu WSL

Con `AMR_Logistico_Fase_1-main.zip` descargado desde Blackboard en `C:\Users\Henry\Downloads`, entrar en la raíz del repositorio:

```bash
amr iniciar
git fetch origin --tags
python3 tools/a1_act5_verificar_zip_tag.py \
  /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip \
  --export-csv docs/A1_ACT5_COMPARACION_ARCHIVOS.csv
```

La salida esperada es `AUDITORÍA APROBADA`, con 76/76 archivos idénticos en el commit original y la partición **66 idénticos + 10 modificados + 80 solo tag**. El CSV generado enumera **156 rutas** e incorpora los hashes exactos de ambas referencias.

Al finalizar se publicará únicamente este informe, el verificador y el CSV de hashes, con `amr guardar`. **No subir** capturas o correos privados de Blackboard y **no modificar** `main`, `avance1`, `final` ni `d1558d7`.

## 7. Resolución de la Actividad 5

**Concluida en contenido y trazabilidad.** La publicación del acta y la ejecución del verificador en la copia Git de Ubuntu quedan como paso operativo para registrar el cierre. No es necesario volver a ejecutar simulaciones ni alterar la configuración del robot.
