# A3 · Actividad 6 — Auditoría y corrección bibliográfica IEEE

**Proyecto:** AMR Logístico · Autómatas y Expertos (MEC451)
**Entrega evaluada:** U3S7, 16 de noviembre de 2026
**Rama de trabajo:** `u3s7-entrega-final-2026-11-16`
**Referencia histórica inalterada:** tag `final` → `e38b177e373e3a68c2e77d478231b3e97f658f4b`

## Objetivo y fuentes

Corregir referencias y citas del informe final, incorporar **dos referencias presentes en la bibliografía básica del Programa de Asignatura** y conservar exactamente ocho páginas, sin modificar los resultados experimentales.

Fuentes docentes contrastadas:

- `Programa de Asignatura Automatas Expertos.pdf`, apartado 8, Bibliografía básica.
- `Plan de clases Automatas Expertos.pdf`, apartado 4, Bibliografía básica.

Se incorporan las obras **Russell y Norvig (2021)** y **Goodfellow, Bengio y Courville (2018)** con los datos bibliográficos consignados en el programa de la asignatura. No se les atribuyen resultados de las simulaciones del proyecto; se citan como sustento conceptual.

## Cambios efectuados

1. Se conservan las ocho fuentes originales y se añaden dos libros del programa oficial: **10 referencias en total**.
2. Se renumeran las referencias IEEE según su **primera aparición** y se vinculan las diez con citas explícitas en el texto.
3. Se incorporan citas contextuales en la introducción, fundamentos de A*, CNN y transferencia, configuración Keras, EarlyStopping e integración ROS 2/TurtleBot3.
4. No se cambian valores de entrenamiento, métricas, resultados, interpretaciones de fallos, tablas ni figuras.
5. Se ajusta exclusivamente el **ancho de las ocho tablas** del DOCX para que LibreOffice las exporte sin recortes. Sus títulos, filas, celdas y valores son idénticos a los del documento original.
6. Se genera un PDF nuevo a partir del DOCX corregido y se revisan visualmente las ocho páginas.

## Matriz de referencias y sustento

| N.º | Referencia | Uso en el informe |
|---|---|---|
| [1] | Universidad ECOTEC, enunciado del Proyecto 1 | Objetivo y marco del proyecto |
| [2] | Hart, Nilsson y Raphael (1968) | Algoritmo A* |
| [3] | Russell y Norvig (2021) **(nueva, programa oficial)** | Fundamentos de búsqueda y planificación |
| [4] | Goodfellow, Bengio y Courville (2018) **(nueva, programa oficial)** | Redes neuronales convolucionales |
| [5] | Sandler et al. (2018) | Arquitectura MobileNetV2 |
| [6] | Deng et al. (2009) | ImageNet y transferencia de aprendizaje |
| [7] | Keras, MobileNet models | Arquitectura y preprocesamiento del modelo |
| [8] | Keras, EarlyStopping | Detención temprana |
| [9] | ROBOTIS, TurtleBot3 Simulation | Plataforma robótica simulada |
| [10] | Open Robotics, ROS 2 Humble | Infraestructura de integración de nodos |

El orden de primera citación es [1] → [2] → [3] → [4] → [5] → [6] → [7] → [8] → [9] → [10]. No quedan referencias numeradas sin citación en el cuerpo.

## Verificaciones de integridad

| Control | Resultado |
|---|---|
| Documento original | 8 páginas |
| PDF corregido | **8 páginas**, carta, dos columnas |
| Referencias | **10/10** citadas y ordenadas |
| Tablas | **8/8** con contenido idéntico |
| Recursos gráficos | **10/10** con SHA-256 idéntico |
| Párrafos modificados fuera de bibliografía y citas | **0** |
| Tablas recortadas en el PDF exportado | **0** observadas en revisión visual |
| Resultados experimentales | **Sin modificaciones** |

SHA-256 de los entregables corregidos:

- `docs/entrega_u3s7/Informe_IEEE_U3S7.docx`: `dcfd5388d7cc814ec83f5a68909b9127c6f2b6e1191fa9d58a2c5df66bc03b0d`
- `docs/entrega_u3s7/Informe_IEEE_U3S7.pdf`: `ec81a0603ae591028aa79b0df73d1cf2d950d97f38a34b5538c2046862428e85`

**Nota de fidelidad visual:** el PDF actualizado se exportó con LibreOffice. La composición puede diferir ligeramente de la exportación original de Microsoft Word, pero se conservaron las ocho páginas y el contenido íntegro. La normalización de anchura de tablas corrige diferencias de interpretación entre ambos programas.

## Reproducción y publicación

Desde la raíz del repositorio, tras aplicar el paquete de esta actividad:

```bash
python3 tools/a3_act6_validar_informe.py
```

El verificador compara el DOCX con la versión congelada en el tag `final`, revisa los ocho bloques de tabla, los diez recursos multimedia, la numeración y primera aparición de las referencias y, cuando `pdfinfo` está disponible, el número de páginas del PDF.

Después de un resultado `[RESULTADO] AUDITORIA A3-ACT6 APROBADA`, publicar **solo en la rama de trabajo**:

```bash
amr guardar "docs(a3): corregir bibliografia IEEE y validar informe de ocho paginas"
```

Se preserva el tag histórico `final` y no se reescriben las entregas anteriores.

**Estado:** corrección y auditoría preparadas, validación local aprobada. Pendiente: importación a WSL y publicación con `amr guardar`.
