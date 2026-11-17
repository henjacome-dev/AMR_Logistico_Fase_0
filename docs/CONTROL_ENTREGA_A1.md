# Control de evidencias y entrega — Avance 1 (A1)

**Proyecto:** AMR logístico · Autómatas y Expertos (MEC451).
**Fecha de entrega académica:** 4 de noviembre de 2026.
**Objeto:** separar la constancia de presentación del cumplimiento técnico documentado al cierre de A1.

## 1. Acreditación administrativa

| Requisito | Evidencia revisada | Estado |
|---|---|---|
| Actividad A1 enviada | Acuse automatizado de Blackboard «Proyecto Avance 1» | Confirmado |
| Fecha/hora | 2026-11-04, 20:37 (Ecuador, UTC−5), según correo | Confirmado |
| Informe de 2 páginas | `Avance1_AMR_Grupo_1.docx.pdf`, facilitado para esta revisión | Confirmado: PDF de 2 páginas A4 |
| Original idéntico al adjunto de Blackboard | Falta captura con nombres de adjuntos o descarga desde el intento de A1 | No comprobado |
| Repositorio o archivo del intento | Acuse solo muestra tamaño total del envío, no nombres de adjuntos | No comprobado desde el acuse |
| Calificación del docente | No aportada | Pendiente |

**Privacidad:** conservar capturas de Blackboard y correo, junto con identificadores de confirmación, fuera del repositorio público.

## 2. Informe A1: contenido efectivamente presentado

| Tarea de enunciado | Contenido del PDF | Diagnóstico documental |
|---|---|---|
| 1.1 Análisis técnico | Sensores, odometría, cámara, límites de velocidad, geometría, incertidumbres | Descrito |
| 1.2 Comparación de IA | Tabla simbólica / difusa / conexionista y fundamento de enfoque híbrido | Descrito |
| 1.3 Almacén Gazebo | Plano **propuesto** de 6 × 6 m, cuatro estaciones y pasillos; la figura señala que no es una captura de Gazebo | No demostrado por el PDF |
| 1.4 Emulador de tres sensores | Interfaces y especificaciones; sección 6 indica ejecutar el nodo y verificar tópicos todavía pendiente | No demostrado por el PDF |
| 1.5 A* con MATLAB | Tres rutas y CSV de un algoritmo de **referencia en Python**; nota explícita aclara que no son medidas en MATLAB | No acreditado por el PDF |
| 1.6 Seguimiento de ruta | Diagrama de ROS 2; sección 6 indica pendiente ejecutar una orden completa | No demostrado por el PDF |
| 1.7 Arquitectura y plan de datos | Diagrama de nodos, captura prevista de 150 imágenes por estación | Descrito |
| Formato de informe | Objetivo, figuras, tablas, pendientes y próximos pasos | Cumple extensión y estructura |

**Observación:** la evaluación técnica del avance presentado no debe sustentarse únicamente en desarrollos posteriores. El informe es explícito acerca de las validaciones aún pendientes al momento de redactarse.

## 3. Versionado Git y evidencia posterior

- Se constató la existencia del tag anotado `avance1` en `henjacome-dev/AMR_Logistico_Fase_0`.
- El tag fue creado **2026-11-05 21:25:59 UTC** (16:25:59 en Ecuador), **después** de la entrega Blackboard del 4/11 a las 20:37.
- El árbol etiquetado incluye un mundo `.world`, herramientas MATLAB y documentos de validación de Gazebo, sensores y seguimiento.
- Esas evidencias posteriores demuestran el progreso del proyecto, pero no acreditan automáticamente que esas mismas versiones se adjuntaran al intento Blackboard del 4/11.
- Para verificar el corte exacto, cotejar el archivo ZIP adjuntado en Blackboard, su contenido y, cuando sea posible, hashes/fecha de creación con el historial de Git.

## 4. Próximas comprobaciones

- [ ] Abrir **Blackboard > Proyecto Avance 1 > detalles del intento** y registrar únicamente los **nombres** de los archivos adjuntos.
- [ ] Descargar el reporte **desde Blackboard** y comparar tamaño y SHA-256 con `Avance1_AMR_Grupo_1.docx.pdf`, evitando asumir identidad por el título.
- [ ] Si se adjuntó un ZIP, inspeccionar su árbol y comparar el código de ese corte contra los requisitos 1.3–1.6.
- [ ] Recuperar la calificación/rúbrica aplicada por el docente, si está disponible.
- [ ] Conservar el correo y captura original en una carpeta privada local; no subir al repositorio público.

## 5. Dictamen provisional

**Entrega administrativa de A1: confirmada.**
**Informe de dos páginas: disponible y conforme en extensión.**
**Demostración de las tareas 1.3 a 1.6 al momento de entrega: no acreditada por el PDF proporcionado.**
**Nota A1: no estimable de forma suficientemente rigurosa sin cotejar el código realmente enviado o la rúbrica calificada.**
