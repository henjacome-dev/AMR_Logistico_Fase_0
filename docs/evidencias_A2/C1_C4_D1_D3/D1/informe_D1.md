# D1 — Auditoría integral de entrega | AMR logístico

**Dictamen:** `D1_EVIDENCIA_VERIFICADA_CON_RESERVAS`

## Batería oficial de escenarios

| Escenario | Repeticiones acreditadas sobre 5 | Observación |
|---|---:|---|
| B1 | 5/5 | Pasillo 1 m; excluir intento fallido sin evidencia |
| B2 | 5/5 | A* y desvío estático preplanificado |
| B3 | 5/5 | Evasión híbrida con subobjetivos, NO Mamdani aislado |
| B4 | 5/5 | Centrado geométrico asistido, A3/A4/FIS en sombra |
**Total acreditado automáticamente: 20/20.**

## Rediseño

C1–C4: C1=localizado, C2=localizado, C3=localizado, C4=localizado.

El antes/después B3 es descriptivo (un fallo original y cinco llegadas híbridas),
no es comparación A/B pareada, y no demuestra mejora aislada de Mamdani.

## Reservas metodológicas

- Rosbags: se verificó presencia de SQLite `.db3`, NO su reproducción.
- Contactos físicos Gazebo: no medidos directamente.
- El ZIP CNN requiere distribución accesible para el docente y revisión visual del mosaico.
- `git status` local no confirma publicación remota ni creación del tag remoto.

## Bloqueos y faltantes

- Ningún bloqueo automático.

## Revisiones y entregables pendientes

- B1-B4/contactos: Los ensayos reportan despeje geométrico; no hay auditoría directa de contactos
- CNN/revision_visual: Abrir mosaico_control_calidad.png; no se acredita inspección visual automáticamente
- CNN/distribucion: ZIP local no implica que el docente pueda descargarlo; adjuntar o publicar enlace verificable
- GIT/estado_local: Cambios locales detectados: 90 entradas; revisar antes de commit
- GIT/tag_local: No existe tag local avance2
- GIT/scripts_de_prueba: Comprobar que ejecutores B2–B4 y scripts C1–C4 usados realmente estén añadidos al repositorio, no solo en Descargas
- GIT/readme: Reconciliar descripciones antiguas de CNN pendiente y resultados B1–B4 con la evidencia actual
- GIT/push_y_tag_remoto: No se hacen conexiones ni pushes: D5/D6 requieren verificación posterior en GitHub
- ENTREGA/informe_2_paginas: D4: verificar que exista y tenga exactamente dos páginas y se corresponda con las evidencias
- ENTREGA/blackboard: D7: pendiente de cargar archivos y comprobar acuse Blackboard

## Próximos pasos

1. D2: abrir mosaico CNN, comprobar paquete completo y acceso externo.
2. D3: integrar B1–B4 y dictámenes C1–C4 al cierre.
3. D4: preparar y validar informe académico de dos páginas.
4. D5: hacer commit/push controlado y verificar GitHub Actions.
5. D6: crear `avance2` tras verificar el commit y confirmar tag remoto.
6. D7: preparar Blackboard y conservar acuse de entrega.

Ver detalles por ensayo y evidencia en `matriz_D1.csv`; fuentes y SHA en `manifiesto_SHA256.csv`.
