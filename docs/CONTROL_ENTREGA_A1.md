# Control de evidencias — Avance 1 (A1)

**Proyecto:** AMR logístico, MEC451, Grupo 1.
**Corte histórico evaluado:** entrega de Blackboard de 4 de noviembre de 2026, 20:37 UTC−5.
**Estado documental:** entrega acreditada por captura de la actividad y correo de recepción. La calificación docente no se ha comprobado.

## Archivos realmente entregados

- Informe: `Avance1_AMR_Grupo_1.docx.pdf` — dos páginas, recibido e inspeccionado.
- Código: `AMR_Logistico_Fase_1-main.zip` — ZIP comprobado sin error CRC; 93 entradas (incluye 23 directorios/entradas vacías); carpeta raíz interna `AMR_Logistico_Fase_0-main/`.
- El acuse de Blackboard confirma presentación, pero las capturas y el correo deben permanecer **fuera del repositorio público**.
- La etiqueta Git `avance1` existe; fue creada el 5 de noviembre de 2026, posteriormente a la presentación del 4 de noviembre. No debe sustituirse retroactivamente el ZIP entregado por el estado de dicho tag.

## Matriz de comprobación de requisitos A1

| ID | Requisito | Evidencia del ZIP entregado | Veredicto |
|---|---|---|---|
| R03 | Mundo >=4 × 4 m | `modelos/almacen_f1.world`, configuración 10 × 8 m | Cumple en archivo |
| R04 | >=3 pasillos, ancho >=1 m | Dos estantes horizontales con tres corredores nominales libres; ancho aproximado 2,1–2,2 m según geometría | Cumple geométricamente, falta medición dinámica |
| R05 | 4 estaciones con marcadores diferenciados | Cuatro cilindros de 0,3 m de radio y 0,01 m de altura con colores distintos, **no paneles verticales** | Parcial |
| R06 | 3 sensores emulados como `sensor_msgs/Range` | Código del emulador no incluido; `/scan` LiDAR nativo sí aparece en diagnóstico F0 | No acreditado |
| R07 | Sectorización 15° a 0° y ±45° | Parámetros especificados en README, pero no código ni registros de emulador | No acreditado |
| R08 | Alcance 2 m y ruido gaussiano sigma 0,01 m | Especificación declarada, sin implementación comprobable | No acreditado |
| R09 | Odometría y cámara disponibles | `resultados/F0_ros_topics.txt` y `resultados/F0_gazebo_waffle_pi.png` | Evidencia archivada |
| R10 | Mapa manual sin SLAM | `mapas/almacen_f1.pgm`, YAML, scripts de generación y MATLAB | Cumple |
| R11 | Resolución <=0,05 m e inflado 0,21 m | Mapas y scripts; `config/escenario_f1.yaml` | Cumple en configuración |
| R12 | Tres órdenes A* distintas | CSV de órdenes A, B, C en Python y MATLAB; tablas de tiempos y longitudes | Cumple documentalmente |
| R13 | Seguidor completa al menos una orden sin obstáculos | Código `tools/seguidor_ruta_f13.py` y tres logs F13 con aproximaciones a metas. No hay registro explícito de una orden logística completa (con todos sus tramos) | Parcial, requiere calificar alcance de «orden» |
| R14 | Reporte 2 páginas, repositorio y tag | Informe de 2 páginas y ZIP en Blackboard; tag `avance1` existe pero es posterior | Parcial temporal |

## Datos A* presentes en el ZIP original

| Orden | Secuencia | Python (m) | MATLAB (m) |
|---|---|---:|---:|
| A | Recepción–Almacenamiento–Empaque–Despacho | 20,449 | 20,38995 |
| B | Recepción–Empaque–Almacenamiento–Despacho | 32,817 | 32,64092 |
| C | Despacho–Empaque–Almacenamiento–Recepción | 26,449 | 26,38995 |

Fuente interna: `resultados/F12_comparacion_ordenes.csv` y `matlab/F124_MATLAB/resultados/F124_comparacion_final.csv`.

## Divergencias entre el informe y el ZIP

- El informe describe un esquema preliminar de **6 × 6 m**, mientras que el ZIP contiene un mundo **10 × 8 m**.
- El informe muestra tres rutas de referencia ejecutadas en **Python** y declara MATLAB pendiente; el ZIP incorpora scripts y CSV de resultados **MATLAB** para el mapa ampliado.
- El informe señala como pendientes mundo, sensores y seguimiento; el ZIP sí contiene el mundo y tres registros F13, pero no el código del emulador de tres `Range`.
- Los registros F13 son de tres **tramos individuales** (Robot–Recepción, Recepción–Almacenamiento y Almacenamiento–Empaque); no permiten concluir por sí solos la ejecución de la orden A completa hasta Despacho.

## Cierre y próximos pasos

1. **No reemplazar retrospectivamente** los archivos históricos entregados. Conservar el PDF y ZIP originales como referencia de corte académico.
2. **No publicar** capturas privadas, acuses con identificadores ni correo de Blackboard en GitHub.
3. Verificar, si el docente lo requiere, la calificación y la evaluación de la demostración de A1 en la hoja de rúbrica.
4. Conservar en ramas posteriores las mejoras técnicas realmente ejecutadas después del 4 de noviembre (marcadores, emulador, validaciones y seguimiento completo).
5. Comparar el ZIP histórico con Git solo para trazabilidad; no alterar `main` ni tags anteriores.

**Dictamen:** la entrega formal de A1 está acreditada. Su cumplimiento técnico histórico es **parcial**, aunque el ZIP demuestra más avance que el informe de dos páginas por sí solo. No asignar una nota oficial sin la rúbrica calificada del docente.
