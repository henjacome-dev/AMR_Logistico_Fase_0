# MEC451 · Autómatas y Expertos — Registro de cierre A13 y decisión de configuración

**Fecha:** 9 de noviembre de 2026  
**Proyecto:** Robot móvil autónomo para logística en almacén · TurtleBot3 Waffle Pi · ROS 2 Humble/Gazebo Classic  
**Actividad:** A13 — Comparación experimental pareada de controladores difusos  
**Estado:** **A13 completada — comparación descriptiva 5 pares / 10 ensayos**  
**Decisión de configuración:** **FIS v03 permanece como la única versión oficial para todas las actividades pendientes y el Avance 3.**

## 1. Identidad de los controladores

- **FIS A — v03 (oficial y de referencia):** `matlab/A2_MAMDANI/A2_v03_rubrica/AMR_Mamdani_A2_v03_rubrica.fis` dentro del repositorio `AMR_Logistico_Fase_0`.
- SHA256 conocido y previamente validado de v03: `67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903`.
- **FIS B — v04 (solo experimental):** `AMR_Mamdani_A2_v04_A13.fis`, utilizado para el ensayo A/B. No se adopta como controlador del proyecto.
- **Cambio evaluado:** única función de pertenencia de salida `v/Lenta` de tipo `trapmf`: v03 `[0, 0, 0.03, 0.07]` frente a v04 `[0, 0, 0.025, 0.055]`. Se preservaron las 27 reglas, las entradas `df`, `di`, `dd`, ambas salidas `v`, `omega` y el universo del giro.
- **Modalidad:** escenario B3, navegación **híbrida asistida**, no Mamdani puro.

## 2. Evidencia experimental A13

**Dictamen de la ejecución corregida:** `A13_AB_10_ENSAYOS_PAREADOS_COMPLETOS_DESCRIPTIVA`.

| Métrica | v03 (A) | v04 (B) | Lectura |
|---|---:|---:|---|
| Ensayos ejecutados y acreditados | 5/5 | 5/5 | Igual cumplimiento |
| Pares comparables | — | — | 5/5 |
| Contactos robot–obstáculo | 0 | 0 | Ambas variantes con cobertura válida de contactos |
| Ciclos de influencia real sobre velocidad | 925 | 987 | Modificación con participación efectiva; no equivale por sí sola a mejora |
| Error final medio de llegada (m), calculado de los diez valores mostrados por consola | 0.14568 | 0.14448 | B − A = **−0.00120 m** (1,2 mm menos) |
| Tiempo aproximado de pared por recorrido (s) | Referencia | Referencia + 1.239 | B − A = **+1.239 s**; no equivale necesariamente a tiempo de simulación |
| Velocidad media positiva de asistencia aplicada (m/s) | Referencia | Referencia − 0.0017911 | B − A = **−0.0017911 m/s** |
| Velocidad media propuesta de asistencia (m/s) | Referencia | Referencia − 0.0023667 | B − A = **−0.0023667 m/s** |
| Despeje geométrico medio (m) | Referencia | Referencia − 0.0001 | Diferencia pequeña, no mejora acreditada |

**Interpretación:** v04 alcanzó la meta en todos los ensayos, sin contactos físicos, y redujo ligeramente el error final medio (1,2 mm). A la vez, aumentó el tiempo aproximado de pared y redujo la velocidad media de asistencia. **No se demuestra superioridad global de v04 ni significación estadística con n=5 pares.** La evaluación cumple el objetivo de comparar resultados, no de forzar una mejora.

**Carpeta de evidencia real en Ubuntu:**

`/home/enry/AMR_A2_entrega_local/A13_B3_FIS_AB_5_pares_20261109_173556_619177_UTC/`

Se conserva separadamente la serie inicial con fallos de acreditación provocados por un defecto de ruta externa del archivo FIS; **no se reinterpretan ni se sobrescriben** esos resultados. La serie corregida es la que sustenta el dictamen final.

## 3. Decisión explícita e invariable del proyecto

1. **Conservar y utilizar FIS v03** en A14–A20 y, expresamente, en **todo el Avance 3**: simulación, integración ROS, navegación, algoritmos, entrenamiento/evaluación, pruebas y entrega final.
2. **Mantener v04 como artefacto experimental A13**, archivado y referenciado únicamente en el análisis comparativo. No copiarlo encima de v03, no cambiar referencias de lanzamiento a v04 ni alterar la configuración operativa.
3. Una hipotética mejora posterior de v04 **se documentaría como hallazgo**, pero **no autoriza su adopción automática**. Cualquier cambio del FIS oficial exigiría una decisión distinta y explícita del responsable del proyecto.
4. **No modificar** el tag histórico `avance2`, el FIS v01, el FIS v03 ni la evidencia original de A12/A13. Las mejoras documentales posteriores deben mantenerse separadas y ser trazables.
5. Toda entrega, README, presentación o informe debe distinguir: **v03 = configuración oficial**; **v04 = comparación experimental**, sin alegar control Mamdani puro para el escenario híbrido B3.

## 4. Estado del plan

- **A01–A13:** completadas en el sentido de sus hitos de validación; A13 cierra con comparación descriptiva y conclusión sin superioridad global acreditada de v04.
- **A14–A20:** pendientes (7 actividades): revisión/reproducción de rosbags; CNN y datos; Git/CI y empaquetado; demostración; informe; auditoría final.
- **Calificación:** pendiente de una nueva auditoría formal A20; este registro no modifica por sí solo la nota.

> **Nota de trazabilidad:** este documento se elaboró a partir del registro de consola compartido por el usuario. Su archivo descargable no modifica automáticamente el repositorio Ubuntu ni las referencias de los scripts. Para incorporarlo al proyecto, copiarlo como archivo nuevo de documentación sin reemplazar recursos existentes.
