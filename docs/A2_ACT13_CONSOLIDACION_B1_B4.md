# A2 · Actividad 13 — Consolidación reproducible B1–B4

**Propósito:** documentar las cinco repeticiones de cada escenario de la rúbrica U2S7, con trazabilidad y sin editar código ni resultados experimentales. **Fuentes:** corte A12/A17 e historiales independientes B1_3/B2/B3/B4. **No se reejecutó Gazebo ni ROS 2.**

## 1. Criterio normativo
La rúbrica pide para B1 (pasillo libre), B2 (obstáculo fijo), B3 (obstáculo aparecido a 1 m) y B4 (pasillo de 0,6 m) cinco repeticiones con: colisiones, distancia mínima al obstáculo, tiempo de llegada, desviación máxima de A* y oscilaciones de ω. La evaluación de esos indicadores depende de conservar la identidad de cada ejecución. Esta actividad **no fusiona resultados de corridas distintas**.

## 2. Matriz A12/A17: veinte ensayos acreditados

| Escenario | Llegadas | Contactos registrados | Error medio (m) | Recorrido medio (m) | Despeje SDF mínimo (m) | Desv. máxima (m) | Modo demostrado |
|---|---:|---:|---:|---:|---:|---:|---|
| B1 | 5/5 | 0 | 0.0746 | 0.9075 | N/D | N/D | RUTA A* (sin evidencia de mando DIFUSO en A17) |
| B2 | 5/5 | 0 | 0.1455 | 2.2140 | N/D | N/D | A* y desvío geométrico (DIFUSO no actuó) |
| B3 | 5/5 | 0 | 0.1444 | N/D | N/D | N/D | HÍBRIDO ASISTIDO (Mamdani activo; no puro) |
| B4 | 5/5 | 0 | 0.1471 | N/D | 0.3000 | 0.0001 | CENTRADO GEOMÉTRICO (FIS en sombra) |

**Interpretación:** el cierre A17 informa cobertura válida, 20/20 llegadas y cero contactos medidos en sus tablas. En los JSON preliminares B2–B4 constaba que todavía NO se había medido el contacto directamente; se trata de versiones y auditorías diferentes. **Los rosbags brutos no están todos en el repositorio publicado**: la auditoría presente valida las fuentes archivadas y sus hashes, no reproduce la medición original.

**Datos ausentes de A17:** B1–B4 no publican tiempo de llegada ni oscilaciones de ω para esas mismas corridas. B1–B3 tampoco incluyen la misma columna de despeje y desviación completa. Se conserva N/D en las tablas en lugar de imputar datos.

**Responsabilidad del control:** B1 sigue A*, B2 completa desvío A* pero **no** demuestra actuación Mamdani, B3 combina Mamdani activo y subobjetivos geométricos (no Mamdani puro; 162 comandos activos DIFUSO en total), B4 completa centrado geométrico con FIS en sombra. Estas condiciones impiden atribuir los 20 éxitos a evasión difusa exclusiva.

## 3. Métricas adicionales de series históricas independientes

| Serie | n | Duración media (s) | Recorrido medio (m) | Despeje mínimo geom. (m) | Desv. máxima (m) | Σ|Δω| media (rad/s) | Inversiones signo ω medias |
|---|---:|---:|---:|---:|---:|---:|---:|
| B1 temprana | 5 | 16.948 | N/D | 0.2000 | 0.0001 | 0.0001 | N/D |
| B2 temprana | 5 | 31.769 | 2.2141 | 0.3447 | 0.0313 | 2.7379 | 1.00 |
| B3 temprana | 5 | 50.785 | 2.3900 | 0.3124 | 0.4453 | 6.6013 | 2.00 |
| B4 temprana | 5 | 31.488 | 1.8534 | 0.3000 | 0.0001 | 0.0000 | 0.00 |

**B1_3 registra 0 oscilaciones de ω en las cinco repeticiones**, según su JSON. Su campo `distancia_neta_m` es desplazamiento neto, no una trayectoria integrada; por ello el recorrido de esa serie aparece como N/D y su desplazamiento se conserva en una columna aparte.

**Método temporal:** B1_3 usa `tiempo_llegada_s_reloj_real` consignado en su `resumen.json`; B2–B4 usan `t_s(último)-t_s(primero)` de `control.csv`. Por sus orígenes distintos, NO son una comparación de rendimiento temporal normalizada ni tiempos válidos para sustituir los de A17.

**Oscilaciones:** para B1_3 se retiene el indicador histórico `oscilaciones_omega`; para B2–B4 se calculó la **variación angular acumulada** `Σ|ωᵢ-ωᵢ₋₁|` y las **inversiones de signo** ignorando valores `|ω|≤0,05 rad/s`. Son indicadores derivados de COMANDOS, no observaciones directas de la velocidad angular física del robot; el umbral es una decisión metodológica explícita. La suma no debe interpretarse como giro acumulado en radianes.

**Distancia mínima:** B1_3 utiliza distancia robot/obstáculo geométrica, aunque no se inserta una caja en ese pasillo libre; B2–B4 reportan distancia mínima a superficies SDF. No son medidas homogéneas de distancia a una misma clase de obstáculo.

**Contactos en las trazas tempranas:** en los JSON B2–B4 aparece `contactos_gazebo_verificados: false`; en B1_3 tampoco consta sensor de contacto medido. Es incorrecto afirmar cero CONTACTOS FÍSICOS a partir de esas fuentes. El cero que figura en A17 corresponde a la auditoría posterior de cobertura.

## 4. Figuras y archivos reproducibles

- `docs/A2_ACT13_A17_20_REPETICIONES.csv`: las 20 filas A17, sin datos imputados.
- `docs/A2_ACT13_A17_RESUMEN.csv`: resultados agregados exclusivos de A17.
- `docs/A2_ACT13_HISTORICOS_20_REPETICIONES.csv`: 20 pruebas más tempranas, independientes.
- `docs/A2_ACT13_HISTORICOS_RESUMEN.csv`: promedios de ensayos tempranos (no comparable temporalmente con A17).
- `docs/A2_ACT13_FUENTES_SHA256.csv`: manifiesto de fuentes originales sin modificaciones.
- `docs/evidencias_A2/ACT13/*.svg`: cuatro gráficos construidos directamente de las tablas anteriores.
- `tools/a2_act13_consolidar_escenarios.py`: regeneración y verificación deterministas (Python estándar).

## 5. Limitaciones y próximos pasos

1. Completar una captura uniforme con tiempo simulado de llegada, despeje, error respecto a A*, señales ω reales y contacto Gazebo para cinco repeticiones de **cada** escenario bajo la misma versión de control.
2. Repetir B2 y B4 demostrando actuación directa del difuso si se pretende acreditar evasión Mamdani pura; B3 ya demuestra actuación pero está asistido.
3. Utilizar los resultados A13 (experimento pareado v03/v04) exclusivamente como estudio de rediseño separado; no incorporarlos como repeticiones de A12.
4. Preservar los registros y variantes originales. No reescribir `avance2` ni `final`.

## 6. Procedimiento de reproducción

Desde la raíz del repositorio en Ubuntu:

```bash
python3 tools/a2_act13_consolidar_escenarios.py --check
# Solo si necesita regenerar los derivados del ACT13:
python3 tools/a2_act13_consolidar_escenarios.py --write
```

**Criterio de cierre:** la actividad 13 documenta honestamente toda la información publicada y sus ausencias. No constituye la validación integral pendiente de los cuatro escenarios, que sigue requiriendo pruebas uniformes.
