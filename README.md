# AMR logístico para almacén — Autómatas y Expertos (MEC451)

**Universidad ECOTEC · Ingeniería Mecatrónica**  
**Proyecto 1:** Robot móvil autónomo para logística en almacén  
**Hitos documentados:** Avance 1 (corte técnico y documental de noviembre de 2026) y Avance 2, Fase 1 (auditoría numérica MATLAB del 6 de noviembre de 2026; revisión de evidencias del 7 de noviembre de 2026)
**Plataforma:** Ubuntu 22.04 (WSL2), ROS 2 Humble, Gazebo Classic y TurtleBot3 Waffle Pi

## 1. Objetivo y alcance

Desarrollar un prototipo de robot móvil autónomo para tareas logísticas en un almacén simulado. El Avance 1 comprende escenario Gazebo, estaciones, sensores, mapa manual, planificación A* y seguimiento de rutas. El proyecto completo contempla posteriormente control difuso, reconocimiento visual CNN e implementación física con ESP32.

**Estado del Avance 1:** infraestructura y componentes de navegación demostrados en simulación; verificaciones EX-01 y EX-02 realizadas. Las observaciones técnicas R04, R05, R07, R08 y R11 fueron regularizadas e integradas en `main`. Las actividades de cierre A–E quedaron verificadas. La Fase 1 del Avance 2 (controlador Mamdani) está validada numéricamente en MATLAB Online; la integración ROS 2, las pruebas operativas y los componentes posteriores siguen pendientes. **No equivale a la conclusión del proyecto completo.**

## 2. Inicio rápido del sistema integrado

Ejecutar desde Ubuntu 22.04 con ROS 2 Humble, Gazebo Classic y el workspace TurtleBot3 previamente instalados. Cerrar instancias anteriores de Gazebo y del supervisor antes de iniciar:

```bash
cd ~/AMR_Logistico_Fase_0
source tools/activar_entorno.sh
ros2 launch ./launch/almacen_operativo_f1.launch.py
```

Este comando inicia el almacén, TurtleBot3 Waffle Pi, el emulador de sensores y el supervisor. **No inicia una ruta ni ordena movimiento automático.**

En una segunda terminal:

```bash
cd ~/AMR_Logistico_Fase_0
source tools/activar_entorno.sh
ros2 node list
ros2 topic list -t
ros2 topic info /cmd_vel -v
ros2 topic echo /rango_frontal --once
```

El supervisor debe ser el único publicador autónomo en `/cmd_vel`. Cuando no existe una propuesta de ruta válida, publica velocidad cero. **No ejecutar teleoperación simultánea mientras el supervisor controle `/cmd_vel`.**

Para abrir solamente el escenario vigente corregido, sin lanzar emulador ni supervisor:

```bash
ros2 launch ./launch/almacen_f1.launch.py
```

Validación estática del formato de una ruta, sin mover el robot:

```bash
python3 tools/seguidor_ruta_f13.py --ruta resultados/F12_ruta_1.csv --validar
```

El seguidor real se ejecuta por separado únicamente cuando el robot se encuentra en el origen de la ruta y el supervisor está activo. Los ensayos ya realizados se conservan en `resultados/`.

## 3. Escenario y mapa

- Mundo: **10 × 8 m**; paredes y dos estantes longitudinales.
- Estaciones: **RECEPCIÓN** (azul), **ALMACENAMIENTO** (rojo), **EMPAQUE** (verde) y **DESPACHO** (amarillo).
- Identificación visual: cuatro paneles con texturas PNG y materiales OGRE, además de marcas geométricas, sin nueva geometría de colisión.
- Mapa de ocupación manual **sin SLAM**, con resolución **0,05 m/celda** y radio de inflado configurado de **0,21 m**.
- Mapas: `mapas/almacen_f1.pgm` y `mapas/almacen_f1_inflado.pgm`, ambos de **200 × 160 celdas**.
- Configuración común: `config/escenario_f1.yaml`.
- Generadores: `tools/generar_mundo_f1.py` y `tools/generar_mapa_f1.py`.

EX-01 confirmó la correspondencia estática entre configuración, SDF y mapas, así como las cuatro metas libres en el mapa inflado. **R04 corregido:** se reposicionaron dos obstáculos de prueba. La auditoría estática actual no presenta advertencias de ancho; los anchos nominales son 2,20 m, 2,10 m y 2,20 m. Las rutas históricas se comprobaron sobre el mapa nuevo sin conflictos de ocupación. Posteriormente se validó una orden logística integral sobre el escenario corregido: cuatro estaciones alcanzadas, 275,03 s y error final al waypoint de 0,0711 m. No se repitió la batería histórica completa. Ver `docs/F1_R04_correccion_geometrica.md`.

## 4. Interfaces ROS 2

| Tópico | Tipo | Responsable / uso |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR de TurtleBot3 |
| `/odom` | `nav_msgs/msg/Odometry` | Odometría |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Cámara simulada |
| `/rango_frontal` | `sensor_msgs/msg/Range` | Emulador, centro 0° |
| `/rango_izq` | `sensor_msgs/msg/Range` | Emulador, centro +45° |
| `/rango_der` | `sensor_msgs/msg/Range` | Emulador, centro −45° |
| `/cmd_vel_ruta` | `geometry_msgs/msg/Twist` | Propuesta del seguidor |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Orden final del supervisor |

El emulador `tools/emulador_sensores_f1.py` utiliza sectores de **15°**, alcance máximo **2,0 m** y ruido gaussiano configurado con **σ = 0,01 m** para detecciones no saturadas. Los tres tópicos `Range` fueron observados en ROS 2. R07 y R08 fueron verificados posteriormente mediante un ensayo aislado con LiDAR sintético: 600 muestras por sensor, centros de 0° y ±45°, límites sectoriales y desviaciones estándar muestrales próximas a 0,01 m. No equivale a una prueba física o de rendimiento dinámico en Gazebo. Ver `docs/F1_R0708_validacion_emulador.md`.

**Arquitectura implementada:** `/scan` alimenta al emulador y, por separado, al seguidor y supervisor; el seguidor publica en `/cmd_vel_ruta`; el supervisor filtra las propuestas y publica en `/cmd_vel`. Las salidas `Range` **aún no alimentan el control de movimiento**. El arbitraje difuso y la CNN corresponden a hitos posteriores.

El supervisor limita la velocidad a **0,12 m/s** y el giro a **0,70 rad/s**, con umbral frontal provisional de **0,35 m**, caducidad de sensores de **1,5 s** y caducidad de propuestas de **0,4 s**. Son parámetros del prototipo simulado, no una certificación de seguridad física.

## 5. Planificación A* y resultados

El escenario original dispone de planificación Python y validación MATLAB con `binaryOccupancyMap` y `plannerAStarGrid`. Sus resultados históricos permanecen en `resultados/F12_*` y `matlab/F124_MATLAB/`. MATLAB Online fue el entorno de trabajo; no está acreditada una instalación local de MATLAB en Ubuntu.

| Orden | Python (m) | MATLAB (m) | Diferencia absoluta (m) |
|---|---:|---:|---:|
| A | 20,449 | 20,390 | 0,059 |
| B | 32,817 | 32,641 | 0,176 |
| C | 26,449 | 26,390 | 0,059 |

Entre estas tres alternativas, **A** tiene la menor longitud. Datos: `resultados/F12_comparacion_ordenes.csv` y `matlab/F124_MATLAB/resultados/F124_comparacion_final.csv`.

### Resultados posteriores sobre el escenario corregido

Las cifras anteriores pertenecen al escenario original del Avance 1.
No deben combinarse con los siguientes resultados.

| Orden | Python R04 (m) | MATLAB R11 (m) | Diferencia absoluta (m) |
|---|---:|---:|---:|
| A | 20,200 | 20,100 | 0,100 |
| B | 32,568 | 32,702 | 0,134 |
| C | 26,200 | 26,100 | 0,100 |

La orden A continúa siendo la más corta entre las alternativas evaluadas.

La validación MATLAB R11 aplica explícitamente
`inflate(mapa, 0.21)` al mapa base corregido.
MATLAB produjo 12.065 celdas ocupadas y Python 10.699.
La diferencia de 1.366 celdas está documentada y
no representa igualdad de ambos algoritmos de inflado.

Evidencias:
- `resultados/R04_astar_mapa_corregido/`
- `matlab/R11_MATLAB/resultados/`
- `docs/F1_R11_inflado_matlab.md`


## 6. Evidencias experimentales

| Evidencia | Resultado | Archivo de referencia |
|---|---|---|
| Seguimiento F1.3.1 | Cuatro rutas ejecutadas en simulación | `resultados/F13_ruta_1_log.csv` a `F13_ruta_4_log.csv` |
| Batería F1.3.2 | 12/12 repeticiones satisfactorias | `resultados/F131_bateria_20261105_005858/resumen.csv` |
| Seguridad F1.3.3 | S01–S12 satisfactorias | `resultados/F133_seguridad_20261105_132453/resumen.csv` |
| Integración GI-01 a GI-03 | Parada, pérdida de propuesta y recuperación verificadas según protocolos | `docs/F133_protocolo_seguridad.md` |
| B3 | Tres tópicos `Range` comprobados | `docs/F1_B3_emulador_sensores.md` |
| EX-01 original | Consistencia estática con dos advertencias de paso, antes de R04 | `resultados/EX01_auditoria.txt` |
| EX-02 | Arranque integrado y velocidad cero sin ruta | `docs/F1_EX02_arranque_integrado.md` |
| R04 | Geometría corregida, consistencia estática sin advertencias | `docs/F1_R04_correccion_geometrica.md` |
| R05 | Cuatro texturas y capturas verificadas en Gazebo | `docs/F1_R05_texturas_estaciones.md` |
| R07/R08 | Sectores y ruido verificados con LiDAR sintético | `docs/F1_R0708_validacion_emulador.md` |
| R11 | Inflado MATLAB y A* sobre mapa corregido | `docs/F1_R11_inflado_matlab.md` |
| Tarea 1.7 | Plan de 600 imágenes; 0 capturas acreditadas | `docs/F1_plan_captura_CNN.md` |

Las evidencias se refieren a simulación. GI-03 no acredita un enclavamiento de rearme manual. No se afirma evasión automática, replanificación ante obstáculos ni robustez física.

## 7. Organización del repositorio

- `config/`: escenario y contratos de interfaces.
- `modelos/`: mundo SDF del almacén.
- `mapas/`: mapas de ocupación base e inflado.
- `launch/`: lanzamiento del almacén y lanzamiento operativo integrado.
- `tools/`: generadores, planificadores/verificadores, seguidor, supervisor y emulador.
- `matlab/F124_MATLAB/`: resultados MATLAB históricos del escenario original.
- `matlab/R11_MATLAB/`: inflado y A* MATLAB del escenario corregido.
- `resultados/`: CSV, métricas, capturas y registros seleccionados de pruebas.
- `docs/`: requisitos F0, arquitectura, validaciones B3–B5 y EX-01/EX-02.
- `datos/cnn/plan_adquisicion.csv`: manifiesto de 600 capturas planificadas, aún no adquiridas.
- `ros2_ws/src/`, `firmware/`: estructura prevista para las siguientes fases.

Documentos de consulta: `docs/F0_requisitos.md`, `docs/F1_B4_mapa_planificacion.md`, `docs/F1_B5_integracion_navegacion.md`, `docs/F1_EX01_consistencia.md` y `docs/F1_EX02_arranque_integrado.md`.

## 8. Pendientes y alcance de la entrega

- **R04:** corregido y validado estática y dinámicamente; orden A integral satisfactoria y registros históricos conservados.
- **R05:** cuatro texturas y capturas verificadas y archivadas en GitHub.
- **R07–R08:** sectores y ruido validados con LiDAR sintético; no se afirma una prueba formal de normalidad.
- **R11:** inflado y planificación MATLAB ejecutados sobre el mapa corregido.
- **Tarea 1.7:** protocolo y 600 registros de adquisición planificados; no constituye un dataset capturado.
- **R22–R23:** dataset real y primer entrenamiento CNN pendientes.
- **R35:** verificado en simulación; CSV, mapa y odometría compatibles, con TF estática identidad `map → odom` integrada. Ver `docs/F1_R35_marcos_tf.md`.
- **R14:** informe entregado al docente mediante Blackboard, según confirmación del estudiante. No se incorpora a GitHub.
- **Fase 0 organizativa:** cinco roles asignados; accesos y acuerdos confirmados por el grupo el 2026-11-06, según declaración del responsable.
- **Avance 2, Fase 1:** diseño Mamdani validado en MATLAB Online (3 entradas, 2 salidas, 27 reglas, 1.331 pruebas de barrido y diez casos críticos). Nueve evidencias PNG reparadas en `desarrollo-avance2`, commit `6b46367`.
- **Próximos hitos:** validación operativa, integración ROS 2 y comparación MATLAB/Python; después CNN y plataforma física. Ninguno se presenta como terminado.

Hitos según el enunciado: Avance 1 (**4 nov. 2026**, 20 %), Avance 2 (**9 nov. 2026**, 35 %), final de simulación (**16 nov. 2026**, 45 %). La fase física se evalúa por separado.

**Etiquetas de cierre:** `avance1` corresponde al corte original; `avance1-v2` identifica la revisión documental que incorpora este README actualizado.

## 9. Trazabilidad de las correcciones posteriores

- `avance1`: corte original del Avance 1.
- `avance1-v2`: revisión documental posterior de ese corte.
- `correccion-avance1-r04`: rama de trabajo con regularizaciones
  R04, R05, R07, R08, R11 y planificación CNN.

Las etiquetas anteriores no se han modificado.
La rama de corrección no se presenta como una nueva
etiqueta oficial de entrega ni como proyecto finalizado.

Los resultados F1.2/F1.3 históricos corresponden al
escenario original. Sobre la geometría corregida se
ejecutó satisfactoriamente la orden A integral.
Ver `docs/F1_cierre_avance1_20261106.md`.

La Fase 0 cuenta con responsables y acuerdos grupales
confirmados. R14: entrega informada en Blackboard.
R35: verificado para la configuración de simulación.

## 10. Estado de la Fase 1 del Avance 2 (2026-11-06)

El controlador `matlab/A2_MAMDANI/AMR_Mamdani_A2_v01.fis` emplea tres entradas de distancia `df`, `di`, `dd` (0–2 m), dos salidas `v` (0–0,20 m/s) y `omega` (−1,5–1,5 rad/s), y 27 reglas Mamdani. El script `A2_F1_verificacion_auditoria.m` comprobó en MATLAB Online la carga de tres archivos `.mat`, la reproducción de 1.331 evaluaciones y diez casos críticos (error máximo mostrado: 0) y la integridad de nueve imágenes PNG.

**Alcance:** diseño, integridad de archivos y reproducibilidad numérica del FIS. La Fase 1 no certifica integración ROS 2, colisiones, saturación física ni equivalencia Python. Para `df=di=dd=0,2 m`, el Mamdani entrega `v≈0,011815589 m/s`: se requiere parada estricta en el supervisor.

**Trazabilidad:** `matlab/A2_MAMDANI/A2_F1_acta_auditoria_20261106.md`, `matlab/A2_MAMDANI/A2_F1_resumen_tecnico.md`, `matlab/A2_MAMDANI/evidencias/` y commit `6b46367`. Las pruebas históricas del Avance 1 permanecen sin redefinirse.

### Seguimiento de observaciones gráficas y documentales (2026-11-07)

- **AC-02 — Equivalencia MAT:** MATLAB confirmó equivalencia de configuración entre las copias de `A2_F1_membresias_base.mat`, pese a SHA-256 diferentes. En el cierre final, el ZIP adoptó la copia ya versionada en GitHub (`54abfd3`); la copia local alternativa se conservó sin cambios.
- **AC-01 — Reproducción gráfica:** nueve PNG válidos de 900 × 650 píxeles, generados mediante `getframe`/`imwrite` e inspeccionados visualmente. Los generadores corregidos y las nueve evidencias se publicaron en `desarrollo-avance2` (`36ce917`).
- **AC-05 — Identificación de entradas fijas:** cuatro superficies publicadas con las condiciones `dd=1.7 m`, `df=0.2 m`, `df=1.7 m` y `di=0.9 m`, respectivamente (`36ce917`).
- **AC-04:** corrección documental publicada; **AC-03:** paquete definitivo de 25 archivos y manifiesto SHA-256 publicados y reconciliados con el árbol remoto (`36ce917`, `54abfd3`). GitHub Actions `validar-avance2` aprobó ese cierre (run `42426641603`).

El commit `6b46367` reparó los nueve PNG originales; `3f0acc7` incorporó la auditoría numérica; `36ce917` publicó las correcciones AC-01/AC-05 y AC-04; `54abfd3` reconcilió el ZIP definitivo con GitHub. Las imágenes oficiales actualizadas están en `matlab/A2_MAMDANI/evidencias/`. La integración ROS 2 y las pruebas de seguridad operativa continúan pendientes. La comparación MATLAB/Python se aprobó posteriormente en R18.

## 11. Equivalencia MATLAB–Python: R18

La etapa E2.1/R18 aprobó 10/10 casos con `scikit-fuzzy` 0.5.0; errores relativos máximos: `v=0.502821%` y `omega=0.145568%` (umbral <2%). El código y las evidencias reproducibles se documentan en `docs/A2_R18_concordancia_MATLAB_Python.md`. La integración ROS 2 y el supervisor de parada estricta continúan pendientes.

## 11. Avance 2 — E2.2: nodo ROS 2 de propuestas difusas (2026-11-07)

El nodo `tools/A2_E22_nodo_difuso.py` reutiliza el motor Mamdani de R18,
consume los tres tópicos `Range` y publica propuestas únicamente en
`/cmd_vel_difuso` y estados en `/estado_difuso`. Se verificó en Ubuntu/ROS 2
Humble, mediante un dominio aislado sin Gazebo, el funcionamiento con
entradas válidas, ausencia de datos, caducidad, recuperación y rechazo de
`NaN`. No existen emisiones en `/cmd_vel` desde este nodo.

**Alcance:** E2.2 corresponde a validación de nodo aislado. La integración
con el supervisor, el arbitraje y las pruebas en Gazebo corresponden a E2.3
y E2.4. Ver `docs/A2_E22_validacion_nodo_difuso.md`.

## 12. Avance 2 — E2.3: arbitraje y supervisor en preintegración (2026-11-07)

Se verificó un árbitro RUTA/DIFUSO en modo sombra (E2.3.1: 17/17),
su comunicación mediante ROS 2 Humble aislado (E2.3.2: 14/14) y un
candidato de supervisor con barrera LiDAR adicional (E2.3.3: 20/20).
Las propuestas se publicaron solamente en tópicos de prueba; **no** en
`/cmd_vel`. El supervisor operativo `tools/supervisor_f13.py` y el
controlador `AMR_Mamdani_A2_v01.fis` no fueron modificados.

**R19 sigue pendiente:** no existe todavía integración operativa ni
prueba dinámica con Gazebo; los umbrales de cambio 0,60 y 0,80 m son
provisionales y requieren cotejo con el enunciado. Detalles y evidencias
en `docs/A2_E23_preintegracion_aislada.md`.


### Avance 2 — E2.3.4 y E2.3.5: Gazebo en modo sombra

E2.3.4: comunicación entre Gazebo, LiDAR, odometría, emulador de tres sensores,
control Mamdani y supervisor candidato: nueve tópicos registrados.
E2.3.5: secuencia RUTA → DIFUSO → DETENIDO → DIFUSO → RUTA mediante
obstáculos temporales en Gazebo; la propuesta de ruta fue siempre cero.
El candidato solo publicó `/cmd_vel_integrado_prueba`: **ningún publicador
activo en `/cmd_vel` durante el ensayo**. Véase
`docs/A2_E2345_gazebo_sombra.md` y el validador
`python3 tools/A2_E2345_validar_evidencias_gazebo.py`.

**Alcance:** pruebas en sombra, sin desplazamiento autónomo. Los umbrales
0,60/0,80 m son propuestas de ingeniería; **R19 operativo NO está cerrado**.


### Avance 2 — E2.3.6A: canal operativo /cmd_vel limitado a cero

En Gazebo Classic headless (ROS_DOMAIN_ID=224) se verificaron insercion
`waffle_pi`, diez topicos, cinco transiciones RUTA–DIFUSO–DETENIDO–DIFUSO–RUTA
y **un solo publicador** en `/cmd_vel`. El publicador autorizado emitio
exclusivamente `Twist()` de cero: 110 enviados, 110 recibidos, 9,39 Hz;
ninguna orden no nula. El desplazamiento por odometria fue inferior a 5 cm.

El supervisor candidato continuo enviando sus propuestas no nulas unicamente
a `/cmd_vel_integrado_prueba`. No se reenvio al robot ninguna propuesta no nula.
Evidencias: `resultados/A2_E236A_gazebo_headless_canal_cero/evidencia_publicada/`, validador
`python3 tools/A2_E236A_validar_evidencia_canal_cero.py` y `docs/A2_E236A_canal_cero_gazebo.md`.

**No es navegacion autonoma ni cierra R19:** falta cotejar los umbrales
experimentales 0,60/0,80 m con el enunciado, integrar el supervisor operativo
y demostrar detenciones bajo movimiento. GitHub Actions verifica registros
capturados en Ubuntu, no ejecuta Gazebo.
