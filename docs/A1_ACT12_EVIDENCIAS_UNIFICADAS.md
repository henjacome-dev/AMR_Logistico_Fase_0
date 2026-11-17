# A1 / Actividad 12 — Evidencias unificadas (mundo, sensores, arquitectura y dataset)

**Estado:** consolidación documental, sin alterar las fuentes. **Corte de trabajo:** rama `u3s7-entrega-final-2026-11-16`, hasta `32b82ff`.
**Referencias históricas:** ZIP A1 presentado el **4/11/2026, 20:37 UTC−5** (`AMR_Logistico_Fase_1-main.zip`, 76 archivos; commit idéntico `d1558d7`), tag `avance1` (commit posterior `db4e4b4`, 5/11), y ZIP U3S7 del 16/11 (1.245 archivos, commit `e38b177`).
**Advertencia:** este documento consolida evidencia existente. No valida nuevas pruebas ROS/Gazebo ni reescribe el estado de A1 del 4/11.

## 1. Mundo Gazebo — requisito A1.3

| Elemento | Fuente principal | Comprobación e interpretación |
|---|---|---|
| Geometría original | `config/escenario_f1.yaml`, `modelos/almacen_f1.world` | 10 × 8 m, dos estanterías, cuatro estaciones; originales A1 con identificadores en suelo, **sin paneles con texturas** |
| Mapa de ocupación | `mapas/almacen_f1.pgm`, `mapas/almacen_f1_inflado.pgm`, YAML correspondientes | Resolución 0,05 m; inflación 0,21 m; configuración manual, no SLAM |
| Corrección R04 (5/11) | `docs/F1_R04_correccion_geometrica.md`, `resultados/EX01_auditoria_R04_v2.txt` | Reubicación de dos cajas; tres pasillos sin advertencias de ancho y cuatro estaciones accesibles; **validación estática**, no repetición de navegación |
| Paneles R05 (5/11) | `modelos/senales_almacen/materials/scripts/estaciones.material`, cuatro PNG, `docs/F1_R05_texturas_estaciones.md` | Cuatro paneles con materiales OGRE y símbolos I/II/III/IV |
| Evidencias visuales | `resultados/R05_capturas/R05_gazebo_recepcion_almacenamiento.png`, `.../R05_gazebo_empaque_despacho.png` | Capturas realizadas en Gazebo; no deben presentarse como imágenes del ZIP del 4/11 |

## 2. Sensores — requisito A1.4

**Corte del 4/11:** `config/interfaces.yaml` definía los tópicos, pero el ZIP no contenía `tools/emulador_sensores_f1.py`. No debe declararse implementado en ese corte.

**Evidencia posterior:** `tools/emulador_sensores_f1.py` recibe `/scan` (`LaserScan`) y publica tres `sensor_msgs/Range` con centros 0°, +45°, −45°, ancho sectorial 15°, alcance máximo 2 m y ruido gaussiano σ=0,01 m. `docs/F1_B3_emulador_sensores.md` registra la recepción ROS 2. R07 valida orientaciones y R08 usa 600 muestras por sensor en una prueba sintética:

| Tópico | Media (m) | Sigma muestral (m) | n |
|---|---:|---:|---:|
| `/rango_frontal` | 0,700508 | 0,010093 | 600 |
| `/rango_izq` | 1,100382 | 0,010029 | 600 |
| `/rango_der` | 1,399916 | 0,010117 | 600 |

`resultados/F1_A_cierre_20261106/actividad_B_pared_oeste.txt` documenta que al avanzar 0,852 m hacia una pared, la lectura frontal disminuyó 0,842 m. Esta es una prueba de respuesta dinámica en **simulación**, no de sensores físicos.

## 3. Arquitectura — requisito A1.7

**Diseño original (4/11):** `docs/F0_arquitectura.md` y `config/interfaces.yaml` especifican cinco nodos, sus interfaces y el supervisor. No equivalen a demostrar que todos estén activos en ese corte.

```text
A* + mapa ── rutas CSV ───────────► seguidor_ruta ── /cmd_vel_ruta ─┐
/scan ──► emulador_sensores ── Range x3 ──► controlador_difuso ── /cmd_vel_difuso ─┼► supervisor ─► /cmd_vel ─► TurtleBot3
/odom ───────────────────────────► seguidor_ruta ─────────────────────┘
/camera/image_raw ───────────────► reconocedor_estacion ── /estacion_detectada ──► supervisor
```

Este es el **flujo híbrido objetivo**, no una certificación de activación simultánea en A1. `docs/F1_B5_integracion_navegacion.md` demuestra posteriormente emulador, seguidor y supervisor; aclara que entonces los `Range` aún no gobernaban el movimiento, ni estaban integrados el difuso ni la CNN. `docs/F1_R35_marcos_tf.md` valida `map→odom` fijo en simulación con el robot detenido; no prueba localización global o eliminación de deriva.

## 4. Plan y dataset — requisito A1.7, evolución A2/A3

| Hito | Archivos | Resultado y condición |
|---|---|---|
| Diseño en informe A1 (4/11) | `Avance1_AMR_Grupo_1.docx.pdf` adjunto en Blackboard, fuera del Git público | Objetivo de ≥150 imágenes por estación, 0,3–1,5 m y ±30° |
| Protocolo de captura posterior | `docs/F1_plan_captura_CNN.md`, `datos/cnn/plan_adquisicion.csv` | 5 distancias × 5 ángulos × 6 tomas = 150 por estación; 600 filas **PLANIFICADA**, no imágenes |
| Captura posterior de Gazebo | `datos/u3s7/manifiesto.csv`, `datos/u3s7/resumen_dataset.json`, `datos/u3s7/imagenes/` | 600 PNG: cuatro clases de 150; 420 train, 90 val, 90 test; 100 grupos de pose sin cruce |
| Control de integridad | Manifiesto con `sha256` individual y 600 rutas de archivo | Los 600 hashes concuerdan con las imágenes archivadas; fuente visual **simulada** |

**Matiz:** el reparto por estación entre train/val/test en `datos/cnn/plan_adquisicion.csv` no coincide íntegramente con el reparto de `datos/u3s7/manifiesto.csv`, aunque los totales globales y las 150 imágenes por clase sí coinciden. El manifiesto de capturas es la referencia definitiva del dataset, no el plan de adquisición. La configuración óptica temporal está consignada en `datos/u3s7/configuracion_optica.json`; esta evidencia no valida cámaras físicas.

## 5. Interpretación para rúbrica y defensa

- **A1.3:** mundo y mapas en el ZIP original; paneles texturizados y corrección geométrica acreditados **después** de Blackboard.
- **A1.4:** sensores definidos pero sin nodo ejecutable en ZIP original; implementación y pruebas posteriores acreditadas documentalmente.
- **A1.7:** arquitectura propuesta en el original; diagrama y protocolo posteriores; dataset efectivamente capturado solo en etapas posteriores.
- **No extrapolar:** trazas sintéticas R07–R08 ≠ sensores físicos; pruebas de un módulo ≠ integración híbrida final; datos capturados ≠ resultado de entrenamiento CNN.
- **Reproducción:** ejecutar `python3 tools/a1_act12_verificar_evidencias.py --zip-original /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip --check` desde la raíz del repositorio. Requiere solo Python 3 estándar. El comprobador valida archivos, hashes, estructura del mundo, datos tabulados y separación de grupos, **sin arrancar ROS**.

## 6. Preservación de versiones

Este documento es una **consolidación retrospectiva**. No mueve los tags `avance1` o `final`, no sustituye el ZIP entregado al docente, no crea nuevos resultados experimentales y no modifica los archivos de fuente originales. Las dos capturas R05 y las imágenes RViz2 se enlazan desde sus ubicaciones actuales para evitar copias innecesarias.

**Trazabilidad adicional:** `docs/A1_ACT12_MATRIZ_EVIDENCIAS.csv` y `docs/A1_ACT12_MANIFIESTO_SHA256.csv`.
