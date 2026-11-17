# A1 — Actividad 14: Reproducción controlada de la orden A

## Objeto y delimitación histórica

El avance A1 fue presentado en Blackboard el 4-11-2026 (`AMR_Logistico_Fase_1-main.zip`, commit `d1558d7`). La demostración de **orden A completa** que se toma como referencia fue ejecutada **después**, en el cierre del 6-11-2026 (`resultados/F1_A_cierre_20261106/`). Por tanto, una repetición ahora **no acredita retroactivamente** el contenido del 4-11 ni debe presentarse como si se hubiese ejecutado en aquella fecha.

## Línea base verificable (cierre F1 del 6-11)

- Plataforma: Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic / TurtleBot3 Waffle Pi.
- Mundo: `modelos/almacen_f1.world` y lanzamiento `launch/almacen_operativo_f1.launch.py`.
- Ruta: `resultados/R04_astar_mapa_corregido/resultados/F12_orden_A.csv`, 405 puntos, 20,200 m.
- Estaciones visitadas: Recepción → Almacenamiento → Empaque → Despacho (4/4).
- Seguimiento histórico: 275,03 s de tiempo del proceso, 20,022 m odométricos, error final registrado 0,0711 m; 16.937 mensajes ROS en el bag histórico.
- Seguridad: el seguidor F1 publica `/cmd_vel_ruta` y el supervisor es el único publicador de `/cmd_vel`. No usa controlador Mamdani ni CNN. **No hay prueba independiente de contactos** en el cierre histórico.

Los datos anteriores son **referenciales** y no deben reemplazarse ni copiarse como si fueran resultados nuevos.

## Preparación y dos terminales

**Requisitos:** WSL Ubuntu con ROS 2 Humble, Gazebo Classic y TurtleBot3 Waffle Pi previamente funcionales; disponer al menos de 500 MB para bag. No ejecutar otra instancia de Gazebo, supervisor, Nav2 ni teleoperación.

Actualizar antes de ejecutar: `amr iniciar`.

En **terminal A**:

```bash
cd ~/AMR_publicacion_u3s7-entrega-final-2026-11-16
source tools/activar_entorno.sh
ros2 launch ./launch/almacen_operativo_f1.launch.py
```

Confirmar en la GUI que se cargó el mundo, que no hay cajas en la ruta y que el robot se encuentra próximo a `(-3,-2.8)` m. Esperar a que `/odom` y `/scan` publiquen datos.

En **terminal B**, abrir otra consola Ubuntu:

```bash
cd ~/AMR_publicacion_u3s7-entrega-final-2026-11-16
source tools/activar_entorno.sh
python3 tools/a1_act14_validar_orden_a.py --check
bash tools/a1_act14_ejecutar_orden_a.sh
```

La ejecución hace una prevalidación del entorno y de la autoridad única de `/cmd_vel`, valida la ruta, conserva logs y bag en `~/AMR_evidencias_privadas/A1_ACT14/ejecucion_.../` y pide una confirmación antes de enviar movimiento. Usa una **copia temporal del seguidor histórico** (`TIEMPO_RUTA_MAX = 720 s`) en la que cambia **únicamente la ruta de salida del CSV**, para evitar sobrescribir el log original que sí está versionado. Se conserva `ajuste_temporal.diff` como evidencia de la única modificación de la copia.

No se ejecuta con obstáculos dinámicos porque se pretende validar el alcance estricto de A1 (navegación global sin obstáculos). Para registrar Mamdani se usan las pruebas específicas A2, no este protocolo.

## Aceptación

| Criterio | Condición de aprobación | Prueba |
|---|---|---|
| Origen de trayectoria | CSV de 405 puntos, 20,200 m, hash coincidente | Verificador `--check` |
| Autoridad de movimiento | `/cmd_vel` con un publicador y `/cmd_vel_ruta` sin otro seguidor antes del inicio | Prevalidación ROS |
| Llegada a destino | Código de salida 0 y mensaje `Meta alcanzada` | `seguidor.log`, `seguidor_exit_code.txt` |
| Precisión | Error final reportado ≤ 0,08 m (redondeo ±0,001 m) | `seguidor.log` |
| Estaciones | 4/4 pasos ordenados a menos de 0,20 m del centro | CSV de seguimiento |
| Tiempo límite interno | ≤ 720 s de proceso; **no** es el límite 130 % de la entrega A3 | CSV de seguimiento |
| Evidencia | Logs y bag fuera del Git público; captura de GUI opcional | Carpeta privada |

**No se puede certificar cero colisiones físicas sin un sensor o medición de contactos independiente**. La presencia del ROS bag demuestra registro de tópicos, pero no equivale a certificar contactos.

## Evaluación y publicación del resumen

El ejecutor invoca automáticamente `--evaluar`. Si hace falta repetir la verificación:

```bash
python3 tools/a1_act14_validar_orden_a.py --evaluar \
  "$HOME/AMR_evidencias_privadas/A1_ACT14/ejecucion_FECHA"
```

Tras revisar un resultado **APROBADO**, publica **solo el resumen Markdown**, no los archivos pesados del rosbag:

```bash
cp "$HOME/AMR_evidencias_privadas/A1_ACT14/ejecucion_FECHA/A1_ACT14_RESULTADO_PUBLICABLE.md" \
  docs/A1_ACT14_RESULTADO_REPRODUCCION.md
amr guardar "test(a1): documentar reproducción de orden A en Gazebo"
```

`FECHA` se reemplaza por el nombre real mostrado por el ejecutor. Si el ensayo falla, **no se crea una declaración de éxito**: conservar y analizar los registros privados antes de repetir. El archivo `docs/A1_ACT14_RESULTADO_REPRODUCCION.md` solo se debe publicar cuando exista evidencia de éxito efectivamente reproducido.

## Procedencia de datos y limitaciones

- El ZIP A1 entregado el 4 de noviembre y el tag `avance1` del 5 de noviembre están auditados por las actividades 5, 9 y 12.
- Esta repetición utiliza fuentes del cierre F1 **del 6 de noviembre** presentes en `final` y no modifica el comportamiento del controlador.
- Si se modifica mundo, ubicación inicial, ruta o código, la prueba debe declararse nueva variante; no reutilizar este acta como equivalencia exacta.
- El controlador F1 detiene el movimiento ante riesgo; no replanifica obstáculos espontáneos. El sensor `Range` se registra, pero la navegación utiliza `/scan`.
