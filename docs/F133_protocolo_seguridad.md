# F1.3.3 — Protocolo de validación de seguridad (AMR)

**Proyecto:** AMR_Logistico_Fase_0 (ROS 2 Humble, Gazebo Classic, TurtleBot3 Waffle Pi)
**Ámbito:** F1.3.3, prueba adicional interna. No sustituye los entregables oficiales del Avance 1.
**Estado:** 12/12 pruebas lógicas aprobadas; GI-01 y GI-02 ejecutadas con resultados satisfactorios; GI-03 pendiente.

## 1. Objetivo

Comprobar el mecanismo de parada y limitación de velocidad del archivo `tools/supervisor_f13.py` bajo condiciones artificiales reproducibles. Estas pruebas se realizan **sin movimiento del robot**: el banco remapea las interfaces a los tópicos `/f133/cmd_vel`, `/f133/cmd_vel_ruta`, `/f133/scan` y `/f133/odom`.

El proceso de pruebas arranca una instancia separada (`supervisor_f133_test`) y verifica que **no publique en `/cmd_vel`**. Se requiere Gazebo ejecutándose con `/clock` activo para que avance el temporizador del supervisor configurado con `use_sim_time:=true`.

## 2. Parámetros implementados actualmente

| Parámetro | Valor actual | Archivo |
|---|---:|---|
| Velocidad lineal máxima | 0,12 m/s | `tools/supervisor_f13.py` |
| Velocidad angular máxima | ±0,70 rad/s | Ídem |
| Distancia de parada frontal | menor de 0,35 m | Ídem |
| Datos LiDAR/odometría vencidos | mayor de 1,5 s reales | Ídem |
| Comando de ruta vencido | mayor de 0,4 s reales | Ídem |
| Supervisión periódica | 0,1 s de reloj ROS | Ídem |

Estos umbrales son **provisionales** y no constituyen certificación de seguridad física.

## 3. Pruebas automáticas de aislamiento ROS

| Código | Condición | Salida esperada |
|---|---|---|
| S01 | No hay propuesta de ruta | Velocidad cero |
| S02 | Propuesta válida, LiDAR y odometría sanos | Avance limitado y no nulo |
| S03 | Velocidades de ruta superiores a los máximos | Saturación a 0,12 m/s y ±0,70 rad/s |
| S04 | Obstáculo frontal simulado a 0,20 m | Velocidad cero |
| S05 | LiDAR no finito (NaN) | Velocidad cero |
| S06 | Comando de ruta con NaN | Velocidad cero |
| S07 | Comando con retroceso no autorizado | Velocidad cero |
| S08 | Componente lateral no autorizada | Velocidad cero |
| S09 | Propuesta sin renovación durante más de 0,4 s | Velocidad cero |
| S10 | LiDAR sin renovación durante más de 1,5 s | Velocidad cero |
| S11 | Odometría sin renovación durante más de 1,5 s | Velocidad cero |
| S12 | SIGINT controlado al supervisor aislado | Emisión de parada antes del fin del proceso |

**Criterio de cierre de este nivel:** 12/12 comprobaciones con estado `OK`; revisar cualquier `FALLO`, en especial S12, antes de concluir que la parada por interrupción es confiable.

## 4. Instalación y ejecución desde WSL2 (Ubuntu)

1. Descargar `prueba_seguridad_f133.py` desde esta conversación al directorio Descargas de Windows.
2. Abrir terminal Ubuntu y ejecutar (ajustar la ruta de Windows si fuera distinta):

```bash
cd ~/AMR_Logistico_Fase_0
cp /mnt/c/Users/Henry/Downloads/prueba_seguridad_f133.py tools/
python3 -m py_compile tools/prueba_seguridad_f133.py
source tools/activar_entorno.sh
```

3. Mantener abierto Gazebo Classic con el almacén y la simulación **sin pausar**. Detener teleoperación y cualquier batería de pruebas concurrente.
4. Verificar el banco y ejecutarlo:

```bash
python3 tools/prueba_seguridad_f133.py --preflight
python3 tools/prueba_seguridad_f133.py
```

5. Revisar la carpeta `resultados/F133_seguridad_YYYYMMDD_HHMMSS/`, especialmente `resumen.csv` y `supervisor.log`.

## 5. Segunda etapa — Pruebas integradas Gazebo

Tras aprobar S01–S12, ejecutar en un entorno controlado separado tres pruebas del sistema completo:

- **G01 — Obstáculo estático:** iniciar una ruta libre y añadir un obstáculo inmóvil en el trayecto con geometría y distancia conocidas; validar que la parada se activa cuando la lectura frontal cae por debajo de 0,35 m. Medir con odometría la distancia adicional recorrida durante el frenado. No presentar el LiDAR como detector definitivo de colisión.
- **G02 — Pérdida de entradas:** con el robot en movimiento, detener de forma controlada el productor de propuestas. Verificar parada tras caducidad de 0,4 s, más latencias de comunicación y ejecución. Repetir ante pérdida de LiDAR y odometría, si se dispone de un mecanismo de inyección sin alterar Gazebo.
- **G03 — Recuperación:** retirar el obstáculo o restituir los datos y comprobar que el sistema no arranca espontáneamente sin una autorización de reanudación. El supervisor F1.3 actual **no implementa un enclavamiento de falla**; una reanudación automática se debe registrar como limitación y tratar antes de usar el sistema en un entorno físico.

**No tocar la simulación real durante S01–S12.** G01–G03 necesitan procedimientos separados para evitar falsos positivos y preservar evidencias.

## 6. Evidencias finales requeridas

- Captura del `preflight` satisfactorio.
- `resumen.csv` y `supervisor.log` del banco aislado.
- Registros ROS y de odometría de G01–G03, señalando antes/después de cada evento.
- Matriz de resultados, problema detectado, corrección aplicada y revalidación.
- `git diff --check`, commit identificado y tag o referencia correspondiente cuando F1.3.3 esté cerrada.

## 7. Limitaciones relevantes

- `supervisor_f13.py` no emplea todavía el emulador obligatorio de tres sensores `sensor_msgs/Range`, sino directamente `/scan`; eso corresponde a una integración posterior.
- El supervisor hace una parada reactiva; no se ha comprobado frenado físico, colisión real, fallo del actuador, fallo total de ROS ni watchdog independiente.
- Una parada de `supervisor_f13.py` no sustituye un paro de emergencia físico. Si el publicador `/cmd_vel` desaparece, la respuesta depende del mecanismo de timeout del controlador inferior, que debe verificarse por separado.
- Estas pruebas de seguridad son una mejora interna. El enunciado oficial del Avance 1 todavía exige cerrar el emulador de sensores, los paneles de estaciones y los entregables documentales.


## 8. Resultados de validación

### 8.1 Seguridad lógica

- Resultado: 12 de 12 controles aprobados.
- Evidencia: `resultados/F133_seguridad_20261105_132453/`
- Alcance: validación aislada ROS 2.
- S12: parada estable observada tras SIGINT.

### 8.2 Pruebas integradas

**GI-01 — Obstáculo frontal**

- Obstáculo detectado por el sistema.
- Supervisor ordenó velocidad lineal y angular cero.
- Evidencias registradas en los archivos F133_GI01.

**GI-02 — Interrupción de propuestas**

- 82 muestras de comandos registradas.
- 36 muestras de avance a 0,04 m/s.
- Comandos cero observados posteriormente.
- Latencia de parada observada: 0,449 s.
- Evidencias registradas en los archivos F133_GI02.

**GI-03 — Recuperación**

- Estado: pendiente de validación integrada.
- No se declara recuperación autónoma verificada.

### 8.3 Limitaciones

La validación lógica y los ensayos integrados no constituyen
certificación de seguridad física, ni demuestran ausencia de
colisiones en todos los escenarios.

Los umbrales utilizados son provisionales y requieren
calibración para su aplicación en un robot físico.
