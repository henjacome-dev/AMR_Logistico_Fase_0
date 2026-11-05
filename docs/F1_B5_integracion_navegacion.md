# B5 — Integracion operativa de navegacion

Fecha: 2026-11-05
Entorno: Ubuntu 22.04, ROS 2 Humble y Gazebo Classic.

## 1. Componentes implementados

| Componente | Entradas | Salidas |
|---|---|---|
| Gazebo/TurtleBot3 | /cmd_vel | /scan, /odom |
| Emulador | /scan | /rango_frontal, /rango_izq, /rango_der |
| Seguidor proporcional | CSV, /odom, /scan | /cmd_vel_ruta |
| Supervisor | /cmd_vel_ruta, /scan, /odom | /cmd_vel |

El emulador publica tres mensajes sensor_msgs/msg/Range.

El supervisor es el unico publicador autonomo de /cmd_vel.
Limita velocidades y ordena parada ante datos invalidos,
obstaculos frontales o falta de propuestas recientes.

## 2. Integracion actual y limitaciones

- Los tres sensores virtuales estan implementados y operativos.
- El supervisor y el seguidor consumen directamente /scan.
- Los mensajes Range aun no alimentan el control de movimiento.
- No existe arbitraje implementado con un controlador difuso.
- No existe integracion de reconocimiento CNN.
- No se declara replanificacion automatica tras obstaculos.
- La seguridad verificada corresponde solamente a simulacion.

## 3. Lanzamiento del sistema

Archivo: launch/almacen_operativo_f1.launch.py

Con el entorno ROS 2 activado y sin otra instancia de Gazebo
ni supervisor, ejecutar desde la raiz del repositorio:

ros2 launch ./launch/almacen_operativo_f1.launch.py

El lanzamiento inicia el almacen, el emulador y el supervisor.
No ejecuta automaticamente ninguna ruta.

Para validar el formato de una ruta sin movimiento:

python3 tools/seguidor_ruta_f13.py \
  --ruta resultados/F12_ruta_1.csv --validar

Para ejecutar una ruta, se requiere que el robot este en el
origen correspondiente y que el entorno este despejado.
El seguidor se ejecuta separadamente con use_sim_time:=true.

No ejecutar teleoperacion directa mientras el supervisor
sea responsable de /cmd_vel.

## 4. Evidencias previas

- F1.2: mapas, rutas A* y comparacion MATLAB-Python.
- F1.3.1: seguimiento de cuatro rutas.
- F1.3.2: 12 ensayos satisfactorios de repetibilidad.
- F1.3.3: controles S01-S12 aprobados.
- GI-01: parada ante obstaculo frontal.
- GI-02: parada tras interrupcion de propuestas.
- GI-03: recuperacion mediante nueva propuesta valida.
- B3: tres topicos Range verificados en ROS 2.

Las pruebas previas no constituyen certificacion de
seguridad fisica ni validan componentes aun no integrados.

## 5. Verificacion EX-02

El arranque integrado fue satisfactorio en la ejecucion
observada. El supervisor fue el unico publicador de
/cmd_vel y envio velocidad cero sin ruta activa.
Ver docs/F1_EX02_arranque_integrado.md.

No se repiten los ensayos F1.3.1, F1.3.2 ni F1.3.3.
