# EX-02 — Arranque integrado del sistema AMR

Fecha: 2026-11-05
Entorno: ROS 2 Humble, Gazebo Classic, TurtleBot3 Waffle Pi.

## Objetivo

Comprobar la ejecución conjunta del simulador, el
emulador de sensores y el supervisor de movimiento,
sin iniciar automáticamente el seguimiento de rutas.

## Lanzamiento

Archivo:
launch/almacen_operativo_f1.launch.py

Comando:
ros2 launch ./launch/almacen_operativo_f1.launch.py

## Evidencia observada

Nodos presentes:
- /gazebo
- /robot_state_publisher
- /turtlebot3_diff_drive
- /turtlebot3_laserscan
- /turtlebot3_imu
- /turtlebot3_joint_state
- /camera_driver
- /emulador_sensores
- /supervisor_f13

Topicos disponibles:
- /scan
- /odom
- /rango_frontal
- /rango_izq
- /rango_der
- /cmd_vel
- /cmd_vel_ruta

Autoridad de movimiento:
- /cmd_vel: 1 publicador, /supervisor_f13.
- /cmd_vel: 1 suscriptor, /turtlebot3_diff_drive.
- /cmd_vel_ruta: 0 publicadores.
- /cmd_vel_ruta: 1 suscriptor, /supervisor_f13.

Sensor frontal:
- Tipo: sensor_msgs/msg/Range.
- Marco: base_scan.
- Campo de vision: 0.261799395 rad.
- Alcance maximo: 2.0 m.
- Lectura observada: 2.0 m.

Orden observada sin ruta activa:
- Velocidad lineal: 0.0 m/s.
- Velocidad angular: 0.0 rad/s.
- Las demas componentes de Twist: 0.0.

## Resultado

EX-02: satisfactorio para el arranque integrado
observado y la parada sin propuestas de movimiento.

La evidencia corresponde a una ejecucion.
No constituye certificacion de seguridad fisica
ni una prueba de estabilidad prolongada.

No se repitieron ensayos de navegacion ni de seguridad.
