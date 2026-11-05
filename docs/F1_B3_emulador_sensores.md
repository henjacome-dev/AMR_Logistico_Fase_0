# B3 — Emulador de tres sensores virtuales

Fecha: 2026-11-05
Entorno: ROS 2 Humble, Gazebo Classic, TurtleBot3 Waffle Pi.

## Implementacion

Archivo: tools/emulador_sensores_f1.py

Entrada:
- /scan — sensor_msgs/msg/LaserScan

Salidas:
- /rango_frontal — sensor_msgs/msg/Range
- /rango_izq — sensor_msgs/msg/Range
- /rango_der — sensor_msgs/msg/Range

Parametros implementados:
- Centros: 0, +45 y -45 grados.
- Ancho sectorial: 15 grados.
- Alcance maximo: 2.0 m.
- Ruido gaussiano: sigma = 0.01 m.
- El ruido no se aplica a lecturas saturadas a 2.0 m.

## Resultados observados en ROS 2

| Sensor | Distancia (m) |
|---|---:|
| Frontal | 2.0000 |
| Izquierdo | 1.4687 |
| Derecho | 1.3059 |

Los tres topicos publicaron mensajes Range.
Campo de vision observado: 0.261799395 rad.
Alcance maximo publicado: 2.0 m.
Marco de referencia: base_scan.

## Conclusion

Validada la publicacion de las tres interfaces ROS 2.
Los centros sectoriales y el ruido se verificaron en el codigo.
Las muestras individuales no acreditan estadisticamente
la distribucion de ruido ni prueban todas las orientaciones.
El nodo no genera comandos de movimiento.
