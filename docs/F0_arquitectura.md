# F0.4 — Arquitectura de nodos e interfaces

## A. Nodos exigidos en el enunciado
- `emulador_sensores`: se suscribe `/scan`, produce tres tópicos `sensor_msgs/msg/Range`.
- `seguidor_ruta`: recibe puntos de ruta (CSV) y odometría, calcula `v, ω`.
- `controlador_difuso`: consume tres distancias, calcula `v, ω` con scikit-fuzzy (parámetros del .fis).
- `reconocedor_estacion`: procesa imágenes de cámara y publica estación detectada.
- `supervisor`: alterna seguidores, verifica destino y determina finalización o replanificación.

## B. Contratos especificados por el documento
| Tópico/dato | Tipo | Emisor | Consumidor | Estado |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | TurtleBot3/Gazebo | `emulador_sensores` | Especificado |
| `/rango_frontal` | `sensor_msgs/msg/Range` | `emulador_sensores` (luego ESP32) | difuso/supervisor | Especificado |
| `/rango_izq` | `sensor_msgs/msg/Range` | ídem | difuso/supervisor | Especificado |
| `/rango_der` | `sensor_msgs/msg/Range` | ídem | difuso/supervisor | Especificado |
| `/odom` | `nav_msgs/msg/Odometry` | Robot simulado | `seguidor_ruta` | Especificado |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Cámara simulado | `reconocedor_estacion` | Especificado; verificar disponibilidad real |
| `/estacion_detectada` | Tipo no fijado por enunciado | `reconocedor_estacion` | `supervisor` | Nombre especificado; tipo propuesto `std_msgs/msg/String` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | control de movimiento activo | TurtleBot3 | Especificado |
| Lista de ruta CSV | columnas no fijadas | MATLAB A* | `seguidor_ruta` | Propuesta: `x_m,y_m` en marco `map` |

## C. Interfaces PROPUESTAS (no exigencias oficiales)
- `/cmd_vel_ruta` (`geometry_msgs/msg/Twist`): candidato de seguidor.
- `/cmd_vel_difuso` (`geometry_msgs/msg/Twist`): candidato de evasión.
- `supervisor` es único publicador autónomo de `/cmd_vel`; durante teleoperación de prueba se detiene el supervisor.
- `/destino_solicitado` (`std_msgs/msg/String`): un identificador `RECEPCION|ALMACENAMIENTO|EMPAQUE|DESPACHO`.
- `ruta_*.csv`: cabecera `x_m,y_m`, sistema de coordenadas `map`; origen y ejes deben fijarse en F1.2.
- Publicación de `estacion_detectada` como nombre de estación y, de ser necesario, tópico opcional de confianza (`std_msgs/msg/Float32`).

## D. Supervisor y seguridad
- Sin obstáculo: si `d_f > 0.60` y `d_i,d_d > 0.35`, seleccionar `seguidor_ruta`.
- Evasión: si `d_f <= 0.60` o alguna lateral `<= 0.35`, seleccionar `controlador_difuso`.
- Cuando desaparece el obstáculo, reenganchar al punto de ruta más cercano, cuidando evitar oscilación entre estados.
- Considerar tiempo máximo sin datos y parada segura como medidas de ingeniería propuestas.

## E. Unidades y orientación
- `v`: m/s, rango [0, 0.20].
- `omega`: rad/s, rango [-1.5, 1.5]. Positiva = giro hacia izquierda; negativa = derecha, REP-103.
- Distancias: m; 0–2; sectores angostos de LiDAR con centros 0°, +45°, -45° y ancho 15°; min cada sector, límite 2 m, ruido gaussiano sigma 0.01 m.
- `use_sim_time` debe ser coherente entre nodos simulados.

## F. Advertencia: controladores de materiales previos
Un material anterior de clase emplea signo negativo para izquierda; es incompatible con el signo oficial de este proyecto. Adaptar reglas y pruebas para REP-103, no reutilizar sin conversión.

## G. Flujo
```
mapa_manual -> MATLAB A* -> puntos CSV -> seguidor_ruta -> /cmd_vel_ruta --+
/scan -> emulador_sensores -> /rango_* -> controlador_difuso -> /cmd_vel_difuso --+--> supervisor -> /cmd_vel
/odom ---------------------------------------------------------> seguidor_ruta
/camera/image_raw -> reconocedor_estacion -> /estacion_detectada -> supervisor
orden de origen/destino ----------------------------------------------> supervisor
```
