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


## H. Contratos preliminares de comunicación

Estado al 2026-11-05: arquitectura parcialmente implementada; contratos restantes pendientes de integración.

### Responsabilidades
- emulador_sensores: convierte /scan en tres distancias sectoriales.
- seguidor_ruta: utiliza /odom y waypoints CSV para proponer velocidades.
- controlador_difuso: utiliza distancias para proponer velocidades de evasión.
- reconocedor_estacion: analiza imágenes y notifica estaciones detectadas.
- supervisor: selecciona comandos, aplica límites y gestiona paradas.

### Contratos de tópicos
| Tópico | Publicador | Consumidor | Tipo |
|---|---|---|---|
| /scan | Gazebo/TurtleBot3 | emulador_sensores | sensor_msgs/msg/LaserScan |
| /odom | TurtleBot3 | seguidor_ruta | nav_msgs/msg/Odometry |
| /camera/image_raw | Gazebo/TurtleBot3 | reconocedor_estacion | sensor_msgs/msg/Image |
| /rango_frontal | emulador_sensores | controlador_difuso, supervisor | sensor_msgs/msg/Range |
| /rango_izq | emulador_sensores | controlador_difuso, supervisor | sensor_msgs/msg/Range |
| /rango_der | emulador_sensores | controlador_difuso, supervisor | sensor_msgs/msg/Range |
| /cmd_vel_ruta | seguidor_ruta | supervisor | geometry_msgs/msg/Twist |
| /cmd_vel_difuso | controlador_difuso | supervisor | geometry_msgs/msg/Twist |
| /estacion_detectada | reconocedor_estacion | supervisor | std_msgs/msg/String (propuesto) |
| /cmd_vel | supervisor | TurtleBot3 | geometry_msgs/msg/Twist |

### Reglas de integración
1. Durante la operación autónoma, solo supervisor publica /cmd_vel.
2. No ejecutar simultáneamente teleoperación directa y supervisor.
3. Los controladores internos publican propuestas, no órdenes finales.
4. Velocidad lineal máxima del proyecto: 0.20 m/s.
5. Velocidad angular máxima del proyecto: 1.5 rad/s.
6. Giro positivo: antihorario, conforme a REP-103.
7. Sin entradas válidas o ante una parada solicitada, el supervisor
   debe publicar velocidad cero.
8. Todos los nodos simulados deben utilizar el mismo reloj de simulación.
9. La ruta CSV utilizará x_m,y_m en el marco map; origen y
   transformaciones pendientes de definición en F1.2.
10. Los umbrales, tiempos de espera y criterios de replanificación
    requieren validación antes de implementar el supervisor.

### Restricciones de alcance
- Los contratos internos son propuestas de diseño.
- Los tópicos internos están parcialmente implementados; los restantes requieren integración y verificación.
- Este documento no sustituye config/interfaces.yaml.
- Los parámetros definitivos se consolidarán tras su revisión.

## I. Auditoria de implementacion — 2026-11-05

| Modulo | Estado |
|---|---|
| Gazebo y TurtleBot3 Waffle Pi | Operativos |
| Mapa y planificacion A* | Implementados con evidencias |
| Seguidor proporcional | Implementado y validado |
| Supervisor de seguridad F1.3.3 | Validado en simulacion |
| Emulador de tres sensores Range | Implementado; R07/R08 validados con LiDAR sintético |
| Controlador difuso integrado | Pendiente |
| Reconocedor visual CNN | Pendiente |

La validacion del supervisor F1.3.3 corresponde a parada,
limitacion de velocidad y recuperacion mediante nuevas propuestas.
No demuestra todavia arbitraje completo entre ruta y controlador difuso.

## J. Estado operativo — 2026-11-06

Los contratos de las secciones precedentes incluyen
interfaces propuestas para etapas posteriores.

Implementado y comprobado en F1:
- LiDAR y cámara del robot simulado.
- Emulador /scan -> tres tópicos Range.
- Seguidor proporcional basado en CSV y /odom.
- Supervisor F1.3 que consume /cmd_vel_ruta,
  /scan y /odom y publica /cmd_vel.
- Parada preventiva ante obstáculo o datos inválidos.

No implementado todavía:
- Arbitraje efectivo entre ruta y controlador difuso.
- Publicación del controlador en /cmd_vel_difuso.
- Detección CNN y /estacion_detectada.
- Replanificación autónoma integrada.

El supervisor F1.3 utiliza /scan directamente;
no consume los tres tópicos /rango_*.
El contrato A2 del supervisor no debe confundirse
con el comportamiento actual del prototipo.

La alineación geométrica mundo-mapa fue verificada
estáticamente. R35 conserva validación parcial
de transformaciones y convenciones operativas.
