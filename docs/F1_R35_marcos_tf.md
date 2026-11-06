# R35 — Validación de coordenadas y marcos TF

Fecha: 2026-11-06
Entorno: ROS 2 Humble, Gazebo Classic, TurtleBot3 Waffle Pi.

## Objetivo

Comprobar la coherencia entre SDF, mapa de ocupación,
waypoints A*, odometría y transformaciones TF.

## Validación estática

- Mapa: 200 x 160 celdas.
- Resolución: 0.05 m.
- Origen: (-5.0, -4.0, 0.0).
- Cuatro estaciones coherentes con SDF y YAML.
- Dos obstáculos coherentes con SDF y YAML.
- CSV: cabeceras x_m,y_m.
- Giro positivo: antihorario.
- Cuatro rutas y tres órdenes A*: correctas.
- Sin celdas ocupadas ni cortes diagonales.

## Verificación ROS 2

Posición inicial configurada:
(-3.000, -2.800) m.

Posición observada en /odom:
(-3.000384, -2.799999) m.

Marcos observados:
- header.frame_id: odom.
- child_frame_id: base_footprint.

La transformación odom -> base_footprint
se observó operativa.

Inicialmente no se publicaba map -> odom.

## Integración TF

Se añadió al archivo
launch/almacen_operativo_f1.launch.py
un publicador estático map -> odom.

Transformación configurada:
- Traslación: (0, 0, 0) m.
- Rotación: (0, 0, 0, 1).

Se comprobó:
- Nodo /map_odom_static_f1 activo.
- map -> odom disponible.
- map -> base_footprint disponible.
- Robot en aproximadamente (-3.001, -2.800) m.

Las pruebas fueron realizadas con el robot detenido.

## Limitaciones

La identidad map -> odom se utiliza como hipótesis
de referencia fija de esta simulación.

No demuestra localización global, corrección
de deriva odométrica ni precisión durante
recorridos prolongados.

No debe ejecutarse simultáneamente otro emisor
de map -> odom. Si se integra un localizador,
esta transformación estática debe retirarse.

El marco world corresponde al escenario Gazebo;
su coherencia con el mapa se comprobó
geométricamente. No se afirma la existencia
de una transformación TF world -> map.

## Evidencias

- tools/verificar_coordenadas_r35.py
- resultados/R35_validacion_estatica.txt
- resultados/R35_odom_pose.txt
- resultados/R35_tf_integrado_map_odom.txt
- resultados/R35_tf_integrado_map_robot.txt
- launch/almacen_operativo_f1.launch.py

## Dictamen

R35: Verificado para la configuración
de simulación con referencia fija.
