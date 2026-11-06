# Cierre técnico de las Etapas 0 y 1

Fecha: 2026-11-06.
Entorno: Ubuntu 22.04, ROS 2 Humble,
Gazebo Classic y TurtleBot3 Waffle Pi.

## A. Navegación integral

Orden A ejecutada sobre la geometría R04.

- Ruta A*: 20.200 m; 405 waypoints.
- Recorrido odométrico: 20.022 m.
- Duración: 275.03 s.
- Estaciones visitadas: 4/4.
- Error respecto al waypoint final: 0.0711 m.
- Velocidad lineal máxima: 0.075 m/s.
- Velocidad angular máxima: 0.700 rad/s.
- Registro ROS: 16937 mensajes.

No se observaron colisiones durante la ejecución.
No se realizó una prueba independiente con
sensores de contacto.

Evidencias:
- resultados/F1_A_cierre_20261106/
  seguimiento_orden_A.csv
- resultados/F1_A_cierre_20261106/
  analisis_rosbag.txt
- resultados/F1_A_cierre_20261106/rosbag/

## B. Sensores virtuales

Prueba dinámica usando la misma navegación:

- Aproximación a pared oeste: 0.852 m.
- Disminución del Range frontal: 0.842 m.
- Diferencia entre ambas magnitudes: 0.010 m.
- Tres sensores Range registrados en movimiento.
- 1374 lecturas por sensor durante el recorrido.

Resultado: satisfactorio en simulación.

Evidencia:
resultados/F1_A_cierre_20261106/
actividad_B_pared_oeste.txt

## C. Visualización RViz2

Se verificaron:
- Marco fijo map.
- Odometry /odom: Status Ok.
- LaserScan /scan: Status Ok.
- Cámara con imagen del almacén visible.

Evidencias:
resultados/F1_C_rviz_20261106/

## D. Comparación A* MATLAB-Python

Se utilizó el mismo mapa de ocupación inflado
exportado por MATLAB para ambos planificadores.

Diferencias de longitud:
- Orden A: 0.498 %.
- Orden B: 0.665 %.
- Orden C: 0.383 %.

La orden A fue la más corta entre las tres
alternativas evaluadas.

Evidencias:
resultados/F1_D_cierre_20261106/

## E. Organización y cierre

Responsables:
- Ángel Rodríguez: integración ROS 2 y Gazebo.
- Henry Jácome: IA clásica y documentación.
- Luis Lluiguipuma: lógica difusa.
- Francisco Freire: redes neuronales.

Confirmación organizativa — 2026-11-06:
- Los cuatro integrantes confirman acceso a GitHub.
- Arquitectura e interfaces aprobadas por el grupo.
- Información comunicada por Henry Jácome;
  no constituye auditoría independiente de permisos.

## Alcance del dictamen

Las actividades A-D están técnicamente verificadas.

Calificación de la rúbrica técnica interna: 100/100.
No representa la calificación oficial del docente.

Las mejoras posteriores al 4 de noviembre de 2026
no alteran retroactivamente el corte oficial de
entrega del Avance 1.

Este documento no acredita todavía los requisitos
del Avance 2 ni de la entrega final.

## Conservación de evidencia ROS

El ROS bag completo se conserva localmente y no se
incluye en Git debido a la política de archivos
binarios del repositorio (.gitignore).

SHA-256 del archivo rosbag_0.db3:
`139df55b084c169f1b49cc149a0fc211c709d79e8f5f131249468c89998b1184`
