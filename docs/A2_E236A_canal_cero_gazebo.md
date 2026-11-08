# Avance 2 — E2.3.6A: canal /cmd_vel solo con velocidad cero

**Fecha de ensayo:** 2026-11-07, hora de Ecuador.
**Entorno:** WSL2 Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11,
TurtleBot3 Waffle Pi; servidor sin interfaz grafica (*headless*).
**Estado:** ensayo local aprobado; R19 operativo no validado.

## Metodologia y seguridad

- Dominio ROS 224, comunicacion local, sin supervisor operativo F1.3 ni seguidor de rutas.
- Servidor `gzserver` con `libgazebo_ros_init.so` y `libgazebo_ros_factory.so`;
  comprobacion de `/spawn_entity` ANTES de insertar `waffle_pi`.
- Un unico publicador de `/cmd_vel` en el monitor del ensayo:
  **solo `Twist()` completamente cero**, con temporizador de 0,10 s.
- El candidato arbitra RUTA/DIFUSO y publica SOLO `/cmd_vel_integrado_prueba`.
  Las propuestas no nulas no llegaron al robot.
- Propuesta `/cmd_vel_ruta` igualmente cero en todo momento.
- Se observan los diez topicos: `scan`, `odom`, tres `Range`, propuesta y estado
  difusos, propuesta y estado del candidato, y `/cmd_vel`.

## Transiciones medidas en Gazebo

| Etapa | LiDAR frontal (m) | Modo | Propuesta del candidato |
|---|---:|---|---|
| Pasillo libre | 2,5655 | RUTA | (0, 0) |
| Caja intermedia | 0,5106 | DIFUSO | (0,0496 m/s, +0,7000 rad/s) |
| Caja cercana | 0,2961 | DETENIDO | (0, 0) |
| Retirada caja cercana | 0,5102 | DIFUSO | (0,0474 m/s, +0,7000 rad/s) |
| Pasillo despejado | 2,5571 | RUTA | (0, 0) |

Esas propuestas se observaron solamente en el topico de prueba; **el robot
no recibio propuestas distintas de cero**.

## Canal de actuacion

- 110 comandos `/cmd_vel` enviados; 110 recibidos; todos con sus seis componentes cero.
- Frecuencia media calculada: 9,39 Hz (intervalo aceptable 8–12 Hz).
- Exclusividad: un publicador de `/cmd_vel` durante el ensayo.
- Desplazamiento inicial-final por `/odom`: inferior a 0,05 m.
- Las cajas de Gazebo fueron creadas y eliminadas mediante servicios de simulacion.

## Evidencias y reproducibilidad

- Codigo ensayado: `tools/A2_E236A_Gazebo_canal_cmd_vel_cero_v5_headless.py`, SHA256 `9589b7f32cdfe4c3c81b5d33accd378ac8395aa13104ff28ec91c1805bdd1869`.
- Captura: `resultados/A2_E236A_gazebo_headless_canal_cero/evidencia_publicada/` con resumen JSON, CSV y cuatro registros.
- Verificacion estatica: `python3 tools/A2_E236A_validar_evidencia_canal_cero.py`.
- Se conservan en Ubuntu los intentos anteriores fallidos y el informe
  de diagnostico ROS Factory; no se confunden con el ensayo final aprobado.
- El script fuente fija el SHA anterior `8c893f13710713be5ea35ef226952bf10a91d6d9` como precondicion para
  ejecutar exactamente el protocolo: tras publicarlo en GitHub, la prueba
  **no puede relanzarse sin adaptar conscientemente esa precondicion**.
- GitHub Actions comprueba la integridad y coherencia de las evidencias; no
  ejecuta Gazebo ni demuestra seguridad fisica.

## Pendientes de R19

1. Validar contra el enunciado los umbrales 0,60/0,80 m (provisionales).
2. Incorporar un unico supervisor operativo responsable de `/cmd_vel`.
3. Verificar movimientos no nulos seguros, paradas reales en Gazebo, tiempos
   de reaccion y logs de conmutacion.
4. Evaluar R20: cuatro escenarios de navegacion por cinco repeticiones.

**No se autoriza movimiento por esta evidencia ni se declara R19 cerrado.**
