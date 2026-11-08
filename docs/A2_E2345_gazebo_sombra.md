# Avance 2 — E2.3.4 y E2.3.5: ensayos Gazebo en modo sombra

**Fecha de captura:** 2026-11-07, hora local de Ecuador.
**Stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, TurtleBot3 Waffle Pi.
**Estado:** validación de comunicación y conmutación en sombra aprobada; R19 operativo pendiente.

## E2.3.4 — Comunicación sin movimiento

- Gazebo proporciona `/scan` y `/odom`.
- El emulador publica `/rango_frontal`, `/rango_izq` y `/rango_der`.
- Mamdani publica `/cmd_vel_difuso` y `/estado_difuso`.
- El candidato de supervisor publica `/cmd_vel_integrado_prueba` y `/estado_integrado_prueba`.
- Se observaron datos de nueve tópicos; no se inició seguidor de ruta.
- El nodo difuso alcanzó `ACTIVO`, pero el candidato mantuvo `DETENIDO` al faltar propuestas de ruta.
- Dominio ROS 223, solo loopback. No hubo publicadores de `/cmd_vel`.

## E2.3.5 — Transiciones con cajas reales de Gazebo

| Fase | LiDAR frontal observado | Decisión | Propuesta integrada (v, omega) |
|---|---:|---|---|
| Pasillo libre | 2,5728 m | RUTA | (0, 0) |
| Caja intermedia | 0,5205 m | DIFUSO | (0,1200 m/s, +0,1697 rad/s) |
| Caja cercana | 0,2989 m | DETENIDO | (0, 0) |
| Retirar caja cercana | 0,5174 m | DIFUSO | (0,0134 m/s, +0,7000 rad/s) |
| Retirar caja intermedia | 2,5440 m | RUTA | (0, 0) |

Las propuestas no nulas se emitieron **solamente en** `/cmd_vel_integrado_prueba`,
que no acciona el TurtleBot3. La entrada `/cmd_vel_ruta` recibió solo `Twist()` cero.
Dominio ROS 224; las entidades temporales se eliminaron al finalizar.

## Evidencias y reproducción

- `resultados/A2_E234_gazebo_sombra/evidencia_publicada/`: resumen, CSV y registros de observación.
- `resultados/A2_E235_gazebo_transiciones_sombra/evidencia_publicada/`: resumen, CSV y registros de observación.
- `tools/A2_E234_Gazebo_sombra_seguro.py` y `tools/A2_E235_Gazebo_transiciones_sombra_v3.py`
  preservan exactamente el código utilizado en las pruebas.
- `python3 tools/A2_E2345_validar_evidencias_gazebo.py` comprueba los informes y
  el origen del código; **no reproduce ROS ni Gazebo dentro de GitHub Actions**.

Los dos scripts de ejecución fijan como condición el commit inicial
`30fe276b594cdcdd5f2b5d2a81c0f1a265422e4e`. Para repetir exactamente
el protocolo deben ejecutarse en un árbol de trabajo que cumpla dicha condición;
su incorporación documental en un commit posterior no cambia esta limitación.

## Restricciones para R19

- Los umbrales **DIFUSO ≤0,60 m / RUTA ≥0,80 m** son provisionales. No se han
  acreditado como valores oficiales del enunciado.
- La barrera de parada a 0,35 m y la saturación de velocidades funcionaron en
  sombra; **no constituyen validación de navegación con movimiento**.
- Aún falta establecer un único publicador de `/cmd_vel` bajo lanzamiento
  operativo controlado, probar movimiento y detenerse de forma fiable en Gazebo.
- No se declara evitación de colisiones ni seguridad certificada en hardware.
