# E2.2 — Nodo de propuestas difusas ROS 2: validación aislada

**Fecha de prueba:** 7 de noviembre de 2026 (hora de Ecuador)
**Base de referencia:** R18 aprobado; controlador `AMR_Mamdani_A2_v01.fis` inmutable.
**Estado:** prueba aislada en ROS 2 Humble aprobada; integración operativa pendiente (E2.3).

## Arquitectura

- Entradas: `/rango_frontal`, `/rango_izq`, `/rango_der` (`sensor_msgs/msg/Range`).
- Motor: `scikit-fuzzy` reconstruido desde el FIS validado de 27 reglas mediante el módulo de R18.
- Salidas exclusivas: `/cmd_vel_difuso` (`geometry_msgs/msg/Twist`) y `/estado_difuso` (`std_msgs/msg/String`).
- **No** publica en `/cmd_vel` ni modifica `tools/supervisor_f13.py`.
- Frecuencia experimental: 5 Hz. Caducidad: 0,8 s desde recepción (tiempo monotónico).
- Ante rango no finito, fuera de [0,2] m, ausente o caducado: propuesta `Twist()` cero.

## Evidencia ejecutada en Ubuntu/WSL2

La prueba `tools/A2_E22_prueba_ROS_aislada.py` activó ROS 2 Humble y un
`ROS_DOMAIN_ID` aislado, sin Gazebo. Los resultados observados fueron:

| Comprobación | Resultado |
|---|---|
| Nodo e interfaces ROS descubiertas | Aprobado |
| Sin datos de sensores | Estado `SIN_DATO_df`, propuesta cero |
| Tres `Range` válidos (1,7 m) | Estado `ACTIVO`, v aproximadamente 0,16111 m/s |
| Datos caducados | Propuesta cero |
| Recuperación tras lecturas nuevas | Aprobado |
| `NaN` frontal | `INVALIDO_df`, propuesta cero |
| Emisiones en `/cmd_vel` | Ninguna en el dominio aislado |

Archivos: `resultados/A2_E22_prueba_ros_aislada/evidencia_publicada/resumen.json`
y `nodo_ros.log`. El CSV de R18 y sus errores siguen documentados por separado.

## Reproducción

Desde Ubuntu con ROS 2 Humble y el entorno virtual R18 instalado:

```bash
cd ~/AMR_Logistico_Fase_0
source tools/activar_entorno.sh
~/.venvs/amr-a2-r18/bin/python tools/A2_E22_nodo_difuso.py --self-test
python3 tools/A2_E22_prueba_ROS_aislada.py
```

El segundo comando configura y usa un dominio ROS 2 separado para no
interferir con una simulación activa. Se ejecuta sobre un entorno Ubuntu
que tenga ROS 2 y `rclpy`; no forma parte de GitHub Actions.

GitHub Actions solo repite la **autoprueba del motor sin ROS** y comprueba
la estructura de la evidencia ROS ya generada. No demuestra ejecución
ROS 2 en el runner ni seguridad operacional.

## Restricciones de alcance

- No hay arbitraje ruta/difuso todavía. El supervisor sigue consumiendo
  `/cmd_vel_ruta`, no `/cmd_vel_difuso`.
- El Mamdani puede entregar velocidad positiva ante tres obstáculos cercanos;
  E2.3 debe imponer parada estricta independiente del resultado de inferencia.
- E2.2 no acredita navegación sin colisiones, selección automática de modo
  ni pruebas dinámicas en Gazebo.
