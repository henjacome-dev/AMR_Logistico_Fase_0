# Avance 2 — E2.3: preintegración de arbitraje y supervisor (R19 parcial)

**Fecha de ensayos:** 7 de noviembre de 2026, hora de Ecuador.
**Entorno:** WSL2 Ubuntu 22.04, ROS 2 Humble.
**Estado:** E2.3.1–E2.3.3 aprobadas en aislamiento. Integración operativa R19 pendiente.

## Pruebas realizadas

| Etapa | Validación | Resultado |
|---|---|---|
| E2.3.1 | Contrato puro RUTA/DIFUSO, histéresis y saturación | 17/17 |
| E2.3.2 | `Range`, `Odometry`, `Twist` por ROS 2 en dominio aislado | 14/14 |
| E2.3.3 | Candidato con barrera LiDAR independiente y parada estricta | 20/20 |

**Total:** 51 verificaciones de banco, no 51 pruebas de navegación en Gazebo.

## Interfaces

- Entradas: `/rango_frontal`, `/rango_izq`, `/rango_der`, `/scan`, `/odom`, `/cmd_vel_ruta` y `/cmd_vel_difuso`.
- E2.3.2 solo publica `/cmd_vel_arbitraje_sombra` y `/estado_arbitraje_sombra`.
- E2.3.3 solo publica `/cmd_vel_integrado_prueba` y `/estado_integrado_prueba`.
- **Ningún banco publica en `/cmd_vel`; el supervisor operativo F1.3 permanece intacto.**

## Configuración de prueba (NO valores definitivos del enunciado)

- Parada estricta si distancia frontal `Range <=0,35 m` o distancia frontal LiDAR `<=0,35 m`.
- Conmutar de RUTA a DIFUSO con distancia `<=0,60 m`.
- Mantener DIFUSO en la banda intermedia; retornar a RUTA con `>=0,80 m`.
- Limitar velocidad a `0,12 m/s` y giro a `±0,70 rad/s` por supervisor.
- Caducidad de rangos: `0,80 s`; odometría: `1,50 s`; propuestas: `0,40 s`.
- Caducidad de LiDAR en el candidato: `1,50 s`; los tiempos usan recepción monotónica.
- Bloqueo por sensor ausente, no finito, caducado, odometría inválida, comando no autorizado.

Una prueba clave: LiDAR detectó obstáculo a 0,30 m mientras Range indicaba 1,70 m; el candidato publicó propuesta cero (`PARADA_SCAN_ESTRICTA`).
La salida del Mamdani por sí sola no garantiza parada a velocidad cero.

## Evidencia registrada

- `resultados/A2_E23_arbitraje_sombra/evidencia_publicada/`: resumen JSON y CSV (17 casos).
- `resultados/A2_E232_arbitraje_ros_sombra/evidencia_publicada/`: resumen JSON, observaciones CSV y log ROS (14 casos).
- `resultados/A2_E233_supervisor_integrado_aislado/evidencia_publicada/`: resumen JSON, observaciones CSV y log ROS (20 casos).

Reproducción de E2.3.1 (sin ROS):

```bash
python3 tools/A2_E23_arbitraje_sombra.py --self-test
```

Los ensayos E2.3.2–E2.3.3 requieren WSL2 y ROS 2 Humble con `rclpy` y el entorno R18; se ejecutaron con datos sintéticos en dominios aislados.
**GitHub Actions no ejecuta ROS 2 ni Gazebo:** repite E2.3.1 y comprueba los informes ya capturados.

## Pendientes para R19 operativo

1. Verificar en el documento oficial los umbrales de activación del control difuso; `0,60/0,80` son solo propuestas.
2. Integrar el arbitraje con un único publicador de `/cmd_vel` bajo lanzamiento controlado.
3. Ensayar en Gazebo, registrar transiciones y validar detenciones con datos reales simulados.
4. Revalidar los límites y la seguridad operativa antes de iniciar R20 (4 escenarios por 5 repeticiones).

No se ha demostrado seguridad física, evitación de colisiones ni operación autónoma completa.
