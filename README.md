# Proyecto AMR logístico en almacén — MEC451

**Institución:** Universidad ECOTEC · Ingeniería Mecatrónica · Autómatas y Expertos  
**Referencia:** Enunciado «Proyecto 1 — Robot móvil autónomo para logística en almacén»  
**Objetivo:** navegación A*, evasión difusa y reconocimiento visual CNN, inicialmente en ROS 2/Gazebo y luego en ESP32.

## Alcance y fechas
- 4 nov. 2026: Avance 1 (20 %): mundo Gazebo, mapa, A*, seguimiento y reporte.
- 9 nov. 2026: Avance 2 (35 %): controlador difuso, evaluación y adelanto CNN.
- 16 nov. 2026: final de simulación (45 %): CNN e integración completa.
- Semanas 13–16: fase física ESP32, evaluación independiente.

## Archivos de Fase 0
- `docs/F0_requisitos.md`: matriz inicial de requisitos verificables.
- `docs/F0_entorno.md`: instalación y verificación del entorno según configuración.
- `docs/F0_arquitectura.md`: arquitectura y contratos de interfaces.
- `docs/F0_validacion.md`: criterios de cierre y registro de evidencias.
- `docs/F0_roles.md`: asignación sugerida de responsabilidades.
- `docs/F0_analisis_problema.md`: borrador técnico preliminar aprovechable en el Avance 1.
- `config/interfaces.yaml`: nombres de tópicos y parámetros compartidos. **Los añadidos no definidos por el docente son propuestas.**
- `tools/diagnostico_f0.sh`: inspección de ROS y paquetes, sin instalar ni alterar el sistema.

## Estructura oficial del repositorio
`matlab/`, `ros2_ws/src/`, `modelos/`, `datos/`, `resultados/`, `firmware/`.
Se agregan carpetas auxiliares `docs/`, `config/` y `tools/` para organización.

## Uso rápido (Ubuntu 22.04)
Desde la raíz del proyecto:
```bash
bash tools/diagnostico_f0.sh
```
Una vez instalado ROS y TurtleBot3, en una terminal con los entornos cargados:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
En otra terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```
Si el paquete de simulación se instala en otra área de trabajo, cambie la ruta del `setup.bash`.

## Convenciones obligatorias
- `/cmd_vel`: `geometry_msgs/msg/Twist`, giro positivo a izquierda (REP-103).
- `v_max = 0.20 m/s`, `|omega_max| = 1.5 rad/s`.
- Radio de inflado: `0.21 m`; resolución de mapa: `0.05 m` o más fina.
- LiDAR se reduce a 3 sectores de 15° (`0°`, `+45°`, `−45°`), alcance de 2 m, ruido gaussiano `sigma=0.01 m`.

**Estado actual (2026-11-04):** Ubuntu 22.04 sobre WSL2, ROS 2 Humble, Gazebo Classic y TurtleBot3 Waffle Pi verificados localmente. Odometria, LiDAR, camara y teleoperacion comprobados. Repositorio privado en GitHub sincronizado. Arquitectura F0.4 validada estaticamente. Algoritmos autonomos y MATLAB pendientes de validacion.

## Entorno robotico validado en Fase 0

Activar el entorno en una terminal Ubuntu:

```bash
source ~/AMR_Logistico_Fase_0/tools/activar_entorno.sh
```

Ejecutar la simulacion del escenario oficial:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Verificar topicos desde otra terminal con el entorno activado:

```bash
ros2 topic list
ros2 topic hz /scan
```

Los nodos autonomos del proyecto aun no estan implementados.
Las interfaces F0.4 representan contratos de diseno.
