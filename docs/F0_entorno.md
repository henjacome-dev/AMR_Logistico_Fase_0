# F0.2 — Instalación y verificación tecnológica

## 1. Ruta recomendada
- Mantener **Ubuntu 22.04 + ROS 2 Humble** como exige el documento.
- Si el equipo usa Windows 11, WSL2 con GUI (WSLg) puede funcionar, pero es obligatorio verificar la aceleración gráfica; si usa VMware, validar gráficos 3D y memoria suficiente. Ubuntu nativo es una opción directa.
- **No cambiar a ROS 2 Jazzy ni mezclar paquetes de Gazebo Classic y Gazebo nuevo** sin acordar una migración de equipo. La documentación clásica de TurtleBot3 Humble usa Gazebo Classic, que ya no recibe soporte.

## 2. Verificación previa — sin instalar nada
```bash
lsb_release -ds
uname -m
python3 --version
which ros2 || true
test -f /opt/ros/humble/setup.bash && echo 'ROS HUMBLE disponible'
```
Ejecutar `bash tools/diagnostico_f0.sh` desde la carpeta raíz de este paquete. Guardar la salida en `resultados/f0_diagnostico.txt` si se desea.

## 3. Si todavía no se instaló ROS 2 Humble
Seguir el procedimiento oficial de repositorios y dependencias para **Ubuntu 22.04**, no instalar simplemente paquetes `ros-humble-*` sin configurar antes el repositorio oficial:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Tras agregar el repositorio como indica la guía oficial:
```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions git
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```
La última línea debe devolver `humble`.

## 4. Paquetes TurtleBot3 y simulación para Humble (ejemplo de fuente oficial)
Instalar la integración clásica con Gazebo según el manual de TurtleBot3:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs python3-rosdep
source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Si existen problemas de dependencias durante `colcon build`, resolverlas con `rosdep` según la guía oficial antes de modificar el código. Si faltan aceleración gráfica o plugins, no asumir que el software está correcto.

Guía oficial de simulación: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

## 5. Pruebas con simulación ejecutándose
En una SEGUNDA terminal, tras source de ROS y `~/turtlebot3_ws/install/setup.bash`:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
```
En una TERCERA terminal:
```bash
ros2 topic list -t
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /camera/image_raw --once
```
Nota: una imagen es un mensaje voluminoso; para verificar la cámara puede ser preferible `ros2 topic hz /camera/image_raw` o `rqt_image_view`. La disponibilidad de cámara depende del modelo y plugins cargados: si no aparece, es un fallo por resolver, no evidencia de éxito.

## 6. MATLAB
Verificar manualmente las licencias de MATLAB, Robotics System Toolbox y Fuzzy Logic Toolbox. En MATLAB:
```matlab
ver
which binaryOccupancyMap
which plannerAStarGrid
which mamfis
```

## 7. Evidencias mínimas
Captura del robot moviéndose, versión de sistema, confirmación de ROS Humble, listado de tópicos `/odom`, `/scan`, `/camera/image_raw`, y nota sobre estado de MATLAB.
