#!/usr/bin/env bash

# Entorno base: ROS 2 Humble
source /opt/ros/humble/setup.bash

# Paquetes oficiales TurtleBot3
source "$HOME/turtlebot3_ws/install/setup.bash"

# Paquetes propios del proyecto AMR (cuando estén compilados)
if [ -f "$HOME/AMR_Logistico_Fase_0/ros2_ws/install/local_setup.bash" ]; then
    source "$HOME/AMR_Logistico_Fase_0/ros2_ws/install/local_setup.bash"
fi

# Modelo de robot
export TURTLEBOT3_MODEL=waffle_pi

echo "Entorno AMR activado"
echo "ROS 2: $ROS_DISTRO"
echo "Robot: $TURTLEBOT3_MODEL"

# R05 - Recursos OGRE de paneles con texturas para Gazebo Classic.
_AMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GAZEBO_MODEL_PATH="${_AMR_ROOT}/modelos${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
unset _AMR_ROOT
