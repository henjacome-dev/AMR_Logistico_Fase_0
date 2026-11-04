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
