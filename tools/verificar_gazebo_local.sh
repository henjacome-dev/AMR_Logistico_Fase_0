#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1
duracion=125s
if [[ "${1:-}" == --mision ]]; then
    export ROS_DOMAIN_ID=173
    duracion=210s
elif (( $# != 0 )); then
    printf 'Uso: bash tools/verificar_gazebo_local.sh [--mision]\n' >&2
    exit 2
fi
export GAZEBO_MODEL_PATH="$PWD/modelos:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MASTER_URI=http://127.0.0.1:11374

evidencia=$(mktemp -d "$PWD/resultados/verificacion_gazebo_XXXXXX")
timeout --signal=INT --kill-after=10s "$duracion" ros2 launch ./launch/almacen_f1_optica_v6.launch.py gui:=false \
    >"$evidencia/gazebo.log" 2>&1 &
simulacion=$!
trap 'kill -INT "$simulacion" 2>/dev/null || true; wait "$simulacion" 2>/dev/null || true' EXIT
printf 'Evidencia en %s\n' "$evidencia"

timeout 55s ros2 topic echo /odom nav_msgs/msg/Odometry --once --field header >"$evidencia/odom.txt"
timeout 20s ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --field header --qos-reliability best_effort >"$evidencia/scan.txt"
timeout 20s ros2 topic echo /camera/image_raw sensor_msgs/msg/Image --once --field width --qos-reliability best_effort >"$evidencia/camara.txt"
printf 'Verificados los mensajes de odometría, LiDAR y cámara en Gazebo.\n'
if [[ "${1:-}" == --mision ]]; then
    python3 tools/A3_E6_R27_ejecutor.py --ejecutar --autorizar-movimiento \
        --desde ROBOT --orden RECEPCION | tee "$evidencia/mision.log"
fi
