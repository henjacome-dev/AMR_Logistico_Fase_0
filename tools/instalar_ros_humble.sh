#!/usr/bin/env bash
set -euo pipefail

. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != jammy ]]; then
    printf 'Este proyecto requiere Ubuntu 22.04 (Jammy).\n' >&2
    exit 1
fi

admin=()
if (( EUID != 0 )); then
    admin=(sudo)
fi

"${admin[@]}" apt-get update
"${admin[@]}" apt-get install -y --no-remove ca-certificates curl software-properties-common
"${admin[@]}" add-apt-repository -y universe

if ! dpkg-query -W -f='${db:Status-Status}\n' ros2-apt-source 2>/dev/null | grep -qx installed; then
    version=$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
    paquete=$(mktemp --suffix=.deb)
    trap 'rm -f -- "$paquete"' EXIT
    curl -fsSL --retry 3 -o "$paquete" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
    "${admin[@]}" dpkg -i "$paquete"
fi

"${admin[@]}" apt-get update
"${admin[@]}" apt-get upgrade -y --no-remove
"${admin[@]}" apt-get install -y --no-remove ros-humble-desktop ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description ros-humble-xacro \
    python3-yaml python3-pil python3-numpy python3-venv graphviz

set +u
source /opt/ros/humble/setup.bash
set -u
ros2 pkg prefix gazebo_ros
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix turtlebot3_description
dpkg-query -W gazebo
