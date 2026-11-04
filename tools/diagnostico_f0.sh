#!/usr/bin/env bash
# Fase 0 - Diagnóstico no destructivo. No instala ni modifica configuraciones.
set -u
printf '%s\n' '=== Diagnóstico AMR F0 ==='
date -Is
printf '\n[Sistema]\n'
if command -v lsb_release >/dev/null 2>&1; then lsb_release -ds; else cat /etc/os-release 2>/dev/null | head -n 3; fi
uname -m
printf '\n[Python]\n'
python3 --version 2>&1 || true
printf '\n[ROS]\n'
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  echo "Distribución: ${ROS_DISTRO:-no detectada}"
else
  echo 'PENDIENTE: /opt/ros/humble/setup.bash no encontrado'
fi
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix rclpy 2>&1 || true
  ros2 pkg prefix turtlebot3_gazebo 2>&1 || true
  ros2 pkg prefix turtlebot3_teleop 2>&1 || true
else
  echo 'PENDIENTE: comando ros2 no disponible'
fi
printf '\n[Gazebo]\n'
if command -v gazebo >/dev/null 2>&1; then gazebo --version 2>&1 || true; else echo 'Gazebo Classic no detectado en PATH'; fi
if command -v gz >/dev/null 2>&1; then echo 'Comando gz detectado (revisar variante instalada)'; fi
printf '\n[Sesión gráfica]\n'
echo "DISPLAY=${DISPLAY:-<sin DISPLAY>}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<sin WAYLAND_DISPLAY>}"
printf '\n[Tópicos ROS, si existe simulador activo]\n'
if command -v ros2 >/dev/null 2>&1; then
  timeout 5s ros2 topic list -t 2>&1 || echo 'No se pudo obtener lista de tópicos (simulador apagado o ROS no operativo)'
fi
printf '\nPrueba F0.2 NO validada automáticamente: comprobar teleoperación y cámara visualmente.\n'
