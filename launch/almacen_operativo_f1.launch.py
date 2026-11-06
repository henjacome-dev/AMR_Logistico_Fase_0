#!/usr/bin/env python3
"""B5: almacén, emulador y supervisor sin movimiento automático."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    raiz = Path(__file__).resolve().parent.parent

    almacen = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(raiz / "launch/almacen_f1.launch.py")
        )
    )

    def ejecutar(archivo):
        return ExecuteProcess(
            cmd=[
                sys.executable,
                "-u",
                str(raiz / "tools" / archivo),
                "--ros-args",
                "-p",
                "use_sim_time:=true",
            ],
            cwd=str(raiz),
            output="screen",
        )

    # F1/R35: map y odom coinciden en esta simulacion.
    # No ejecutar simultaneamente otro emisor map -> odom.
    map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_static_f1",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        almacen,
        map_odom,
        ejecutar("emulador_sensores_f1.py"),
        ejecutar("supervisor_f13.py"),
    ])
