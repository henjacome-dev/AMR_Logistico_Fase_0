#!/usr/bin/env python3
"""B5: almacén, emulador y supervisor sin movimiento automático."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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

    return LaunchDescription([
        almacen,
        ejecutar("emulador_sensores_f1.py"),
        ejecutar("supervisor_f13.py"),
    ])
