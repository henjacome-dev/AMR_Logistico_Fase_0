#!/usr/bin/env python3

from pathlib import Path
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    raiz = Path(__file__).resolve().parent.parent

    escenario = yaml.safe_load(
        (raiz / "config/escenario_f1.yaml").read_text(
            encoding="utf-8"
        )
    )

    x, y = escenario["robot"]["start_xy_m"]
    yaw = escenario["robot"]["start_yaw_rad"]

    mundo = raiz / "modelos/almacen_f1.world"

    gazebo_pkg = Path(
        get_package_share_directory("gazebo_ros")
    )
    robot_pkg = Path(
        get_package_share_directory("turtlebot3_gazebo")
    )
    descripcion_pkg = Path(
        get_package_share_directory("turtlebot3_description")
    )

    modelo = (
        robot_pkg / "models/turtlebot3_waffle_pi/model.sdf"
    )
    urdf = (
        descripcion_pkg / "urdf/turtlebot3_waffle_pi.urdf"
    )

    robot_description = subprocess.check_output(
        ["xacro", str(urdf)], text=True
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_pkg / "launch/gazebo.launch.py")
        ),
        launch_arguments={
            "world": str(mundo),
            "verbose": "true",
        }.items(),
    )

    publicador = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "waffle_pi",
            "-file", str(modelo),
            "-x", str(x),
            "-y", str(y),
            "-z", "0.05",
            "-Y", str(yaw),
        ],
    )

    return LaunchDescription([
        gazebo,
        publicador,
        robot,
    ])
