from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from pathlib import Path

import xacro

URDF = "rrbot.urdf.xacro"
PKG_DESCRIPTION = "rrbot_description"


def generate_launch_description():
    ld = LaunchDescription()
    pkg_description = get_package_share_directory(PKG_DESCRIPTION)
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    params = {"robot_description": urdf, "use_sim_time": True}
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rrbot_bringup"),
            "config",
            "position_and_effort_imu.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[urdf, robot_controllers],
        output="both",
    )

    ld.add_action(control_node)
    ld.add_action(node_robot_state_publisher)
    return ld
