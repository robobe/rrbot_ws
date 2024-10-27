import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
from pathlib import Path

URDF = "rrbot.urdf.xacro"
PKG_GAZEBO = "rrbot_gazebo"
PKG_DESCRIPTION = "rrbot_description"
PKG_BRINGUP = "rrbot_bringup"

"""
** test **
use ros2_control without gazebo

"""
def generate_launch_description():
    ld = LaunchDescription()

    pkg_description = get_package_share_directory(PKG_DESCRIPTION)
    pkg_bringup = get_package_share_directory(PKG_BRINGUP)
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    rviz_config_file = Path(pkg_bringup).joinpath("config", "rviz.rviz").as_posix()
    controller_config = Path(pkg_bringup).joinpath("config", "position.yaml").as_posix()

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both" ,
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        
    )

    params = {"robot_description": urdf, "use_sim_time": True}

    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
     
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # ld.add_action(robot_state_publisher)
    # ld.add_action(rviz_node)
    ld.add_action(control_node)
    # ld.add_action(robot_state_publisher)
    # ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(robot_controller_spawner)
    return ld