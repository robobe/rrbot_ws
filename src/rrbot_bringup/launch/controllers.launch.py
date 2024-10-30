from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from pathlib import Path
from launch.substitutions import Command

URDF = "rrbot.urdf.xacro"
PKG_DESCRIPTION = "rrbot_description"
PKG_BRINGUP = "rrbot_bringup"

def generate_launch_description():
    ld = LaunchDescription()

    pkg_description = get_package_share_directory(PKG_DESCRIPTION)
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    robot_description = {"robot_description": urdf}
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    pkg_bringup = get_package_share_directory(PKG_BRINGUP)
    robot_controllers = Path(pkg_bringup).joinpath("config", "rrbot_position.yaml").as_posix()
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[('/controller_manager/robot_description', '/robot_description', )]
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(control_node)

    return ld