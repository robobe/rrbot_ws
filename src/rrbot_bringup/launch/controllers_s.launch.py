from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from pathlib import Path


URDF = "rrbot.urdf.xacro"
PKG_DESCRIPTION = "rrbot_description"
PKG_BRINGUP = "rrbot_bringup"

def generate_launch_description():
    ld = LaunchDescription()


    control_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", 
                   "-c", "/controller_manager"
                  ],
        output="both",
    )

    ld.add_action(control_node)

    return ld