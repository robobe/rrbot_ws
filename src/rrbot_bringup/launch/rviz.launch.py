from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription

PKG = "rrbot_bringup"

def generate_launch_description():
    ld = LaunchDescription()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", "rrbot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    ld.add_action(rviz_node)
    return ld
