from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
import xacro

URDF = "rrbot.urdf.xacro"

def generate_launch_description():
    package_name = "rrbot_description"
    pkg_description = get_package_share_directory(package_name)
    
    xacro_file = Path(pkg_description).joinpath("urdf", URDF).as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    params = {"robot_description": urdf, "use_sim_time": True}
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "rrbot"
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            spawn_entity,
        ]
    )