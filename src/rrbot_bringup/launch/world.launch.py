import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = "rrbot_gazebo"
WORLD = "empty.world"

def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    # source /usr/share/gazebo/setup.sh
    resources = [os.path.join(pkg_share, "worlds")
                ]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"verbose": "true", "world": WORLD}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(resource_env)
    ld.add_action(gazebo)

    return ld