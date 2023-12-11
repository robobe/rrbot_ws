from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
import xacro

PKG = "rrbot_bringup"
URDF_FILE = "rrbot.urdf.xacro"
PKG_DESCRIPTION = "rrbot_description"


def generate_launch_description():
    ld = LaunchDescription()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", "rrbot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(PKG_DESCRIPTION),
                    "urdf",
                    URDF_FILE,
                ]
            ),
        ]
    )

    params = {"robot_description": robot_description_content, "use_sim_time": True}
    node_robot_state_publisher = Node(
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
        arguments=["-d", rviz_config_file],
    )

    app_node = Node(
        package="rrbot_application",
        executable="sim_joint_state",
        name="sim_joint_state",
        output="log"
    )

    ld.add_action(node_robot_state_publisher)
    ld.add_action(app_node)
    ld.add_action(rviz_node)
    return ld
