from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from pathlib import Path
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

URDF = "rrbot.urdf.xacro"
PKG_DESCRIPTION = "rrbot_description"
PKG_BRINGUP = "rrbot_bringup"

def generate_launch_description():
    """
    load urdf
    - load controller and resource manager
    - spawn position and joint state controllers after controller manager start
    """
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

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[('/controller_manager/robot_description', '/robot_description', )]
    )

    position_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", 
                   "-c", "/controller_manager"
                  ],
        output="both",
    )

    state_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "-c", "/controller_manager"
                  ],
        output="both",
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[state_control_spawner],
        )
    )

    delayed_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[position_control_spawner],
        )
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(delayed_position_controller_spawner)
    ld.add_action(delayed_joint_broad_spawner)

    return ld