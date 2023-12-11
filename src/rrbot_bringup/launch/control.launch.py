from launch_ros.actions import Node

from launch import LaunchDescription

PKG = "rrbot_bringup"

def generate_launch_description():
    ld = LaunchDescription()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    imu_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(position_controller_spawner)
    ld.add_action(effort_controller_spawner)
    ld.add_action(imu_controller_spawner)
    
    return ld
