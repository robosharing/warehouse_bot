import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    load_tricycle_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tricycle_controller"],
        output="screen"
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    return LaunchDescription([
        load_tricycle_controller,
        joint_broad_spawner,
    ])
