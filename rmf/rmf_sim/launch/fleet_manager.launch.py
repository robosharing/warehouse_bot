#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Аргументы запуска

    # Запуск ноды update_coordinate.py
    update_coordinate_node = Node(
        package='rmf_sim',
        executable='update_coordinate.py',
        name='update_coordinate',
        output='screen'
    )

    # Путь к launch файлу
    manager_start_launch_file = os.path.join(
        get_package_share_directory('rmf_sim'),
        'manager_launch.py'  # Update to match the correct path
    )

    # Включение launch файла после завершения ноды update_coordinate
    manager_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manager_start_launch_file),
        launch_arguments={}.items()  # Здесь можно передавать аргументы запуска, если нужно
    )

    # Событие для запуска manager_start после завершения update_coordinate_node
    manager_start_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=update_coordinate_node,
            on_exit=[manager_start]
        )
    )

    return LaunchDescription([

        # Запуск ноды update_coordinate первой
        update_coordinate_node,

        # Запуск остальных действий после завершения update_coordinate
        manager_start_event
    ])