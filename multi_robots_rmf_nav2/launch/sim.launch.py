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
        package='multi_robots_rmf_nav2',
        executable='update_coordinate.py',
        name='update_coordinate',
        output='screen'
    )

    # Путь к launch файлу
    ware_bot_start_launch_file = os.path.join(
        get_package_share_directory('multi_robots_rmf_nav2'),
        'launch',
        'robots.launch.py'  # Используйте правильное имя launch файла с .py
    )

    # Включение launch файла после завершения ноды update_coordinate
    ware_bot_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ware_bot_start_launch_file),
        launch_arguments={}.items()  # Здесь можно передавать аргументы запуска, если нужно
    )

    # Событие для запуска ware_bot_start после завершения update_coordinate_node
    ware_bot_start_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=update_coordinate_node,
            on_exit=[ware_bot_start]
        )
    )

    return LaunchDescription([

        # Запуск ноды update_coordinate первой
        update_coordinate_node,

        # Запуск остальных действий после завершения update_coordinate
        ware_bot_start_event
    ])