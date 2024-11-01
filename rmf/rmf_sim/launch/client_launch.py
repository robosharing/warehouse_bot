#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # Указание пути к файлу robots.yaml
    robots_file_path = os.path.join(get_package_share_directory('rmf_sim'), 'config', 'robots.yaml')

    # Общие аргументы для всех роботов
    common_args = [
        DeclareLaunchArgument('fleet_name', default_value='v1'),
        DeclareLaunchArgument('robot_model', default_value='cloudy'),
        DeclareLaunchArgument('level_name', default_value='L1'),
        DeclareLaunchArgument('max_dist_to_first_waypoint', default_value='10.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('update_state_frequency', default_value='20.0'),
        DeclareLaunchArgument('publish_state_frequency', default_value='2.0')
    ]
    
    # Добавляем аргументы в LaunchDescription
    for arg in common_args:
        ld.add_action(arg)

    # Загрузка списка роботов из файла YAML
    robots = []
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots.extend(yaml_data['robots'])
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Запуск fleet_client для каждого робота
    for robot in robots:
        namespace = robot['name']
        robot_name = robot['name']

        # Создание узла fleet_client
        fleet_client = Node(
            package='free_fleet_client_ros2',
            executable='free_fleet_client_ros2',
            name=f"{robot_name}_fleet_client",
            namespace=namespace,
            output='screen',
            remappings=remappings,
            parameters=[{
                'fleet_name': LaunchConfiguration('fleet_name'),
                'robot_name': robot_name,
                'robot_model': LaunchConfiguration('robot_model'),
                'level_name': LaunchConfiguration('level_name'),
                'dds_domain': 42,
                'max_dist_to_first_waypoint': LaunchConfiguration('max_dist_to_first_waypoint'),
                'map_frame': LaunchConfiguration('map_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'nav2_server_name': f'/{robot_name}/navigate_to_pose',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'update_frequency': LaunchConfiguration('update_state_frequency'),
                'publish_frequency': LaunchConfiguration('publish_state_frequency'),
                'battery_state_topic': f'/{robot_name}/battery_state'
            }]
        )

        # Логирование для каждого робота
        log_action = LogInfo(msg=f"Launching fleet_client for {robot_name} in namespace {namespace}")

        # Добавляем логирование и fleet_client в LaunchDescription
        ld.add_action(log_action)
        ld.add_action(fleet_client)

    return ld

if __name__ == "__main__":
    from launch import LaunchService

    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
