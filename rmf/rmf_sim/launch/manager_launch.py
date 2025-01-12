#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # Аргументы запуска
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    failover_mode = LaunchConfiguration('failover_mode', default='true')
    server_uri = LaunchConfiguration('server_uri', default='ws://localhost:8000')

    # Fleet server node
    fleet_server_node = Node(
        package="free_fleet_server_ros2",
        executable="free_fleet_server_ros2",
        name="v1_fleet_server_node",
        output="both",
        parameters=[{
            'fleet_name': 'v1',
            'fleet_state_topic': 'fleet_states',
            'mode_request_topic': 'robot_mode_requests',
            'path_request_topic': 'robot_path_requests',
            'destination_request_topic': 'robot_destination_requests',
            'dds_domain': 0,
            'dds_robot_state_topic': 'robot_state',
            'dds_mode_request_topic': 'mode_request',
            'dds_path_request_topic': 'path_request',
            'dds_destination_request_topic': 'destination_request',
            'update_state_frequency': 20.0,
            'publish_state_frequency': 2.0,
            'translation_x': 0.0,
            'translation_y': 0.0,
            'rotation': 0.0,
            'scale': 1.0
        }]
    )

    # Добавляем действия для fleet_server
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('failover_mode', default_value='true'))
    ld.add_action(DeclareLaunchArgument('server_uri', default_value='ws://localhost:8000'))
    ld.add_action(fleet_server_node)

    # Общие аргументы для fleet_client роботов
    common_args = [
        DeclareLaunchArgument('fleet_name', default_value='v1'),
        DeclareLaunchArgument('robot_model', default_value='cloudy'),
        DeclareLaunchArgument('level_name', default_value='L1'),
        DeclareLaunchArgument('max_dist_to_first_waypoint', default_value='10.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('update_state_frequency', default_value='20.0'),
        DeclareLaunchArgument('publish_state_frequency', default_value='2.0')
    ]

    for arg in common_args:
        ld.add_action(arg)

    # Загрузка списка роботов из YAML
    rmf_sim_share_dir = get_package_share_directory('rmf_sim')
    robots_file_path = os.path.join(rmf_sim_share_dir, 'config', 'robots.yaml')
    robots = []
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots.extend(yaml_data['robots'])
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Запуск fleet_client для каждого робота
    for robot in robots:
        namespace = robot['name']
        robot_name = robot['name']

        # Узел fleet_client
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
                'dds_domain': 0,
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
        ld.add_action(log_action)
        ld.add_action(fleet_client)

    return ld

if __name__ == "__main__":
    from launch import LaunchService

    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()

