#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, GroupAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration
import xacro
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    # Путь до файла с конфигурацией роботов
    robots_file_path = os.path.join(get_package_share_directory('multi_robots_rmf_nav2'), 'config', 'robots.yaml')

    # Создаем пустой список роботов
    robots = []

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    # Добавление роботов из YAML файла в список
    robots.extend(yaml_data['robots'])

    # Общие пути к файлам
    pkg_warehouse_bot = get_package_share_directory('multi_robots_rmf_nav2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_warehouse_bot, 'world', 'ware.world')
    xacro_path = os.path.join(pkg_warehouse_bot, 'urdf', 'robot.urdf.xacro')
    map_dir = os.path.join(pkg_warehouse_bot, 'maps', 'warehouse_map.yaml')
    rviz_config_file = os.path.join(pkg_warehouse_bot, 'rviz', 'multi_nav2_default_view.rviz')
    params_file = os.path.join(pkg_warehouse_bot, 'config', 'nav2_params.yaml')

    # Проверка наличия файла nav2_params.yaml и вывод сообщения
    ld.add_action(LogInfo(msg=f"Путь к nav2_params.yaml: {params_file}"))
    if os.path.exists(params_file):
        ld.add_action(LogInfo(msg=f"Файл nav2_params.yaml успешно загружен из {params_file}"))
    else:
        ld.add_action(LogInfo(msg=f"Ошибка: nav2_params.yaml не найден по пути {params_file}"))

    # Объявление аргументов запуска
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Запуск RViz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Использовать симуляционное время'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_rviz)

    # Запуск серверной и клиентской частей Gazebo
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Подготовка серверной карты и диспетчера жизненного цикла
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Запуск роботов
    last_action = None
    for robot in robots:
        name = robot['name']
        namespace = '/' + robot['name']

        # Публикация описания робота через xacro
        robot_desc = xacro.process_file(xacro_path).toxml()
        params = {'robot_description': robot_desc}
        print(f"Запуск robot_state_publisher для {name}")

        # Узел публикации состояния робота
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[params],
        )    
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=f"{name}_joint_state_publisher",
            namespace=namespace,
        )

        # Запуск робота в Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', namespace + '/robot_description',
                '-x', robot['x_pose'], 
                '-y', robot['y_pose'], 
                '-z', '0.08', 
                '-Y', '0.78535', 
                '-entity', name,
                '-robot_namespace', namespace,
            ],
            output='screen',
            namespace=namespace,
        )

        # Контроллеры и другие узлы
        spawn_controller_1 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{name}_spawn_controller_joint_state_broadcaster",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            output="screen",
        )

        spawn_controller_2 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{name}_spawn_controller_forward_position_controller",
            namespace=namespace,
            arguments=["forward_position_controller"],
            output="screen",
        )

        spawn_controller_3 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{name}_spawn_controller_velocity_controller",
            namespace=namespace,
            arguments=["velocity_controller"],
            output="screen",
        )

        rlcar_gazebo_controller = Node(
            package='rlcar_gazebo_controller',
            executable='rlcar_gazebo_controller',
            name=f"{name}_spawn_controller_rlcar_gazebo_controller",
            namespace=namespace,
            output='screen'
        )

        odometry_node = Node(
            package='rlcar_gazebo_odometry',
            executable='rlcar_gazebo_odometry',
            name=f"{name}_spawn_controller_rlcar_gazebo_odometry",
            namespace=namespace,
            output='log',
            parameters=[{
                "verbose": False,
                'publish_rate': 50,
                'open_loop': False,
                'has_imu_heading': False,
                'is_gazebo': True,
                'wheel_radius': 0.075,
                'imu_topic': "/data/imu",
                'base_frame_id': "base_footprint",
                'odom_frame_id': "odom",
                'lr_wheel_joint_name': 'lr_wheel_joint',
                'rr_wheel_joint_name': 'rr_wheel_joint',
                'clock_topic': '/clock',
                'enable_odom_tf': True,
            }]
        )

        # Команда для запуска Nav2
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_warehouse_bot, 'launch', 'nav2', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'map': map_dir,
                'use_namespace': 'True',
                'params_file': params_file,
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        # Команда для запуска RViz
        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_warehouse_bot, 'launch', "rviz_launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items()
        )

        # Публикация начальной позиции робота
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
                robot['x_pose'] + ', y: ' + robot['y_pose'] + \
                ', z: 0.06}, orientation: {x: 0.0, y: 0.0, z: -0.3827, w: 0.9239}}, }}'
        
        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', 
                namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )

        # Узел клиента RMF
        fleet_client = Node(
            package='free_fleet_client_ros2',
            executable='free_fleet_client_ros2',
            name=f"{name}_ff_client_node",
            namespace=namespace,
            output='both',
            parameters=[
                {'fleet_name': 'v1'},
                {'robot_name': name},
                {'robot_model': 'cloudy'},
                {'level_name': 'L1'},
                {'dds_domain': 42},
                {'max_dist_to_first_waypoint': 10.00},
                {'map_frame': 'map'},
                {'robot_frame': 'base_footprint'},
                {'nav2_server_name': namespace + '/navigate_to_pose'},
                {'battery_state_topic': namespace + '/battery_state'},
                {'update_frequency': 20.0},
                {'publish_frequency': 2.0},
                {'use_sim_time': True}
            ]
        )

        # Группы действий для робота
        robot_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/scan2", dst="scan2"),
            spawn_entity,
            robot_state_publisher,
            joint_state_publisher,
            odometry_node,            
        ])

        controllers_action = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/scan2", dst="scan2"),
            spawn_controller_1,
            spawn_controller_2,
            spawn_controller_3,
            rlcar_gazebo_controller,
        ])

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/scan2", dst="scan2"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        rmf_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/scan2", dst="scan2"),
            fleet_client,
        ])

        # Условное добавление действий
        if last_action is None:
            ld.add_action(robot_actions)
            ld.add_action(controllers_action)
            ld.add_action(nav2_actions)
            ld.add_action(rmf_actions)
            ld.add_action(rviz)
        else:
            spawn_event_handler = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        rviz,
                        rmf_actions,
                        nav2_actions,
                        controllers_action,
                        robot_actions,
                        
                        
                    ],
                )
            )
            ld.add_action(spawn_event_handler)

        last_action = spawn_entity

    return ld
