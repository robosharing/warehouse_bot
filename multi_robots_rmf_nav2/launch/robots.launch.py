#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, LogInfo, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration
import xacro
import yaml

def generate_launch_description():
    ld = LaunchDescription()

    # Путь до файла с конфигурацией роботов
    robots_file_path = os.path.join(get_package_share_directory('multi_robots_rmf_nav2'), 'config', 'robots.yaml')

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    robots = yaml_data['robots']

    # Общие пути к файлам и конфигурация
    pkg_warehouse_bot = get_package_share_directory('multi_robots_rmf_nav2')
    xacro_path = os.path.join(pkg_warehouse_bot, 'urdf', 'robot.urdf.xacro')
    map_dir = os.path.join(pkg_warehouse_bot, 'maps', 'warehouse_map.yaml')
    rviz_config_file = os.path.join(pkg_warehouse_bot, 'rviz', 'multi_nav2_default_view.rviz')
    params_file = os.path.join(pkg_warehouse_bot, 'config', 'nav2_params.yaml')

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

    # Логирование проверки файла параметров
    ld.add_action(LogInfo(msg=f"Путь к nav2_params.yaml: {params_file}"))
    if os.path.exists(params_file):
        ld.add_action(LogInfo(msg=f"Файл nav2_params.yaml успешно загружен из {params_file}"))
    else:
        ld.add_action(LogInfo(msg=f"Ошибка: nav2_params.yaml не найден по пути {params_file}"))

    remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan2", "scan2"),
            ("/odom", "odom")
        ]
    bridge_params = os.path.join(pkg_warehouse_bot,'config','gz_bridge.yaml')
    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ld.add_action(ros_gz_bridge_clock)   

    last_action = None
    for robot in robots:
        name = robot['name']
        namespace = '/' + robot['name']

        # Публикация описания робота через xacro
        robot_desc = xacro.process_file(
            xacro_path,
            mappings={'robot_name': name}
        ).toxml()
        params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

        # Создание узлов для робота
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[params],
            remappings=remappings
        )
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-topic', f'{namespace}/robot_description',
                '-name', f'{namespace}',
                '-allow_renaming', 'true',
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', '0.06',
                '-Y', '0.78535',
            ],
            output='screen'
        )
        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                f'{namespace}/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'{namespace}/scan2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'{namespace}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                # f'{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                # f'{namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                # f'{namespace}/color/image_rect@sensor_msgs/msg/Image@gz.msgs.Image',
                # f'{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ]
        )

        spawn_controller_1 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_controller_joint_state_broadcaster",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            output="screen",
            remappings=remappings
        )
        spawn_controller_2 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_controller_forward_position_controller",
            namespace=namespace,
            arguments=["forward_position_controller"],
            output="screen",
            remappings=remappings
        )
        spawn_controller_3 = Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_controller_velocity_controller",
            namespace=namespace,
            arguments=["velocity_controller"],
            output="screen",
            remappings=remappings
        )

        rlcar_gazebo_controller = Node(
            package='rlcar_gazebo_controller',
            executable='rlcar_gazebo_controller',
            name=f"spawn_controller_rlcar_gazebo_controller",
            namespace=namespace,
            output='screen',
            remappings=remappings
        )

        odometry_node = Node(
            package='rlcar_gazebo_odometry',
            executable='rlcar_gazebo_odometry',
            name=f"spawn_controller_rlcar_gazebo_odometry",
            namespace=namespace,
            output='log',
            parameters=[{
                "verbose": False,
                'publish_rate': 50,
                'open_loop': False,
                'has_imu_heading': True,
                'is_gazebo': True,
                'wheel_radius': 0.075,
                'imu_topic': f"{namespace}/imu",
                'base_frame_id': "base_link",
                'odom_frame_id': "odom",
                'lr_wheel_joint_name': 'lr_wheel_joint',
                'rr_wheel_joint_name': 'rr_wheel_joint',
                'clock_topic': '/clock',
                'enable_odom_tf': True,
            }],
            remappings=remappings
        )

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

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_warehouse_bot, 'launch', "rviz_launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items()
        )

        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
                  robot['x_pose'] + ', y: ' + robot['y_pose'] + \
                  ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.3827, w: 0.9239}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )
        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', f'{namespace}/battery_state', 'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
                '-r', '1'
            ],
            output='log'
        )
        controller_act = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            spawn_controller_1,
            spawn_controller_3,
            spawn_controller_2,
            rlcar_gazebo_controller,
            odometry_node,
        ])

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        robot_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            robot_state_publisher,
            spawn_entity,

            controller_act,
            ros_gz_bridge,
            nav2_actions,
            fake_bms,
            # rviz,
        ])





        # Условное добавление действий
        if last_action is None:
            ld.add_action(robot_actions)
        else:
            spawn_event_handler = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        robot_actions,
                    ],
                )
            )
            ld.add_action(spawn_event_handler)

        last_action = spawn_controller_2

    return ld
