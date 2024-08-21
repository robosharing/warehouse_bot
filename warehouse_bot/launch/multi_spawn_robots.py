#!/usr/bin/python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument, TimerAction, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def nav2_setup(context, *args, **kwargs):
    robot_name = kwargs.get('robot_name')
    my_nav_dir = get_package_share_directory('warehouse_bot')
    my_param_dir = os.path.join(my_nav_dir, 'config')
    my_param_file = 'nav2_params_1.yaml'
    my_bt_file = 'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml'

    namespace = robot_name
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = os.path.join(my_param_dir, my_param_file)
    default_bt_xml_filename = os.path.join(my_param_dir, my_bt_file)
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/robot_1/odom','/odom')
                ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        # root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=namespace,
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            namespace=namespace,
            name='smoother_server',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace=namespace,
            name='planner_server',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace=namespace,
            name='recoveries_server',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace=namespace,
            name='bt_navigator',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            namespace=namespace,
            name='waypoint_follower',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            namespace=namespace,
            name='velocity_smoother',
            output='screen',
            parameters=[{configured_params}],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace,
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': autostart},
                {'node_names': lifecycle_nodes}
            ]
        ),
    ]

# def localization_setup(context, *args, **kwargs):
#     robot_name = kwargs.get('robot_name')
#     params_file_path = kwargs.get('params_file_path')

#     remappings = [
#                 ('/tf', 'tf'),
#                 ('/tf_static', 'tf_static'),
#                 ]

#     return [
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             namespace = robot_name,
#             name='amcl',
#             output='screen',
#             parameters=[params_file_path],
#             remappings=remappings
#         )
#     ]

def generate_amcl_node(config_yaml_file, namespace):
    return Node(
        package='nav2_amcl',
        executable='amcl',
        namespace=namespace,
        name='amcl',
        output='screen',
        parameters=[config_yaml_file  # Передаем путь к YAML файлу
        ]
    )

def generate_map_server_node(map_yaml_file):
    return Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[ {'use_sim_time': True},
                     {'yaml_filename': map_yaml_file},  # Передаем путь к YAML файлу
                     {'frame_id' : "map"},
                     {'topic_name': "map"}],
    )


def generate_lifecycle_manager_node(lifecycle_nodes):
    return Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': lifecycle_nodes}]
    )

def spawn_controllers_setup(context, *args, **kwargs):
    robot_name = kwargs.get('robot_name')
    spawn_controller_1_name = robot_name + "_spawn_controller_joint_state_broadcaster"
    spawn_controller_2_name = robot_name + "_spawn_controller_forward_position_controller"
    spawn_controller_3_name = robot_name + "_spawn_controller_velocity_controller"
    spawn_controller_4_name = robot_name + "_spawn_controller_rlcar_gazebo_controller"
    spawn_controller_5_name = robot_name + "_spawn_controller_rlcar_gazebo_odometry"
    controller_manager_name = "/" + robot_name + "/controller_manager"

    spawn_controller_1 = Node(
        package="controller_manager",
        executable="spawner",
        name=spawn_controller_1_name,
        namespace=robot_name,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager_name],
        output="screen"
    )

    spawn_controller_2 = Node(
        package="controller_manager",
        executable="spawner",
        name=spawn_controller_2_name,
        namespace=robot_name,
        arguments=["forward_position_controller", "--controller-manager", controller_manager_name],
        output="screen"
    )

    spawn_controller_3 = Node(
        package="controller_manager",
        executable="spawner",
        name=spawn_controller_3_name,
        namespace=robot_name,
        arguments=["velocity_controller", "--controller-manager", controller_manager_name],
        output="screen"
    )

    rlcar_gazebo_controller = Node(
        package='rlcar_gazebo_controller',
        executable='rlcar_gazebo_controller',
        name=spawn_controller_4_name,
        namespace=robot_name,
        output='screen',
        arguments=["--controller-manager", controller_manager_name],
        parameters=[],
    )
    odometry_node = Node(
        package ='rlcar_gazebo_odometry',
        executable ='rlcar_gazebo_odometry',
        namespace = robot_name,  # unique robot_name/namespace
        name =spawn_controller_5_name,
        output ='log',
        parameters =[{
            "verbose": False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': False,
            'is_gazebo': True,
            'wheel_radius' : 0.075,
            'imu_topic' : "/data/imu",
            'base_frame_id': robot_name + "_base_footprint",
            'odom_frame_id': robot_name + "_odom",
            'lr_wheel_joint_name': robot_name + '_lr_wheel_joint',  # Указываем имя для левого заднего колеса
            'rr_wheel_joint_name': robot_name + '_rr_wheel_joint',  # Указываем имя для правого заднего колеса
            'clock_topic': '/clock',
            'enable_odom_tf': True,
        }],
    )

    forward_position_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_1,
            on_exit=[spawn_controller_2]
        )
    )

    velocity_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_2,
            on_exit=[spawn_controller_3]
        )
    )

    rlcar_gazebo_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_3,
            on_exit=[rlcar_gazebo_controller]
        )
    )
    rlcar_gazebo_odometry_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_controller_3,
            on_exit=[odometry_node]
        )
    )
    fleet_client = Node(
                package='free_fleet_client_ros2',
                executable='free_fleet_client_ros2',
                name=f"{robot_name}_ff_client_node",
                namespace=robot_name,
                output='both',
                parameters=[
                    {'fleet_name': 'v1'},
                    {'robot_name': robot_name},
                    {'robot_model': 'cloudy'},
                    {'level_name': 'L1'},
                    {'dds_domain': 42},
                    {'max_dist_to_first_waypoint': 10.00},
                    {'map_frame': 'map'},
                    {'robot_frame': f'{robot_name}_base_footprint'},
                    {'nav2_server_name': f'/{robot_name}/navigate_to_pose'},
                    {'battery_state_topic': f'/{robot_name}/battery_state'},
                    {'update_frequency': 20.0},
                    {'publish_frequency': 2.0},
                    {'use_sim_time': False}
                ]
            )
    battery_state = Node(
            package="robot_control",
            executable="battery_simulator",
            namespace=robot_name,
        )




    return [
        spawn_controller_1,
        forward_position_controller_handler,
        velocity_controller_handler,
        rlcar_gazebo_controller_handler,
        rlcar_gazebo_odometry_handler,
        fleet_client,
        battery_state
    ]

def spawn_robot_setup(context, *args, **kwargs):
    entity_name = kwargs.get('robot_name')
    x_spawn = kwargs.get('x_spawn')
    y_spawn = kwargs.get('y_spawn')
    z_spawn = kwargs.get('z_spawn')
    roll_spawn = kwargs.get('roll_spawn')
    pitch_spawn = kwargs.get('pitch_spawn')
    yaw_spawn = kwargs.get('yaw_spawn')

    robot_description_topic_name = "/" + entity_name + "_robot_description"
    robot_state_publisher_name = entity_name + "_robot_state_publisher"

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='robot_spawn_entity',
        output='screen',
        emulate_tty=True,
        arguments=['-entity', entity_name,
                   '-x', str(x_spawn), '-y', str(y_spawn), '-z', str(z_spawn),
                   '-R', str(roll_spawn), '-P', str(pitch_spawn), '-Y', str(yaw_spawn),
                   '-topic', robot_description_topic_name
                   ],
        remappings=[("/robot_state_publisher", robot_state_publisher_name)]
    )
    return [spawn_robot]

def robot_state_publisher_setup(context, *args, **kwargs):
    robot_name = kwargs.get('robot_name')
    robot_file = kwargs.get('robot_file')
    if not robot_name or not robot_file:
        raise ValueError("robot_name and robot_file must be provided")

    robot_description_topic_name = "/" + robot_name + "_robot_description"
    joint_state_topic_name = "/" + robot_name + "/joint_states"
    scan_state_topic_name = "/" + robot_name + "/scan"
    imu_state_topic_name = "/" + robot_name + "/imu/data"
    robot_state_publisher_name = robot_name + "_robot_state_publisher"
    package_description = "warehouse_bot"
    
    warehouse_bot_dir = get_package_share_directory(package_description)
    robot_desc_path = os.path.join(warehouse_bot_dir, "urdf", robot_file)
    
    # Ensure the robot description file exists
    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/Xacro file not found: {robot_desc_path}")

    # Process the xacro file
    robot_desc = xacro.process_file(robot_desc_path, mappings={'robot_name': robot_name})
    xml = robot_desc.toxml()

    # Log the used paths and remappings
    print(f"Starting robot_state_publisher for {robot_name}")
    print(f"URDF file: {robot_desc_path}")
    print(f"Robot description topic: {robot_description_topic_name}")
    print(f"Joint states topic: {joint_state_topic_name}")
    print(f"Scan topic: {scan_state_topic_name}")
    print(f"IMU topic: {imu_state_topic_name}")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'robot_description': xml
        }],
        remappings=[
            ("/robot_description", robot_description_topic_name),
            ("/joint_states", joint_state_topic_name),
            # ("/scan", scan_state_topic_name),  # Remapping scan topic
            # ("/imu/data", imu_state_topic_name)  # Remapping IMU topic
        ],
        output="screen"
    )

    return [robot_state_publisher_node]

def generate_robot_spawn_descriptions(context):
    warehouse_bot_dir = get_package_share_directory('warehouse_bot')
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    x_offset = 2.0  # расстояние между роботами по оси x


    launch_descriptions = []

    for i in range(num_robots):
        x_spawn_value = float(LaunchConfiguration('x_spawn').perform(context)) + i * x_offset
        y_spawn_value = float(LaunchConfiguration('y_spawn').perform(context))
        robot_name = f'robot_{i + 1}'
        # params_file_name = f'nav2_params_{i + 1}.yaml' 
        # params_file_path = os.path.join(warehouse_bot_dir, 'config', params_file_name)

               # Формируем имя файла параметров на основе имени робота

        robot_state_publisher_action = OpaqueFunction(function=robot_state_publisher_setup, kwargs={
            'robot_name': robot_name,
            'robot_file': LaunchConfiguration('robot_file').perform(context)
        })

                # Настройка remappings для каждого робота

        spawn_robot_action = OpaqueFunction(function=spawn_robot_setup, kwargs={
            'robot_name': robot_name,
            'x_spawn': x_spawn_value,
            'y_spawn': LaunchConfiguration('y_spawn').perform(context),
            'z_spawn': LaunchConfiguration('z_spawn').perform(context),
            'roll_spawn': LaunchConfiguration('roll_spawn').perform(context),
            'pitch_spawn': LaunchConfiguration('pitch_spawn').perform(context),
            'yaw_spawn': LaunchConfiguration('yaw_spawn').perform(context)
        })

        robot_control = Node(
            package="robot_control",
            executable="fake_action_server",
            #name="simple_path_planner",
            namespace=robot_name,
            parameters=[{"robot_name":robot_name},
                        {"initial_x":0.0},
                        {"initial_y":0.0}],
        )

                # Создаем amcl_node для каждого робота

        spawn_controllers_action = OpaqueFunction(function=spawn_controllers_setup, kwargs={'robot_name': robot_name})

        # amcl_node = generate_amcl_node(
        #     config_yaml_file=params_file_path,  # Передаем путь к файлу параметров
        #     namespace=robot_name
        # )

        actions = [
            LogInfo(msg=f'Spawning and setting up controllers for robot {robot_name}'),
            robot_state_publisher_action,
            spawn_robot_action,
            spawn_controllers_action,
            # amcl_node
            robot_control
        ]

        if i == 0:
            # Первый робот запускается сразу
            launch_descriptions += actions
        else:
            # Последующие роботы запускаются с задержкой
            other_actions_delay = TimerAction(
                period=10.0 * i,  # Задержка для запуска каждого следующего робота
                actions=actions
            )
            launch_descriptions.append(other_actions_delay)


    return launch_descriptions

def generate_launch_description():
    warehouse_bot_dir = get_package_share_directory('warehouse_bot')
    world_path = os.path.join(warehouse_bot_dir, 'world', 't.world')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    params_file_name = 'nav2_params_1.yaml' 
    params_file_name1 = 'nav2_params_2.yaml' 
    params_file_path = os.path.join(warehouse_bot_dir, 'config', params_file_name)
    params_file_path1 = os.path.join(warehouse_bot_dir, 'config', params_file_name1)
    # Установка пути к моделям Gazebo
    gazebo_model_path = os.path.join(warehouse_bot_dir, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # Вывод информации для пользователей
    print("If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.")

    # Запуск Gazebo сервера
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Запуск Gazebo клиента (опционально)
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    map_yaml_file = os.path.join(warehouse_bot_dir, 'config', 'warehouse_map.yaml')
    # Настройка параметров для map_server и lifecycle_manager
    lifecycle_nodes = ['map_server', 'robot_1/amcl']



    amcl_node = generate_amcl_node(
            config_yaml_file=params_file_path,  # Передаем путь к файлу параметров
            namespace="robot_1"
        )
    
    amcl_node2= generate_amcl_node(
            config_yaml_file=params_file_path1,  # Передаем путь к файлу параметров
            namespace="robot_2"
        )



    # Создаем узлы map_server и lifecycle_manager
    map_server_node = generate_map_server_node(map_yaml_file)
    lifecycle_manager_node = generate_lifecycle_manager_node(lifecycle_nodes)

    declare_arguments = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('robot_file', default_value='robots.urdf.xacro'),
        DeclareLaunchArgument('x_spawn', default_value='0.0'),
        DeclareLaunchArgument('y_spawn', default_value='0.0'),
        DeclareLaunchArgument('z_spawn', default_value='0.06'),
        DeclareLaunchArgument('roll_spawn', default_value='0.0'),
        DeclareLaunchArgument('pitch_spawn', default_value='0.0'),
        DeclareLaunchArgument('yaw_spawn', default_value='0.78535'),
        DeclareLaunchArgument('num_robots', default_value='1'),
    ]

    launch_descriptions = [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        map_server_node,
        lifecycle_manager_node,
        amcl_node,
        # amcl_node2,
        OpaqueFunction(function=generate_robot_spawn_descriptions)
    ]

    return LaunchDescription(declare_arguments + launch_descriptions)
