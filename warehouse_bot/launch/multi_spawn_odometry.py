#!/usr/bin/python3

import os
import xacro # type: ignore
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument, TimerAction, LogInfo # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import LaunchConfiguration # type: ignore
from launch.event_handlers import OnProcessExit # type: ignore
from ament_index_python.packages import get_package_share_directory # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore

def spawn_controllers_setup(context, *args, **kwargs):
    robot_name = kwargs.get('robot_name')
    spawn_controller_1_name = robot_name + "_spawn_controller_joint_state_broadcaster"
    spawn_controller_2_name = robot_name + "_spawn_controller_forward_position_controller"
    spawn_controller_3_name = robot_name + "_spawn_controller_velocity_controller"
    spawn_controller_4_name = robot_name + "_spawn_controller_rlcar_gazebo_controller"
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
        parameters=[],
    )
    odometry_node = Node(
        package ='rlcar_gazebo_odometry',
        executable ='rlcar_gazebo_odometry',
        namespace = robot_name,  # unique robot_name/namespace
        name ='rlcar_gazebo_odometry',
        output ='log',
        parameters =[{
            "verbose": False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'wheel_radius' : 0.075,
            'base_frame_id': "base_footprint",
            'odom_frame_id': "odom",
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
            target_action=rlcar_gazebo_controller_handler,
            on_exit=[odometry_node]
        )
    )

    return [
        spawn_controller_1,
        forward_position_controller_handler,
        velocity_controller_handler,
        rlcar_gazebo_controller_handler,
        rlcar_gazebo_odometry_handler
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
    robot_description_topic_name = "/" + robot_name + "_robot_description"
    joint_state_topic_name = "/" + robot_name + "/joint_states"
    robot_state_publisher_name = robot_name + "_robot_state_publisher"

    package_description = "warehouse_bot"
    
    warehouse_bot_dir = get_package_share_directory(package_description)
    os.environ['warehouse_bot_dir'] = warehouse_bot_dir

    robot_desc_path = os.path.join(warehouse_bot_dir, "urdf", robot_file)
    robot_desc = xacro.process_file(robot_desc_path, mappings={'robot_name': robot_name})
    
    xml = robot_desc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{'use_sim_time': False, 'robot_description': xml}],
        remappings=[("/robot_description", robot_description_topic_name),
                    ("/joint_states", joint_state_topic_name)
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
        robot_name = f'robot_{i + 1}'

        robot_state_publisher_action = OpaqueFunction(function=robot_state_publisher_setup, kwargs={
            'robot_name': robot_name,
            'robot_file': LaunchConfiguration('robot_file').perform(context)
        })

        spawn_robot_action = OpaqueFunction(function=spawn_robot_setup, kwargs={
            'robot_name': robot_name,
            'x_spawn': x_spawn_value,
            'y_spawn': LaunchConfiguration('y_spawn').perform(context),
            'z_spawn': LaunchConfiguration('z_spawn').perform(context),
            'roll_spawn': LaunchConfiguration('roll_spawn').perform(context),
            'pitch_spawn': LaunchConfiguration('pitch_spawn').perform(context),
            'yaw_spawn': LaunchConfiguration('yaw_spawn').perform(context)
        })

        spawn_controllers_action = OpaqueFunction(function=spawn_controllers_setup, kwargs={'robot_name': robot_name})
    

        delay_action = TimerAction(
            period=i * 10.0,  # задержка в секундах между спауном роботов и запуском контроллеров
            actions=[LogInfo(msg=f'Spawning and setting up controllers for robot {robot_name}'),
                     robot_state_publisher_action, spawn_robot_action, spawn_controllers_action]
        )

        launch_descriptions.append(delay_action)

    return launch_descriptions

def generate_launch_description():
    warehouse_bot_dir = get_package_share_directory('warehouse_bot')
    world_path = os.path.join(warehouse_bot_dir, 'world', 't.world')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

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

    declare_arguments = [
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('robot_file', default_value='robot.xacro'),
        DeclareLaunchArgument('x_spawn', default_value='0.0'),
        DeclareLaunchArgument('y_spawn', default_value='0.0'),
        DeclareLaunchArgument('z_spawn', default_value='0.06'),
        DeclareLaunchArgument('roll_spawn', default_value='0.0'),
        DeclareLaunchArgument('pitch_spawn', default_value='0.0'),
        DeclareLaunchArgument('yaw_spawn', default_value='0.78535'),
        DeclareLaunchArgument('num_robots', default_value='1')
    ]

    launch_descriptions = [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        OpaqueFunction(function=generate_robot_spawn_descriptions)
    ]

    return LaunchDescription(declare_arguments + launch_descriptions)
