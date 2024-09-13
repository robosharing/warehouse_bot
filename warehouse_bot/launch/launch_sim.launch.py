#!/usr/bin/env python3

import os
from osrf_pycommon.terminal_color import ansi
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Common paths
    pkg_warehouse_bot = get_package_share_directory('warehouse_bot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # pkg_robosense_description = get_package_share_directory('robosense_description')

    # Set up environment variables for Gazebo
    gazebo_model_path = os.path.join(pkg_warehouse_bot, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = os.environ.get('GAZEBO_MODEL_PATH', '') + ":" + gazebo_model_path
    print(ansi("yellow"), "If it's your 1st time downloading Gazebo model, it may take a few minutes to finish.", ansi("reset"))

    # Launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to start RVIZ')

    gpu = LaunchConfiguration('gpu')
    declare_gpu_cmd = DeclareLaunchArgument('gpu', default_value='False', description='Whether to use Gazebo gpu_ray or ray')

    # Paths
    world_path = os.path.join(pkg_warehouse_bot, 'world', 'updated.world')
    xacro_path = os.path.join(pkg_warehouse_bot, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_warehouse_bot, 'config', 'nav2_config_preferred_lanes.rviz')
    # robosense_rviz_config_file = os.path.join(pkg_robosense_description, 'rviz', 'example.rviz')

    # Robot description
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_path, ' gpu:=', gpu])
    params = {'robot_description': robot_description_content}

    # Nodes and commands
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-x', '10.0', '-y', '10.0', '-z', '0.06', '-Y', '0.78535', '-entity', 'rlcar'],
        output='screen'
    )

    load_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
        output="screen",
    )

    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rlcar_gazebo_controller = Node(
        package='rlcar_gazebo_controller',
        executable='rlcar_gazebo_controller',
        output='screen',
        parameters=[],
    )

    rlcar_gazebo_odometry = Node(
        package='rlcar_gazebo_odometry',
        executable='rlcar_gazebo_odometry',
        name='rlcar_gazebo_odometry',
        output='log',
        parameters=[{
            "verbose": False,
            'publish_rate': 50,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'wheel_radius': 0.075,
            'base_frame_id': "base_footprint",
            'odom_frame_id': "odom",
            'enable_odom_tf': True,
        }],
    )

    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        )
    )

    # Build the launch description
    return LaunchDescription([
        declare_use_rviz,
        declare_gpu_cmd,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # rqt_robot_steering,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_forward_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_forward_position_controller,
                on_exit=[load_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[rlcar_gazebo_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[rlcar_gazebo_odometry],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_velocity_controller,
                on_exit=[start_rviz_cmd],
            )
        ),
        exit_event_handler
    ])
