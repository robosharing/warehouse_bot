#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument,GroupAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap, SetParameter
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
import xacro
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    robots_file_path = os.path.join(get_package_share_directory('multi_robots_rmf_nav2'), 'config', 'robots.yaml')  # Путь до файла
    

    # Создаем пустой список роботовs
    robots = []

    # robots = [
    # {'name': 'robot1', 'x_pose': '0', 'y_pose': '0', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot2', 'x_pose': '30', 'y_pose': '0', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot3', 'x_pose': '60', 'y_pose': '0', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot4', 'x_pose': '30', 'y_pose': '0', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot5', 'x_pose': '0', 'y_pose': '10', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot6', 'x_pose': '10', 'y_pose': '10', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot7', 'x_pose': '20', 'y_pose': '10', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot8', 'x_pose': '30', 'y_pose': '10', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot9', 'x_pose': '0', 'y_pose': '20', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot10', 'x_pose': '10', 'y_pose': '20', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot11', 'x_pose': '20', 'y_pose': '20', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot12', 'x_pose': '30', 'y_pose': '20', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot13', 'x_pose': '0', 'y_pose': '30', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot14', 'x_pose': '10', 'y_pose': '30', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot15', 'x_pose': '20', 'y_pose': '30', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot16', 'x_pose': '30', 'y_pose': '30', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot17', 'x_pose': '0', 'y_pose': '40', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot18', 'x_pose': '10', 'y_pose': '40', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot19', 'x_pose': '20', 'y_pose': '40', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot20', 'x_pose': '30', 'y_pose': '40', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot21', 'x_pose': '0', 'y_pose': '50', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot22', 'x_pose': '10', 'y_pose': '50', 'z_pose': '0.06', 'Y_pose': '0.78535'},
    # # {'name': 'robot23', 'x_pose': '20', 'y_pose': '50', 'z_pose': '0.06', 'Y_pose': '0.78535'},        
    # # {'name': 'robot24', 'x_pose': '30', 'y_pose': '50', 'z_pose': '0.06', 'Y_pose': '0.78535'},                  
    # # Add more robots if needed
    # ]

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    # Добавление роботов из YAML файла в список
    robots.extend(yaml_data['robots'])

    # Common paths
    pkg_warehouse_bot = get_package_share_directory('multi_robots_rmf_nav2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_warehouse_bot, 'world', 'empty.world')
    xacro_path = os.path.join(pkg_warehouse_bot, 'urdf', 'robot.urdf.xacro')
    map_dir = os.path.join(pkg_warehouse_bot, 'maps', 'warehouse_map.yaml')
    rviz_config_file = os.path.join(pkg_warehouse_bot, 'rviz', 'multi_nav2_default_view.rviz')

    # Display path to nav2_params.yaml
    params_file = os.path.join(pkg_warehouse_bot, 'config', 'nav2_params.yaml')
    ld.add_action(LogInfo(msg=f"Path to nav2_params.yaml: {params_file}"))

    # Check if the file exists and log a success message
    if os.path.exists(params_file):
        ld.add_action(LogInfo(msg=f"Successfully loaded nav2_params.yaml from {params_file}"))
    else:
        ld.add_action(LogInfo(msg=f"Error: nav2_params.yaml not found at {params_file}"))

    # Declare arguments
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable RViz launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_rviz)

    # Start Gazebo server and client
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Set up map server and lifecycle manager
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[ {'use_sim_time': True},
                     {'yaml_filename': map_dir},  # Передаем путь к YAML файлу
                     {'frame_id' : "map"},
                     {'topic_name':  "map"}
                     ],
        remappings=remappings
    )

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecyle)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/scan2', 'scan2')]
    # Launch robots
    last_action = None
    for robot in robots:
        name = robot['name']
        namespace = '/' + robot['name']

        robot_description_topic_name = "/" + name + "/robot_description"
        joint_state_topic_name = "/" + name + "/joint_states"
        scan_state_topic_name = "/" + name + "/scan"
        imu_state_topic_name = "/" + name + "/imu/data"
        robot_state_publisher_name = name + "_robot_state_publisher"
        # Process the xacro file
        robot_desc = xacro.process_file(xacro_path).toxml()
        params = {'robot_description': robot_desc}
        # Log the used paths and remappings
        print(f"Starting robot_state_publisher for {name}")
        # print(f"URDF file: {robot_desc}")
        print(f"Robot description topic: {robot_description_topic_name}")
        print(f"Joint states topic: {joint_state_topic_name}")
        print(f"Scan topic: {scan_state_topic_name}")
        print(f"IMU topic: {imu_state_topic_name}")


        controller_manager_name = "/" + name + "/controller_manager"
        spawn_controller_1_name = name + "_spawn_controller_joint_state_broadcaster"
        spawn_controller_2_name = name + "_spawn_controller_forward_position_controller"
        spawn_controller_3_name = name + "_spawn_controller_velocity_controller"
        spawn_controller_4_name = name + "_spawn_controller_rlcar_gazebo_controller"
        spawn_controller_5_name = name + "_spawn_controller_rlcar_gazebo_odometry"



            # Robot state publisher node
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[params],
            # remappings=remappings
        )

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', namespace + '/robot_description',
                '-x', robot['x_pose'], 
                '-y', robot['y_pose'], 
                '-z', '0.06', 
                '-Y', '0.78535', 
                '-entity', name,
                '-robot_namespace', namespace,
            ],
            output='screen',
        )

        # Контроллеры и другие узлы
        spawn_controller_1 = Node(
            package="controller_manager",
            executable="spawner",
            name=spawn_controller_1_name,
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
            output="screen",
        )

        spawn_controller_2 = Node(
            package="controller_manager",
            executable="spawner",
            name=spawn_controller_2_name,
            namespace=namespace,
            arguments=["forward_position_controller"],
            output="screen",
        )

        spawn_controller_3 = Node(
            package="controller_manager",
            executable="spawner",
            name=spawn_controller_3_name,
            namespace=namespace,
            arguments=["velocity_controller"],
            output="screen",
        )

        rlcar_gazebo_controller = Node(
            package='rlcar_gazebo_controller',
            executable='rlcar_gazebo_controller',
            name=spawn_controller_4_name,
            namespace=namespace,
            output='screen'
        )

        odometry_node = Node(
            package='rlcar_gazebo_odometry',
            executable='rlcar_gazebo_odometry',
            name = spawn_controller_5_name,
            namespace=namespace,
            output='screen',
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
                {'battery_state_topic': '/battery_state'},
                {'update_frequency': 20.0},
                {'publish_frequency': 2.0},
                {'use_sim_time': True}
            ]
        )
        battery_state = Node(
                package="robot_control",
                executable="battery_simulator",
                namespace=namespace,
        )
        robot_control = Node(
            package="robot_control",
            executable="fake_action_server",
            #name="simple_path_planner",
            namespace=namespace,
            parameters=[{"robot_name":name},
                        {"initial_x": robot['x_pose']},
                        {"initial_y":robot['y_pose']}],
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_warehouse_bot, 'launch','nav2', 'bringup_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'map': os.path.join(pkg_warehouse_bot, 'maps', 'warehouse_map.yaml'),
                'use_namespace': 'True',
                'params_file': params_file,
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_warehouse_bot,'launch', "rviz_launch.py")),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": 'true',
                    "rviz_config": os.path.join(pkg_warehouse_bot, 'rviz', 'multi_nav2_default_view.rviz'),
                }.items()
        )

                # Group all related actions
        robot_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/scan2", dst="scan2"),
            spawn_entity,
            robot_state_publisher,
            odometry_node,            
            #battery_state,
            # robot_control
            # rlcar_gazebo_controller,
        ])

        controllers_action = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            spawn_controller_1,
            spawn_controller_2,
            spawn_controller_3,
            rlcar_gazebo_controller,
        ])

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.06}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'
        

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', 
                namespace + '/initialpose',  # Correctly concatenate the namespace with the topic name
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )

        rviz_action = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            rviz,
            # initial_pose_cmd

        ])
        nav2_actions = GroupAction([
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                bringup_cmd,
                initial_pose_cmd

        ])

        rmf_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            fleet_client,
            # robot_control
        ])

        if last_action is None:
            ld.add_action(robot_actions)
            ld.add_action(controllers_action)
            ld.add_action(rviz_action)
            ld.add_action(nav2_actions)
            ld.add_action(rmf_actions)
        else:
            spawn_event_handler = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                                rmf_actions,
                                nav2_actions,
                                rviz_action,
                                controllers_action,
                                robot_actions,
                                ],
                )
            )
            ld.add_action(spawn_event_handler)

        last_action = spawn_entity
        


    return ld