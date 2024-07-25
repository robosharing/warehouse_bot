#!/usr/bin/python3

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):

    robot_name = LaunchConfiguration('robot_name').perform(context)
    spawn_controller_1_name = robot_name + "_spawn_controller_joint_state_broadcaster"
    spawn_controller_2_name = robot_name + "_spawn_controller_forward_position_controller"
    spawn_controller_3_name = robot_name + "_spawn_controller_velocity_controller"
    spawn_controller_4_name = robot_name + "_spawn_controller_rlcar_gazebo_controller"
    spawn_controller_5_name = robot_name + "_spawn_controller_rlcar_gazebo_odometryr"
    controller_manager_name = "controller_manager"


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

    # Car controller launch
    rlcar_gazebo_controller = Node(
        package='rlcar_gazebo_controller',
        executable='rlcar_gazebo_controller',
        name=spawn_controller_4_name,
        namespace=robot_name,
        output='screen',
        parameters=[],
        arguments=["--controller-manager", controller_manager_name],
    )

        # car-like robot odometry node 
    rlcar_gazebo_odometry = Node(
        package='rlcar_gazebo_odometry',
        executable='rlcar_gazebo_odometry',
        name=spawn_controller_5_name,
        namespace=robot_name,
        output='log',
        parameters=[{
            "verbose" : False,
            'publish_rate' : 50,
            'open_loop' : False,
            'has_imu_heading' : True,
            'is_gazebo' : True,
            'wheel_radius' : 0.075,
            'base_frame_id': LaunchConfiguration('robot_name').perform(context) + "_base_footprint",
            'odom_frame_id' : "odom",
            'enable_odom_tf' : True,
        }],
    )


    return [spawn_controller_1, spawn_controller_2, spawn_controller_3, rlcar_gazebo_controller , rlcar_gazebo_odometry]


def generate_launch_description(): 

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot')

    return LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function = launch_setup)
        ])