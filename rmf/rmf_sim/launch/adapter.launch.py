from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    failover_mode = DeclareLaunchArgument('failover_mode', default_value='true')
    server_uri = DeclareLaunchArgument('server_uri', default_value='http://localhost:8000/_internal')
    map_building = DeclareLaunchArgument('map_building', default_value='/maps/ware_test/ware.building.yaml')
    nav_graph_dir_arg = DeclareLaunchArgument('nav_graph_dir_arg', default_value='/maps/ware_test')

    # Paths to files
    rmf_sim_path = FindPackageShare('rmf_sim').find('rmf_sim')
    viz_config_file = f'{rmf_sim_path}/config/def.rviz'
    config_file = LaunchConfiguration('map_building')
    nav_graph_file = PathJoinSubstitution([LaunchConfiguration('nav_graph_dir_arg'), '0.yaml'])

    # Data generation process
    generate_map_process = ExecuteProcess(
        cmd=[
            "ros2", "run", "rmf_building_map_tools", "building_map_generator",
            "nav", LaunchConfiguration('map_building'), LaunchConfiguration('nav_graph_dir_arg')
        ],
        output="screen",
    )

    # Common launch inclusion
    common_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f'{rmf_sim_path}/rmf.launch.xml'),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'viz_config_file': viz_config_file,
            'config_file': config_file
        }.items()
    )

    # Fleet adapter group
    fleet_adapter_group = GroupAction([
        DeclareLaunchArgument('fleet_name', default_value='v1'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(f'{rmf_sim_path}/fleet_adapter.xml'),
            launch_arguments={
                'fleet_name': LaunchConfiguration('fleet_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'nav_graph_file': nav_graph_file,
                'server_uri': LaunchConfiguration('server_uri')
            }.items()
        )
    ])

    # Event handler to start other processes after generate_map_process completes
    start_other_processes = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_map_process,
            on_exit=[common_launch, fleet_adapter_group]
        )
    )

    return LaunchDescription([
        use_sim_time,
        failover_mode,
        server_uri,
        map_building,
        nav_graph_dir_arg,
        generate_map_process,
        start_other_processes
    ])
