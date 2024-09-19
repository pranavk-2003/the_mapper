from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare("mapper_description"), "urdf", "mapper.urdf.xacro"
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "rviz", "urdf_config.rviz"
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "worlds", "test_world.world"
    ])
    cartographer_config_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "config"
    ])
    ekf_config_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "config", "ekf_config.yaml"
    ])

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}],
        ),
        # Gazebo Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
            launch_arguments={'world': world_path}.items(),
        ),
        # Gazebo Spawn Entity
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'mapper'],
            output='screen'
        ),
        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_config_path
            ],
        ),
        # Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/imu', '/imu_plugin/out'),
                ('/odom', '/odometry/filtered')  # Use EKF output for odometry
            ],
            arguments=['-configuration_directory', cartographer_config_path, 
                       '-configuration_basename', 'cartographer.lua']
        ),
        # Cartographer Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])