from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare  # Corrected import

def generate_launch_description():
    # Define paths to URDF and RViz config
    urdf_path = PathJoinSubstitution([
        FindPackageShare("mapper_description"), "urdf", "mapper.urdf.xacro"
    ])
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "rviz", "u1.rviz"
    ])
    world_path = PathJoinSubstitution([
        FindPackageShare("mapper_bringup"), "worlds", "env.world"
    ])

    # Define nodes and launch descriptions
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_path])
            }]
        ),
        
        # Gazebo Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
                ])
            ]),
            launch_arguments={"world": world_path}.items()
        ),

        # Spawn Entity in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "mapper"]
        ),

        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        ),
        Node(
            package='ground_mapper',
            executable='mapping_node',
            name='mapping_node',
            output='screen'
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',  # Use 'octomap_server_node' executable directly
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'odom',
                'resolution': 0.1,
                'sensor_model/max_range': 5.0,
                'latch': False, 
                'height_map': True, 

            }],
            remappings=[
                ('/cloud_in', '/assembled_cloud')  # Ensure this matches the output topic of mapping_node
            ]
        )
    ])
