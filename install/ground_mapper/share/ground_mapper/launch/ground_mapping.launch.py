from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'sensor_model/max_range': 10.0,
                'latch': False, 
                'height_map': True, 

            }],
            remappings=[
                ('/cloud_in', '/assembled_cloud')  # Ensure this matches the output topic of mapping_node
            ]
        )
    ])
