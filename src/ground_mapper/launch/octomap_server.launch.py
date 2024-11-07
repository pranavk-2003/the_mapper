from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'odom',       # Frame for the 3D map
                'resolution': 0.1,        # Set resolution (adjust based on performance needs)
                'sensor_model/max_range': 10.0  # Adjust based on LiDAR range
            }],
            remappings=[
                ('/cloud_in', '/assembled_cloud')  # Input from our point cloud publisher
            ]
        )
    ])
