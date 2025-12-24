from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_moving',
            executable='lidar_moving',
            name='lidar_moving_node',
            output='screen',
        )
    ])
