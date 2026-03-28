from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[{
                'seed': 511,
                'update_freq': 1.0,
                'resolution': 0.1,
                'x_length': 10,
                'y_length': 10,
                'z_length': 4,
                'type': 2,
                'width_min': 0.6,
                'width_max': 1.5,
                'obstacle_number': 50,
            }],
        ),
    ])
