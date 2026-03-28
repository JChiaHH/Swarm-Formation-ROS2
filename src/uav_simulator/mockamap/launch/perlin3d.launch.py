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
                'resolution': 0.15,
                'x_length': 40,
                'y_length': 20,
                'z_length': 5,
                'type': 1,
                'complexity': 0.07,
                'fill': 0.1,
                'fractal': 1,
                'attenuation': 0.1,
            }],
        ),
    ])
