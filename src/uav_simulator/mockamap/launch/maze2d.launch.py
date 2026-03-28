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
                'seed': 510,
                'update_freq': 1.0,
                'resolution': 0.1,
                'x_length': 20,
                'y_length': 20,
                'z_length': 2,
                'type': 3,
                'road_width': 0.5,
                'add_wall_x': 0,
                'add_wall_y': 0,
                'maze_type': 1,
            }],
        ),
    ])
