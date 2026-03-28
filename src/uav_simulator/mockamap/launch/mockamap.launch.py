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
                'z_length': 3,
                'type': 1,
                # perlin noise parameters
                'complexity': 0.03,
                'fill': 0.3,
                'fractal': 1,
                'attenuation': 0.1,
                # random map parameters
                'width_min': 0.6,
                'width_max': 1.5,
                'obstacle_number': 50,
                # maze parameters
                'road_width': 0.5,
                'add_wall_x': 0,
                'add_wall_y': 1,
                'maze_type': 1,
                # maze 3d parameters
                'numNodes': 40,
                'connectivity': 0.8,
                'nodeRad': 1,
                'roadRad': 10,
            }],
        ),
    ])
