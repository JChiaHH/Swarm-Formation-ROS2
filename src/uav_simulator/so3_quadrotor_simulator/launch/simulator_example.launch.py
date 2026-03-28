import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('so3_quadrotor_simulator')

    init_x_arg = DeclareLaunchArgument('init_x', default_value='-4')
    init_y_arg = DeclareLaunchArgument('init_y', default_value='0')
    init_z_arg = DeclareLaunchArgument('init_z', default_value='4')

    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')

    # Simulator
    quadrotor_simulator_so3_node = Node(
        package='so3_quadrotor_simulator',
        executable='quadrotor_simulator_so3',
        name='quadrotor_simulator_so3',
        output='screen',
        parameters=[{
            'rate.odom': 100.0,
            'simulator.init_state_x': init_x,
            'simulator.init_state_y': init_y,
            'simulator.init_state_z': init_z,
        }],
        remappings=[
            ('odom', '/sim/odom'),
            ('cmd', '/so3_cmd'),
            ('imu', '/sim/imu'),
        ],
    )

    # Controller
    so3_control_node = Node(
        package='so3_control',
        executable='so3_control_node',
        name='so3_control',
        output='screen',
        parameters=[{
            'so3_control.init_state_x': init_x,
            'so3_control.init_state_y': init_y,
            'so3_control.init_state_z': init_z,
            'mass': 0.98,
            'use_angle_corrections': False,
            'use_external_yaw': False,
            'gains.rot.z': 1.0,
            'gains.ang.z': 0.1,
        }],
        remappings=[
            ('odom', '/sim/odom'),
            ('position_cmd', '/position_cmd'),
            ('motors', 'motors'),
            ('corrections', 'corrections'),
            ('so3_cmd', '/so3_cmd'),
            ('imu', '/sim/imu'),
        ],
    )

    # Visualization
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[{
            'color.a': 0.5,
            'color.r': 1.0,
            'color.g': 0.0,
            'color.b': 0.0,
            'covariance_scale': 100.0,
            'robot_scale': 1.0,
            'tf45': True,
        }],
        remappings=[
            ('odom', '/sim/odom'),
        ],
    )

    # RViz
    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        init_x_arg,
        init_y_arg,
        init_z_arg,
        quadrotor_simulator_so3_node,
        so3_control_node,
        odom_visualization_node,
        rviz_node,
    ])
