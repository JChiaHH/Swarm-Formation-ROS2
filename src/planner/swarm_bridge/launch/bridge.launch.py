from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        description='ID of the drone'
    )

    broadcast_ip_arg = DeclareLaunchArgument(
        'broadcast_ip',
        description='UDP broadcast IP address'
    )

    drone_id = LaunchConfiguration('drone_id')
    broadcast_ip = LaunchConfiguration('broadcast_ip')

    bridge_node = Node(
        package='swarm_bridge',
        executable='bridge_node',
        name=['drone_', drone_id, '_bridge_node'],
        output='screen',
        parameters=[{
            'broadcast_ip': broadcast_ip,
            'drone_id': PythonExpression(["int(", drone_id, ")"]),
            'odom_max_freq': 70.0,
        }],
        remappings=[
            ('my_odom', '/vins_estimator/imu_propagate'),
        ],
    )

    return LaunchDescription([
        drone_id_arg,
        broadcast_ip_arg,
        bridge_node,
    ])
