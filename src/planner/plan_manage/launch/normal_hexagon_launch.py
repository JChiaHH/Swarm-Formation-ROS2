import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ego_planner')

    # Map and common arguments
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='70.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='30.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')
    formation_type = DeclareLaunchArgument('formation_type', default_value='1')

    # Swarm bridge
    swarm_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('swarm_bridge'),
                'launch', 'bridge.launch.py'
            )
        ),
        launch_arguments={
            'drone_id': '999',
            'broadcast_ip': '127.0.0.255',
        }.items(),
    )

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map.x_size': 34.0},
            {'map.y_size': 15.0},
            {'map.z_size': 3.0},
            {'map.resolution': 0.1},
            {'ObstacleShape.seed': 1},
            {'map.obs_num': 30},
            {'ObstacleShape.lower_rad': 0.5},
            {'ObstacleShape.upper_rad': 0.5},
            {'ObstacleShape.lower_hei': 2.0},
            {'ObstacleShape.upper_hei': 3.0},
            {'map.circle_num': 10},
            {'ObstacleShape.radius_l': 1.0},
            {'ObstacleShape.radius_h': 1.2},
            {'ObstacleShape.z_l': 0.7},
            {'ObstacleShape.z_h': 3.0},
            {'ObstacleShape.theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 1.2},
        ],
    )

    # Swarm robots (7 drones: 0-6)
    def make_drone_launch(drone_id_val, init_x_val, init_y_val, init_z_val,
                          target_x_val, target_y_val, target_z_val):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'run_in_sim_launch.py')
            ),
            launch_arguments={
                'drone_id': str(drone_id_val),
                'init_x': str(init_x_val),
                'init_y': str(init_y_val),
                'init_z': str(init_z_val),
                'target_x': str(target_x_val),
                'target_y': str(target_y_val),
                'target_z': str(target_z_val),
                'map_size_x': LaunchConfiguration('map_size_x'),
                'map_size_y': LaunchConfiguration('map_size_y'),
                'map_size_z': LaunchConfiguration('map_size_z'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }.items(),
        )

    drone0 = make_drone_launch(0, -26.0, 0.0, 0.5, 26.0, 0.0, 0.5)
    drone1 = make_drone_launch(1, -23.4, -1.5, 0.5, 28.6, -1.5, 0.5)
    drone2 = make_drone_launch(2, -26.0, -3.0, 0.5, 26.0, -3.0, 0.5)
    drone3 = make_drone_launch(3, -28.6, -1.5, 0.5, 23.4, -1.5, 0.5)
    drone4 = make_drone_launch(4, -28.6, 1.5, 0.5, 23.4, 1.5, 0.5)
    drone5 = make_drone_launch(5, -26.0, 3.0, 0.5, 26.0, 3.0, 0.5)
    drone6 = make_drone_launch(6, -23.4, 1.5, 0.5, 28.6, 1.5, 0.5)

    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        odom_topic,
        formation_type,
        swarm_bridge_launch,
        map_generator_node,
        drone0,
        drone1,
        drone2,
        drone3,
        drone4,
        drone5,
        drone6,
    ])
