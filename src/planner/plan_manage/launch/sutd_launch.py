import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    os.environ['FORMATION_YAML'] = 'sutd_formation.yaml'
    pkg_share = get_package_share_directory('ego_planner')

    map_size_x = DeclareLaunchArgument('map_size_x', default_value='100.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='20.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')

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

    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map.x_size': 50.0},
            {'map.y_size': 10.0},
            {'map.z_size': 3.0},
            {'map.resolution': 0.1},
            {'ObstacleShape.seed': 1},
            {'map.obs_num': 30},
            {'ObstacleShape.lower_rad': 0.4},
            {'ObstacleShape.upper_rad': 0.5},
            {'ObstacleShape.lower_hei': 2.0},
            {'ObstacleShape.upper_hei': 3.0},
            {'map.circle_num': 8},
            {'ObstacleShape.radius_l': 0.8},
            {'ObstacleShape.radius_h': 1.0},
            {'ObstacleShape.z_l': 0.7},
            {'ObstacleShape.z_h': 3.0},
            {'ObstacleShape.theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 2.0},
        ],
    )

    def make_drone(did, ix, iy, iz, tx, ty, tz):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'run_in_sim_launch.py')
            ),
            launch_arguments={
                'drone_id': str(did),
                'init_x': str(ix), 'init_y': str(iy), 'init_z': str(iz),
                'target_x': str(tx), 'target_y': str(ty), 'target_z': str(tz),
                'map_size_x': LaunchConfiguration('map_size_x'),
                'map_size_y': LaunchConfiguration('map_size_y'),
                'map_size_z': LaunchConfiguration('map_size_z'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }.items(),
        )

    # 29-drone SUTD formation: S(6) + U(9) + T(6) + D(8)
    sc = 1.5
    vertices = [
        # S (0-5)
        (-6.2414, 1.4138), (-5.2414, 1.4138), (-6.2414, 0.4138),
        (-6.2414,-0.5862), (-5.2414,-0.5862), (-5.2414,-1.5862),
        # U (6-14)
        (-3.2414, 1.4138), (-1.2414, 1.4138), (-3.2414, 0.4138),
        (-1.2414, 0.4138), (-3.2414,-0.5862), (-1.2414,-0.5862),
        (-3.2414,-1.5862), (-2.2414,-1.5862), (-1.2414,-1.5862),
        # T (15-20)
        ( 0.7586, 1.4138), ( 1.7586, 1.4138), ( 2.7586, 1.4138),
        ( 1.7586, 0.4138), ( 1.7586,-0.5862), ( 1.7586,-1.5862),
        # D (21-28)
        ( 4.7586, 1.4138), ( 5.7586, 1.4138), ( 4.7586, 0.4138),
        ( 6.7586, 0.4138), ( 4.7586,-0.5862), ( 6.7586,-0.5862),
        ( 4.7586,-1.5862), ( 5.7586,-1.5862),
        # Extra S bottom-left (drone 29)
        (-6.2414,-1.5862),
    ]

    drones = []
    for i, (vx, vy) in enumerate(vertices):
        ix = -30 + sc * vx
        iy = sc * vy
        tx = 25 + sc * vx
        ty = sc * vy
        drones.append(make_drone(i, round(ix, 2), round(iy, 2), 0.5,
                                    round(tx, 2), round(ty, 2), 0.5))

    return LaunchDescription([
        map_size_x, map_size_y, map_size_z, odom_topic,
        swarm_bridge_launch, map_generator_node,
    ] + drones)
