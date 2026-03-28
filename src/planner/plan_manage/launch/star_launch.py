import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    os.environ['FORMATION_YAML'] = 'star_formation.yaml'
    pkg_share = get_package_share_directory('ego_planner')

    # Map and common arguments
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='70.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='30.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')

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

    # Map generator node — fewer obstacles for 10-drone formation
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
            {'map.obs_num': 12},
            {'ObstacleShape.lower_rad': 0.3},
            {'ObstacleShape.upper_rad': 0.4},
            {'ObstacleShape.lower_hei': 2.0},
            {'ObstacleShape.upper_hei': 3.0},
            {'map.circle_num': 4},
            {'ObstacleShape.radius_l': 0.8},
            {'ObstacleShape.radius_h': 1.0},
            {'ObstacleShape.z_l': 0.7},
            {'ObstacleShape.z_h': 3.0},
            {'ObstacleShape.theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 2.0},
        ],
    )

    # 10 drones in two rows for star formation
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

    # Initial positions: star formation centered at x=-25
    # Star vertices × swarm_scale(2.0) + center(-25, 0, 0.5)
    # Outer tips (r=2.0×2=4.0): drones 0,2,4,6,8
    # Inner valleys (r=0.8×2=1.6): drones 1,3,5,7,9
    # Init positions: unit_pos × swarm_scale(2.0) + center(-25, 0, 0.5)
    # Inner radius=1.2, outer radius=2.0 at unit scale
    drone0  = make_drone_launch(0,  -25.0,   4.0,   0.5,  25.0,  4.0,   0.5)   # top tip
    drone1  = make_drone_launch(1,  -26.41,  1.942, 0.5,  23.59, 1.942, 0.5)   # inner (r=1.2)
    drone2  = make_drone_launch(2,  -28.80,  1.24,  0.5,  21.20, 1.24,  0.5)   # left tip
    drone3  = make_drone_launch(3,  -27.28, -0.742, 0.5,  22.72,-0.742, 0.5)   # inner (r=1.2)
    drone4  = make_drone_launch(4,  -27.35, -3.24,  0.5,  22.65,-3.24,  0.5)   # lower-left tip
    drone5  = make_drone_launch(5,  -25.0,  -2.4,   0.5,  25.0, -2.4,   0.5)   # inner (r=1.2)
    drone6  = make_drone_launch(6,  -22.65, -3.24,  0.5,  27.35,-3.24,  0.5)   # lower-right tip
    drone7  = make_drone_launch(7,  -22.72, -0.742, 0.5,  27.28,-0.742, 0.5)   # inner (r=1.2)
    drone8  = make_drone_launch(8,  -21.20,  1.24,  0.5,  28.80, 1.24,  0.5)   # right tip
    drone9  = make_drone_launch(9,  -23.59,  1.942, 0.5,  26.41, 1.942, 0.5)   # inner (r=1.2)

    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        odom_topic,
        swarm_bridge_launch,
        map_generator_node,
        drone0, drone1, drone2, drone3, drone4,
        drone5, drone6, drone7, drone8, drone9,
    ])
