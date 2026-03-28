import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ego_planner')

    # Declare all arguments
    map_size_x = DeclareLaunchArgument('map_size_x')
    map_size_y = DeclareLaunchArgument('map_size_y')
    map_size_z = DeclareLaunchArgument('map_size_z')
    init_x = DeclareLaunchArgument('init_x')
    init_y = DeclareLaunchArgument('init_y')
    init_z = DeclareLaunchArgument('init_z')

    point0_x = DeclareLaunchArgument('point0_x', default_value='0.0')
    point0_y = DeclareLaunchArgument('point0_y', default_value='0.0')
    point0_z = DeclareLaunchArgument('point0_z', default_value='0.0')
    point1_x = DeclareLaunchArgument('point1_x', default_value='0.0')
    point1_y = DeclareLaunchArgument('point1_y', default_value='0.0')
    point1_z = DeclareLaunchArgument('point1_z', default_value='0.0')
    point2_x = DeclareLaunchArgument('point2_x', default_value='0.0')
    point2_y = DeclareLaunchArgument('point2_y', default_value='0.0')
    point2_z = DeclareLaunchArgument('point2_z', default_value='0.0')

    target_x = DeclareLaunchArgument('target_x')
    target_y = DeclareLaunchArgument('target_y')
    target_z = DeclareLaunchArgument('target_z')
    drone_id = DeclareLaunchArgument('drone_id')
    formation_yaml = DeclareLaunchArgument('formation_yaml', default_value='normal_hexagon.yaml')
    formation_type = DeclareLaunchArgument('formation_type', default_value='1')
    weight_obstacle = DeclareLaunchArgument('weight_obstacle', default_value='50000.0')
    weight_swarm = DeclareLaunchArgument('weight_swarm', default_value='50000.0')
    weight_feasibility = DeclareLaunchArgument('weight_feasibility', default_value='10000.0')
    weight_sqrvariance = DeclareLaunchArgument('weight_sqrvariance', default_value='10000.0')
    weight_time = DeclareLaunchArgument('weight_time', default_value='80.0')
    weight_formation = DeclareLaunchArgument('weight_formation', default_value='15000.0')
    obstacle_clearance = DeclareLaunchArgument('obstacle_clearance', default_value='0.5')
    swarm_clearance = DeclareLaunchArgument('swarm_clearance', default_value='0.5')
    replan_trajectory_time = DeclareLaunchArgument('replan_trajectory_time', default_value='0.1')

    odom_topic = DeclareLaunchArgument('odom_topic')

    # Include advanced_param_launch.py
    advanced_param_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'advanced_param_launch.py')
        ),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'formation_yaml': LaunchConfiguration('formation_yaml'),
            'map_size_x_': LaunchConfiguration('map_size_x'),
            'map_size_y_': LaunchConfiguration('map_size_y'),
            'map_size_z_': LaunchConfiguration('map_size_z'),
            'odometry_topic': LaunchConfiguration('odom_topic'),
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            'cx': '321.04638671875',
            'cy': '243.44969177246094',
            'fx': '387.229248046875',
            'fy': '387.229248046875',
            'max_vel': '0.5',
            'max_acc': '3.0',
            'planning_horizon': '7.5',
            'use_distinctive_trajs': 'false',
            'flight_type': '3',
            'point_num': '1',
            'point0_x': LaunchConfiguration('target_x'),
            'point0_y': LaunchConfiguration('target_y'),
            'point0_z': LaunchConfiguration('target_z'),
            'point1_x': '0.0',
            'point1_y': '0.0',
            'point1_z': '0.0',
            'point2_x': '0.0',
            'point2_y': '0.0',
            'point2_z': '0.0',
            'point3_x': '0.0',
            'point3_y': '0.0',
            'point3_z': '0.0',
        }.items(),
    )

    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', LaunchConfiguration('drone_id'), '_traj_server'],
        output='screen',
        parameters=[
            {'traj_server/time_forward': 1.0},
        ],
        remappings=[
            ('position_cmd', ['drone_', LaunchConfiguration('drone_id'), '_planning/pos_cmd']),
            ('planning/trajectory', ['drone_', LaunchConfiguration('drone_id'), '_planning/trajectory']),
            ('planning/start', ['drone_', LaunchConfiguration('drone_id'), '_planning/start']),
            ('planning/finish', ['drone_', LaunchConfiguration('drone_id'), '_planning/finish']),
        ],
    )

    # Include simulator_launch.py
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulator_launch.py')
        ),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'map_size_x_': LaunchConfiguration('map_size_x'),
            'map_size_y_': LaunchConfiguration('map_size_y'),
            'map_size_z_': LaunchConfiguration('map_size_z'),
            'init_x_': LaunchConfiguration('init_x'),
            'init_y_': LaunchConfiguration('init_y'),
            'init_z_': LaunchConfiguration('init_z'),
            'odometry_topic': LaunchConfiguration('odom_topic'),
        }.items(),
    )

    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        init_x,
        init_y,
        init_z,
        point0_x, point0_y, point0_z,
        point1_x, point1_y, point1_z,
        point2_x, point2_y, point2_z,
        target_x, target_y, target_z,
        drone_id,
        formation_yaml,
        formation_type,
        weight_obstacle,
        weight_swarm,
        weight_feasibility,
        weight_sqrvariance,
        weight_time,
        weight_formation,
        obstacle_clearance,
        swarm_clearance,
        replan_trajectory_time,
        odom_topic,
        advanced_param_launch,
        traj_server_node,
        simulator_launch,
    ])
