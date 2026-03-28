import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ego_planner')
    formation_config = os.path.join(pkg_share, 'config', os.environ.get('FORMATION_YAML', 'normal_hexagon.yaml'))
    # Declare all arguments
    map_size_x_ = DeclareLaunchArgument('map_size_x_')
    map_size_y_ = DeclareLaunchArgument('map_size_y_')
    map_size_z_ = DeclareLaunchArgument('map_size_z_')

    odometry_topic = DeclareLaunchArgument('odometry_topic')
    camera_pose_topic = DeclareLaunchArgument('camera_pose_topic')
    depth_topic = DeclareLaunchArgument('depth_topic')
    cloud_topic = DeclareLaunchArgument('cloud_topic')

    cx = DeclareLaunchArgument('cx')
    cy = DeclareLaunchArgument('cy')
    fx = DeclareLaunchArgument('fx')
    fy = DeclareLaunchArgument('fy')

    max_vel = DeclareLaunchArgument('max_vel')
    max_acc = DeclareLaunchArgument('max_acc')
    planning_horizon = DeclareLaunchArgument('planning_horizon')

    point_num = DeclareLaunchArgument('point_num')
    point0_x = DeclareLaunchArgument('point0_x')
    point0_y = DeclareLaunchArgument('point0_y')
    point0_z = DeclareLaunchArgument('point0_z')
    point1_x = DeclareLaunchArgument('point1_x')
    point1_y = DeclareLaunchArgument('point1_y')
    point1_z = DeclareLaunchArgument('point1_z')
    point2_x = DeclareLaunchArgument('point2_x')
    point2_y = DeclareLaunchArgument('point2_y')
    point2_z = DeclareLaunchArgument('point2_z')
    point3_x = DeclareLaunchArgument('point3_x')
    point3_y = DeclareLaunchArgument('point3_y')
    point3_z = DeclareLaunchArgument('point3_z')

    flight_type = DeclareLaunchArgument('flight_type')
    use_distinctive_trajs = DeclareLaunchArgument('use_distinctive_trajs')
    obj_num_set = DeclareLaunchArgument('obj_num_set', default_value='1')
    drone_id = DeclareLaunchArgument('drone_id')

    # main ego_planner_node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('odometry_topic')]),
            ('planning/trajectory', ['/drone_', LaunchConfiguration('drone_id'), '_planning/trajectory']),
            ('planning/start', ['/drone_', LaunchConfiguration('drone_id'), '_planning/start']),
            ('planning/finish', ['/drone_', LaunchConfiguration('drone_id'), '_planning/finish']),
            ('planning/data_display', ['/drone_', LaunchConfiguration('drone_id'), '_planning/data_display']),
            ('planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
            ('planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
            ('grid_map/odom', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('odometry_topic')]),
            ('grid_map/cloud', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('cloud_topic')]),
            ('grid_map/pose', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('camera_pose_topic')]),
            ('grid_map/depth', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('depth_topic')]),
            ('grid_map/occupancy', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/grid_map/occupancy']),
            ('grid_map/occupancy_inflate', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/grid_map/occupancy_inflate']),
            ('grid_map/esdf', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/grid_map/esdf']),
            ('goal_point', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/goal_point']),
            ('global_list', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/global_list']),
            ('init_list', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/init_list']),
            ('optimal_list', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/optimal_list']),
            ('a_star_list', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/a_star_list']),
            ('swarm_graph_visual', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/swarm_graph_visual']),
            ('swarm_formation_visual', ['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node/swarm_formation_visual']),
        ],
        parameters=[
            # Load the formation yaml config
            formation_config,
            {'use_sim_time': False},

            # planning fsm
            {'fsm/flight_type': PythonExpression(["int(", LaunchConfiguration('flight_type'), ")"])},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 1.0},
            {'fsm/planning_horizon': PythonExpression(["float(", LaunchConfiguration('planning_horizon'), ")"])},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': 1.0},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            {'fsm/result_file': '/home/jeremychia/Swarm-Formation-ROS2/results/ego_swarm_1.txt'},

            {'fsm/waypoint_num': PythonExpression(["int(", LaunchConfiguration('point_num'), ")"])},
            {'fsm/waypoint0_x': PythonExpression(["float(", LaunchConfiguration('point0_x'), ")"])},
            {'fsm/waypoint0_y': PythonExpression(["float(", LaunchConfiguration('point0_y'), ")"])},
            {'fsm/waypoint0_z': PythonExpression(["float(", LaunchConfiguration('point0_z'), ")"])},
            {'fsm/waypoint1_x': PythonExpression(["float(", LaunchConfiguration('point1_x'), ")"])},
            {'fsm/waypoint1_y': PythonExpression(["float(", LaunchConfiguration('point1_y'), ")"])},
            {'fsm/waypoint1_z': PythonExpression(["float(", LaunchConfiguration('point1_z'), ")"])},
            {'fsm/waypoint2_x': PythonExpression(["float(", LaunchConfiguration('point2_x'), ")"])},
            {'fsm/waypoint2_y': PythonExpression(["float(", LaunchConfiguration('point2_y'), ")"])},
            {'fsm/waypoint2_z': PythonExpression(["float(", LaunchConfiguration('point2_z'), ")"])},
            {'fsm/waypoint3_x': PythonExpression(["float(", LaunchConfiguration('point3_x'), ")"])},
            {'fsm/waypoint3_y': PythonExpression(["float(", LaunchConfiguration('point3_y'), ")"])},
            {'fsm/waypoint3_z': PythonExpression(["float(", LaunchConfiguration('point3_z'), ")"])},

            # grid map
            {'grid_map/resolution': 0.1},
            {'grid_map/map_size_x': PythonExpression(["float(", LaunchConfiguration('map_size_x_'), ")"])},
            {'grid_map/map_size_y': PythonExpression(["float(", LaunchConfiguration('map_size_y_'), ")"])},
            {'grid_map/map_size_z': PythonExpression(["float(", LaunchConfiguration('map_size_z_'), ")"])},
            {'grid_map/local_update_range_x': 5.5},
            {'grid_map/local_update_range_y': 5.5},
            {'grid_map/local_update_range_z': 4.5},
            {'grid_map/obstacles_inflation': 0.1},
            {'grid_map/local_map_margin': 10},
            {'grid_map/ground_height': -0.01},
            # camera parameter
            {'grid_map/cx': PythonExpression(["float(", LaunchConfiguration('cx'), ")"])},
            {'grid_map/cy': PythonExpression(["float(", LaunchConfiguration('cy'), ")"])},
            {'grid_map/fx': PythonExpression(["float(", LaunchConfiguration('fx'), ")"])},
            {'grid_map/fy': PythonExpression(["float(", LaunchConfiguration('fy'), ")"])},
            # depth filter
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 5.0},
            {'grid_map/depth_filter_mindist': 0.2},
            {'grid_map/depth_filter_margin': 2},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            # local fusion
            {'grid_map/p_hit': 0.65},
            {'grid_map/p_miss': 0.35},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.90},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': 0.1},
            {'grid_map/max_ray_length': 4.5},

            {'grid_map/virtual_ceil_height': 3.0},
            {'grid_map/visualization_truncate_height': 2.8},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': 1},
            {'grid_map/frame_id': 'world'},

            # sdf map
            {'grid_map/local_bound_inflate': 0.0},
            {'grid_map/show_esdf_time': False},
            {'grid_map/esdf_slice_height': 0.3},

            # planner manager
            {'manager/max_vel': PythonExpression(["float(", LaunchConfiguration('max_vel'), ")"])},
            {'manager/max_acc': PythonExpression(["float(", LaunchConfiguration('max_acc'), ")"])},
            {'manager/control_points_distance': 0.4},
            {'manager/polyTraj_piece_length': 2.0},
            {'manager/feasibility_tolerance': 0.05},
            {'manager/planning_horizon': PythonExpression(["float(", LaunchConfiguration('planning_horizon'), ")"])},
            {'manager/use_distinctive_trajs': PythonExpression(["True if '", LaunchConfiguration('use_distinctive_trajs'), "'.lower() == 'true' else False"])},
            {'manager/drone_id': PythonExpression(["int(", LaunchConfiguration('drone_id'), ")"])},

            # trajectory optimization
            {'optimization/constrain_points_perPiece': 3},
            {'optimization/max_vel': PythonExpression(["float(", LaunchConfiguration('max_vel'), ")"])},
            {'optimization/max_acc': PythonExpression(["float(", LaunchConfiguration('max_acc'), ")"])},
            {'optimization/record_opt': True},
        ],
    )

    return LaunchDescription([
        map_size_x_,
        map_size_y_,
        map_size_z_,
        odometry_topic,
        camera_pose_topic,
        depth_topic,
        cloud_topic,
        cx, cy, fx, fy,
        max_vel,
        max_acc,
        planning_horizon,
        point_num,
        point0_x, point0_y, point0_z,
        point1_x, point1_y, point1_z,
        point2_x, point2_y, point2_z,
        point3_x, point3_y, point3_z,
        flight_type,
        use_distinctive_trajs,
        obj_num_set,
        drone_id,
        ego_planner_node,
    ])
