from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Declare all arguments
    init_x_ = DeclareLaunchArgument('init_x_')
    init_y_ = DeclareLaunchArgument('init_y_')
    init_z_ = DeclareLaunchArgument('init_z_')
    obj_num = DeclareLaunchArgument('obj_num', default_value='1')
    map_size_x_ = DeclareLaunchArgument('map_size_x_')
    map_size_y_ = DeclareLaunchArgument('map_size_y_')
    map_size_z_ = DeclareLaunchArgument('map_size_z_')
    c_num = DeclareLaunchArgument('c_num', default_value='0')
    p_num = DeclareLaunchArgument('p_num', default_value='0')
    min_dist = DeclareLaunchArgument('min_dist', default_value='0.8')
    odometry_topic = DeclareLaunchArgument('odometry_topic')
    drone_id = DeclareLaunchArgument('drone_id')

    # poscmd_2_odom node
    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=['drone_', LaunchConfiguration('drone_id'), '_poscmd_2_odom'],
        output='screen',
        parameters=[
            {'init_x': PythonExpression(["float(", LaunchConfiguration('init_x_'), ")"])},
            {'init_y': PythonExpression(["float(", LaunchConfiguration('init_y_'), ")"])},
            {'init_z': PythonExpression(["float(", LaunchConfiguration('init_z_'), ")"])},
        ],
        remappings=[
            ('~/command', ['drone_', LaunchConfiguration('drone_id'), '_planning/pos_cmd']),
            ('~/odometry', ['drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('odometry_topic')]),
        ],
    )

    # odom_visualization node
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', LaunchConfiguration('drone_id'), '_odom_visualization'],
        output='screen',
        parameters=[
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': PythonExpression(["int(", LaunchConfiguration('drone_id'), ")"])},
        ],
        remappings=[
            ('odom', ['drone_', LaunchConfiguration('drone_id'), '_visual_slam/odom']),
            ('robot', ['drone_', LaunchConfiguration('drone_id'), '_odom_visualization/robot']),
            ('path', ['drone_', LaunchConfiguration('drone_id'), '_odom_visualization/path']),
            ('trajectory', ['drone_', LaunchConfiguration('drone_id'), '_odom_visualization/trajectory']),
        ],
    )

    # pcl_render_node (local_sensing_node)
    pcl_render_node = Node(
        package='local_sensing_node',
        executable='pcl_render_node',
        name=['drone_', LaunchConfiguration('drone_id'), '_pcl_render_node'],
        output='screen',
        parameters=[
            {'sensing_horizon': 5.0},
            {'sensing_rate': 30.0},
            {'estimation_rate': 30.0},
            {'map.x_size': PythonExpression(["float(", LaunchConfiguration('map_size_x_'), ")"])},
            {'map.y_size': PythonExpression(["float(", LaunchConfiguration('map_size_y_'), ")"])},
            {'map.z_size': PythonExpression(["float(", LaunchConfiguration('map_size_z_'), ")"])},
        ],
        remappings=[
            ('~/global_map', '/map_generator/global_cloud'),
            ('~/odometry', ['/drone_', LaunchConfiguration('drone_id'), '_', LaunchConfiguration('odometry_topic')]),
            ('~/pcl_render_node/cloud', ['/drone_', LaunchConfiguration('drone_id'), '_pcl_render_node/cloud']),
        ],
    )

    return LaunchDescription([
        init_x_,
        init_y_,
        init_z_,
        obj_num,
        map_size_x_,
        map_size_y_,
        map_size_z_,
        c_num,
        p_num,
        min_dist,
        odometry_topic,
        drone_id,
        poscmd_2_odom_node,
        odom_visualization_node,
        pcl_render_node,
    ])
