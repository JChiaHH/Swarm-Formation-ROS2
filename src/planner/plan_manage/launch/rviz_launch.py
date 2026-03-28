import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ego_planner')
    rviz_config = os.path.join(pkg_share, 'launch', 'default.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
    ])
