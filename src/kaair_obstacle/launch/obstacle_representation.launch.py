import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── config 경로: share/<pkg>/config/params.yaml ──────────────
    pkg_share   = get_package_share_directory('kaair_obstacle')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    obstacle_node = Node(
        package    = 'kaair_obstacle',
        executable = 'obstacle_representation',
        name       = 'obstacle_representation_node',
        output     = 'screen',
        parameters = [params_file],
        remappings = [
            # orbbec 실제 토픽명이 다르면 여기서 리매핑
            # ('/femto/depth_registered/points', '/your_actual_topic'),
        ]
    )

    return LaunchDescription([obstacle_node])