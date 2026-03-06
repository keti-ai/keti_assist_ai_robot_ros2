import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # 파라미터 파일 경로
    params_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'config', 'params.yaml'
    )

    obstacle_representation_node = Node(
        package='kaair_obstacle',
        executable='obstacle_representation',
        name='obstacle_representation_node',
        namespace='obstacle',
        output='screen',
        parameters=[params_file],
        remappings=[
            # 필요시 토픽 리매핑
            # ('/camera/depth/color/points', '/your_actual_topic'),
        ]
    )

    return LaunchDescription([
        obstacle_representation_node,
    ])