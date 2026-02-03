import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('keti_assist_ai_robot_description')

    # xacro 파일 경로
    xacro_file = os.path.join(pkg_share, 'urdf', 'lift.urdf.xacro')

    # Xacro를 파이썬에서 직접 처리
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()  # 실제 URDF XML로 변환

    # RViz 파일 경로
    rviz_file = os.path.join(pkg_share, 'rviz', 'lift_config.rviz')

    # LaunchDescription
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['--display-config', rviz_file],
        ),
    ])
