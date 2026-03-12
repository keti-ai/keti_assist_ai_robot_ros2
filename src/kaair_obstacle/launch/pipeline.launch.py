from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # ── params.yaml 경로 ───────────────────────────────────────────
    params_file = PathJoinSubstitution([
        FindPackageShare('kaair_obstacle'),
        'config',
        'params.yaml'
    ])

    # ── 메인 노드 ──────────────────────────────────────────────────
    obstacle_node = Node(
        package='kaair_obstacle',
        executable='obstacle_representation',
        name='obstacle_representation_node',
        output='screen',
        parameters=[
            params_file,
            # launch 인자로 토픽 오버라이드 가능
            # {'input_topic': LaunchConfiguration('input_topic')}
        ],
        # 로그 레벨: DEBUG / INFO / WARN / ERROR
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([
        FindPackageShare('kaair_obstacle'),
        'config',
        'rviz_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # rviz 설정 파일이 있으면 자동 로드
        # arguments=['-d', rviz_config],
        condition=None,   # use_rviz 조건 필요 시 IfCondition 사용
    )

    return LaunchDescription([
        input_topic_arg,
        use_rviz_arg,
        obstacle_node,
        # rviz_node,   # RViz2 자동 실행 필요 시 주석 해제
    ])