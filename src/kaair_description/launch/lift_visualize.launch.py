import os
from os import environ
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1. 환경 변수 및 기본 설정
    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    pkg_name = 'kaair_description'
    robot_model_name = 'lift'

    # 2. 경로 설정 (PathJoinSubstitution 활용)
    # xacro 파일: urdf/lift/lift.urdf.xacro
    xacro_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        robot_model_name,
        f'{robot_model_name}.urdf.xacro'
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        f'{robot_model_name}_config.rviz'
    ])

    # 3. 노드 정의
    
    # [Robot State Publisher] xacro를 urdf로 변환하여 로봇 상태 발행
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description': Command(['xacro ', xacro_path])
        }]
    )

    # [Joint State Publisher GUI] 조인트 제어 슬라이더 표시
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # [RViz2] 시각화 도구
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        on_exit=Shutdown() # RViz 종료 시 전체 런치 종료
    )

    # 4. 런치 액션 추가
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)
    ld.add_action(rviz_node)

    return ld