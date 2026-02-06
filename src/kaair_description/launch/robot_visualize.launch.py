import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'kaair_description'
    
    # 1. 'mode' 인자 선언
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='robot',
        description='Visualization mode: robot(default), lift, head, arm'
    )
    mode = LaunchConfiguration('mode')

    # 2. URDF 설정 (이전과 동일)
    xacro_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'robot.urdf.xacro'])
    robot_description_content = Command([
        'xacro ', xacro_path, 
        ' mode:=', mode
    ])

    # 3. RViz 설정 파일 경로 분기 (핵심!)
    # mode 값에 따라 'all_config.rviz', 'lift_config.rviz' 등을 선택함
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        PythonExpression(["'", mode, "_config.rviz'"])
    ])

    # 4. 노드 정의
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        on_exit=Shutdown()
    )

    return LaunchDescription([
        mode_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])