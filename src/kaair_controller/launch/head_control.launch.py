import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # URDF 로드 (kaair_description 패키지가 있다고 가정)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("kaair_description"), # 또는 URDF가 있는 패키지명
                "urdf", 
                "robot.urdf.xacro"
            ]),
            " ",
            "mode:=head",  # <--- xacro 내부의 <xacro:arg name="mode" default="..."/> 에 전달됨
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 컨트롤러 파라미터 경로
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("kaair_controller"), "config", "head_controllers.yaml"]
    )

    # 노드 1: ros2_control_node (메인 매니저)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 노드 2: robot_state_publisher (TF 발행)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 노드 3: Spawner (Joint State Broadcaster)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 노드 4: Spawner (Head Controller)
    head_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller"],
    )

    # 순차 실행 설정 (매니저 뜬 후 브로드캐스터 -> 그 후 컨트롤러)
    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        RegisterEventHandler(OnProcessStart(target_action=control_node, on_start=[jsb_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=jsb_spawner, on_start=[head_spawner])),
    ])