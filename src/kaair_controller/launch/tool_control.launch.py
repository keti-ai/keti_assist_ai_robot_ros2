import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware (mock_components)"
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start RViz2 and Joint State Publisher GUI"
    )

    # 2. 인자값 가져오기
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_gui")

    # 3. URDF 로드 (use_fake_hardware 인자를 xacro에 전달)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("kaair_description"),
                "urdf",
                "robot.urdf.xacro"
            ]),
            " ",
            "mode:=robot",
            " ",
            "use_fake_hardware:=", use_fake_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 컨트롤러 파라미터 경로
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("kaair_controller"), "config", "kaair_controllers.yaml"]
    )

    # RViz 설정 파일 경로
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kaair_description"), "rviz", "tool_config.rviz"] # 실제 파일명으로 수정하세요
    )

    # --- 노드 설정 ---

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    tool_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tool_controller"],
    )

    # RViz2 노드 (use_gui가 true일 때만 실행)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_gui)
    )

    # --- 실행 순서 및 리턴 ---

    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        control_node,
        robot_state_pub_node,
        rviz_node,
        # 매니저 실행 후 브로드캐스터 실행
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_node,
                on_start=[jsb_spawner]
            )
        ),
        # 브로드캐스터 실행 후 헤드 컨트롤러 실행
        RegisterEventHandler(
            OnProcessStart(
                target_action=jsb_spawner,
                on_start=[tool_spawner]
            )
        ),
    ])