import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Launch Arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Start robot with fake hardware (mock_components)"
    )

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start RViz2 and Joint State Publisher GUI"
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_gui")

    # 2. URDF 및 파라미터 경로 설정
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
            "mode:=lift",
            " ",
            "use_fake_hardware:=", use_fake_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("kaair_controller"), "config", "kaair_controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kaair_description"), "rviz", "lift_config.rviz"]
    )

    # 3. 공통 노드
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_gui)
    )

    # ==========================================================
    # 🌟 4. 실행 흐름 분리를 위한 핵심 노드들
    # ==========================================================
    
    # [A] 단독 초기화 노드: 가짜 하드웨어가 "아닐 때(Unless)"만 실행됨
    lift_initializer_node = Node(
        package="kaair_driver",
        executable="lift_initializer",
        name="lift_initializer",
        output="screen",
        condition=UnlessCondition(use_fake_hardware)
    )

    # [B] 컨트롤 매니저 (Fake 모드용): 가짜 하드웨어일 때 "즉시" 실행됨
    control_node_fake = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=IfCondition(use_fake_hardware)
    )

    # [C] 컨트롤 매니저 (Real 모드용): 조건문 없이, 아래의 Event로만 실행됨
    control_node_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # 스포너 노드
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    lift_spawner = Node(package="controller_manager", executable="spawner", arguments=["lift_controller"])

    # ==========================================================
    # 🌟 5. 도미노 실행 로직 (Event Handlers)
    # ==========================================================
    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        robot_state_pub_node,
        rviz_node,

        # 기본 런처: Fake면 control_node_fake 시작, Real이면 lift_initializer_node 시작
        control_node_fake,
        lift_initializer_node,

        # 💡 [핵심] Real 모드: 초기화 노드가 '완료(Exit)'되면 그제서야 control_node_real 실행!
        RegisterEventHandler(
            OnProcessExit(
                target_action=lift_initializer_node,
                on_exit=[
                    LogInfo(msg="✅ 호밍 초기화 완료! ROS 2 컨트롤러 매니저를 시작합니다..."),
                    control_node_real
                ]
            )
        ),

        # 💡 스포너 연쇄 실행 1 (Fake 컨트롤러 시작 시)
        RegisterEventHandler(
            OnProcessStart(target_action=control_node_fake, on_start=[jsb_spawner])
        ),
        # 💡 스포너 연쇄 실행 2 (Real 컨트롤러 시작 시)
        RegisterEventHandler(
            OnProcessStart(target_action=control_node_real, on_start=[jsb_spawner])
        ),
        
        # 💡 컨트롤러 스포너 충돌 방지: jsb_spawner 설정이 끝난(Exit) 직후 lift_spawner 실행
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner, on_exit=[lift_spawner])
        ),
    ])