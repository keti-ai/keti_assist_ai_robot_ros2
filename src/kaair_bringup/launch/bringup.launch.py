import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# xArm의 파라미터 파싱 유틸리티 임포트
from uf_ros_lib.uf_robot_utils import generate_robot_api_params

def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument("use_fake_hardware", default_value="true")
    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="true")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_gui")

    # 2. 패키지 경로 설정
    kaair_desc_pkg = FindPackageShare("kaair_description")
    kaair_control_pkg = FindPackageShare("kaair_controller")

    # 3. 마스터 URDF 로드 (전체 로봇 모드)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([kaair_desc_pkg, "urdf", "robot.urdf.xacro"]), " ",
        "mode:=robot", " ",
        "use_fake_hardware:=", use_fake_hardware,
    ])
    robot_description = {"robot_description": robot_description_content}

    # 4. 컨트롤러 파라미터 설정
    body_controllers = PathJoinSubstitution([kaair_control_pkg, "config", "kaair_controllers.yaml"])
    arm_controllers = PathJoinSubstitution([kaair_control_pkg, "config", "arm_controllers.yaml"])
    
    # xArm 전용 파라미터
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # ==========================================================
    # [공통] Robot State Publisher (시스템 전체에 딱 1개!)
    # ==========================================================
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ==========================================================
    # [Body 매니저 영역] 리프트, 헤드, 툴
    # ==========================================================
    body_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, body_controllers],
        output="both",
    )

    body_jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager","--param-file", body_controllers])
    lift_spawner = Node(package="controller_manager", executable="spawner", arguments=["lift_controller", "-c", "/controller_manager"])
    head_spawner = Node(package="controller_manager", executable="spawner", arguments=["head_controller", "-c", "/controller_manager"])
    tool_spawner = Node(package="controller_manager", executable="spawner", arguments=["tool_controller", "-c", "/controller_manager"])

    # ==========================================================
    # [Arm 매니저 영역] xArm (네임스페이스: /arm)
    # ==========================================================
    arm_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="arm", # 네임스페이스 격리!
        parameters=[robot_description, arm_controllers, robot_params],
        output="both",
        # 핵심: /arm/joint_states로 나가는 데이터를 /joint_states로 합쳐줌
        remappings=[("/arm/joint_states", "/joint_states")] 
    )

    arm_jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/arm/controller_manager", "--param-file", arm_controllers])
    arm_traj_spawner = Node(package="controller_manager", executable="spawner", arguments=["xarm7_traj_controller", "-c", "/arm/controller_manager"])

    # ==========================================================
    # RViz2 노드
    # ==========================================================
    rviz_config_file = PathJoinSubstitution([kaair_desc_pkg, "rviz", "robot_config.rviz"])
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file], condition=IfCondition(use_gui)
    )

    # --- 실행 순서 조율 ---
    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        robot_state_pub_node,
        
        # 1. 두 개의 매니저를 동시에 실행
        body_control_node,
        arm_control_node,
        rviz_node,
        
        # 2. Body Spawner 순차 실행
        RegisterEventHandler(OnProcessStart(target_action=body_control_node, on_start=[body_jsb_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=body_jsb_spawner, on_start=[lift_spawner, head_spawner, tool_spawner])),

        # 3. Arm Spawner 순차 실행
        RegisterEventHandler(OnProcessStart(target_action=arm_control_node, on_start=[arm_jsb_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=arm_jsb_spawner, on_start=[arm_traj_spawner])),
    ])