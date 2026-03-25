import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, LogInfo
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_robot_api_params
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="false",
        description="가짜 하드웨어(mock_components) 사용 여부"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="RViz2 실행 여부"
    )
    spec_arg = DeclareLaunchArgument(
        "spec", default_value="kaair_specs_01.yaml",
        description="로봇 하드웨어 스펙 파일 (.yaml)"
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_gui")
    spec = LaunchConfiguration("spec")

    # 2. 경로 설정
    pkg_name = 'kaair_moveit_config'
    hw_spec_file = PathJoinSubstitution([
        FindPackageShare('kaair_bringup'), 'config', 'robots', spec
    ])

    # 3. MoveIt 설정 빌드 (URDF/SRDF/컨트롤러 파라미터 추출)
    moveit_config = (
        MoveItConfigsBuilder("kaair", package_name=pkg_name)
        .robot_description(
            file_path=os.path.join(get_package_share_directory("kaair_description"), "urdf", "robot.urdf.xacro"),
            mappings={
                "use_fake_hardware": use_fake_hardware, 
                "mode": "robot",
                "hw_spec_file": hw_spec_file,
            }
        )
        .robot_description_semantic(file_path="config/kaair.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"], default_planning_pipeline="ompl")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # 4. xArm API 파라미터 로드
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # 5. 주요 노드 정의
    # [A] MoveGroup
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # [B] ROS 2 Control Node (Controller Manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("kaair_controller"), "config", "kaair_controllers.yaml"),
            robot_params,
        ],
        output="both",
    )

    # [C] Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


    # ★ orbbec_camera femto_bolt 런치파일 포함
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'), 'launch', 'femto_bolt.launch.py'
            ])
        ])
    )

    # [D] RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("kaair_bringup"), "rviz", "slamtec_moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui)
    )

    # 6. Spawners (Broadcaster 및 컨트롤러)
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    
    # 순차 실행을 위해 개별적으로 선언
    lift_spawner = Node(package="controller_manager", executable="spawner", arguments=["lift_controller"])
    arm_spawner = Node(package="controller_manager", executable="spawner", arguments=["xarm7_traj_controller"])
    head_spawner = Node(package="controller_manager", executable="spawner", arguments=["head_controller"])
    tool_spawner = Node(package="controller_manager", executable="spawner", arguments=["tool_controller"])

    # 7. 이벤트 핸들러 (연쇄 실행 로직)
    # 1. 매니저(ros2_control) 시작 -> JSB 실행
    event_manager_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[jsb_spawner])
    )

    # 2. JSB 시작 -> 나머지 모든 컨트롤러 실행
    event_jsb_to_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=jsb_spawner, 
            on_start=[lift_spawner, arm_spawner, head_spawner, tool_spawner]
        )
    )

    # 3. Arm 컨트롤러 시작 -> MoveGroup 실행 (플래닝 준비 완료)
    event_arm_to_movegroup = RegisterEventHandler(
        OnProcessStart(target_action=arm_spawner, on_start=[move_group_node])
    )

    # 최종 실행 리스트
    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        spec_arg,

        robot_state_pub_node,
        ros2_control_node,
        orbbec_launch,
        rviz_node,

        event_manager_to_jsb,
        event_jsb_to_spawners,
        event_arm_to_movegroup
    ])