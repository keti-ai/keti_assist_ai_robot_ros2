import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

# xArm의 파라미터 파싱 유틸리티
from uf_ros_lib.uf_robot_utils import generate_robot_api_params

def generate_launch_description():
    description_pkg_path = get_package_share_directory("kaair_description")
    moveit_pkg_path = get_package_share_directory("kaair_moveit_config")
    
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="false")

    # 1. xArm 전용 파라미터 로드
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # 2. MoveIt 설정 빌드 (파라미터 추출용)
    moveit_config = (
        MoveItConfigsBuilder("kaair", package_name="kaair_moveit_config")
        .robot_description(
            file_path=os.path.join(description_pkg_path, "urdf", "robot.urdf.xacro"),
            mappings={"use_fake_hardware": use_fake_hardware, "mode": "robot"}
        )
        .robot_description_semantic(file_path="config/kaair.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"], default_planning_pipeline="ompl")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # 3. [수동] ros2_control_node 실행 (robot_params 포함)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("kaair_controller"), "config", "kaair_controllers.yaml"), # 컨트롤러 설정 파일
            robot_params, # xArm API 파라미터 주입
        ],
        output="both",
    )

# 4. [수동] MoveGroup 노드 실행
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()],
    )

    # 5. [수동] 개별 컨트롤러 Spawner 정의
    # (원하시는 대로 하나씩 직접 지정해서 부릅니다)
    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["xarm7_traj_controller", "--controller-manager", "/controller_manager"],
    )

    lift_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["lift_controller", "--controller-manager", "/controller_manager"],
    )

    # 그리퍼가 있다면 추가 (이름은 ros2_controllers.yaml에 정의된 이름과 일치해야 함)
    tool_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["tool_controller", "--controller-manager", "/controller_manager"],
    )

    head_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
    )

    # 6. Robot State Publisher & RViz
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(moveit_pkg_path, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 최종 실행 리스트
    return LaunchDescription([
        ros2_control_node,
        move_group_node,
        rsp_node,
        rviz_node,
        jsb_spawner,
        arm_spawner,
        lift_spawner,
        tool_spawner,
        head_spawner,
    ])