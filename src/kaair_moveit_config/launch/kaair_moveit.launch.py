import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_robot_api_params


def launch_setup(context, *args, **kwargs):
    # ── Runtime 값 resolve ────────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake     = use_fake_str.lower() in ('true', '1', 'yes')
    use_gui      = LaunchConfiguration('use_gui')
    spec         = LaunchConfiguration('spec')

    # ── 경로 ─────────────────────────────────────────────────────────────────
    moveit_pkg_name = 'kaair_moveit_config'
    moveit_pkg_path = get_package_share_directory(moveit_pkg_name)

    hw_spec_file = PathJoinSubstitution([
        FindPackageShare('kaair_bringup'), 'config', 'robots', spec
    ])
    initial_positions_file = os.path.join(
        moveit_pkg_path, 'config', 'initial_positions.yaml'
    )

    # ── 모드별 yaml 선택 ──────────────────────────────────────────────────────
    # controller_manager 에 전달할 컨트롤러 정의
    #   fake : kaair_moveit_config/config/ros2_controllers.yaml  (arm_controller)
    #   real : kaair_controller/config/kaair_controllers.yaml    (xarm7_traj_controller)
    if use_fake:
        ros2_controllers_yaml    = os.path.join(moveit_pkg_path, 'config', 'ros2_controllers.yaml')
        moveit_controllers_yaml  = 'config/moveit_controllers.yaml'
        arm_controller_name      = 'arm_controller'
    else:
        ros2_controllers_yaml    = os.path.join(
            get_package_share_directory('kaair_controller'), 'config', 'kaair_controllers.yaml'
        )
        moveit_controllers_yaml  = 'config/real_moveit_controllers.yaml'
        arm_controller_name      = 'xarm7_traj_controller'

    # ── MoveIt 설정 빌드 ──────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder('kaair', package_name=moveit_pkg_name)
        .robot_description(
            file_path=os.path.join(moveit_pkg_path, 'config', 'kaair.urdf.xacro'),
            mappings={
                'use_fake_hardware':      use_fake_str,
                'mode':                   'robot',
                'hw_spec_file':           hw_spec_file,
                'initial_positions_file': initial_positions_file,
            }
        )
        .robot_description_semantic(file_path='config/kaair.srdf')
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # ── xArm API 파라미터 (실제 드라이버 연결에 필요; fake 모드에서는 무해) ─────
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # ── 노드 정의 ─────────────────────────────────────────────────────────────

    # [A] MoveGroup
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # [B] ROS 2 Control Node (Controller Manager)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_yaml,
            robot_params,
        ],
        output='both',
    )

    # [C] Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    # [D] Static TF (map → base_footprint)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_map_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'slamware_map', 'base_footprint'],
    )

    # [E] RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(moveit_pkg_path, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui),
    )

    # ── Spawner 노드 ──────────────────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    # fake: arm_controller / real: xarm7_traj_controller
    arm_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[arm_controller_name],
    )
    lift_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['lift_controller'],
    )
    head_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['head_controller'],
    )
    tool_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['tool_controller'],
    )

    # ── 이벤트 체인 ───────────────────────────────────────────────────────────
    # 1. ros2_control_node 시작 → joint_state_broadcaster 스폰
    event_manager_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=[jsb_spawner])
    )
    # 2. jsb 시작 → 모든 컨트롤러 동시 스폰
    event_jsb_to_spawners = RegisterEventHandler(
        OnProcessStart(
            target_action=jsb_spawner,
            on_start=[arm_spawner, lift_spawner, head_spawner, tool_spawner],
        )
    )
    # 3. arm 컨트롤러 시작 → MoveGroup 실행 (플래닝 준비 완료)
    event_arm_to_movegroup = RegisterEventHandler(
        OnProcessStart(target_action=arm_spawner, on_start=[move_group_node])
    )

    return [
        robot_state_pub_node,
        static_tf_node,
        ros2_control_node,
        rviz_node,
        event_manager_to_jsb,
        event_jsb_to_spawners,
        event_arm_to_movegroup,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='true',
            description='true: mock_components FakeSystem / false: real HW drivers'
        ),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='RViz2 실행 여부'
        ),
        DeclareLaunchArgument(
            'spec', default_value='kaair_specs_01.yaml',
            description='로봇 하드웨어 스펙 파일 (.yaml)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
