import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
    hw_spec_file_str = hw_spec_file.perform(context)

    initial_positions_file = os.path.join(
        moveit_pkg_path, 'config', 'initial_positions.yaml'
    )

    # ── 모드별 yaml 선택 ──────────────────────────────────────────────────────
    #
    # controller_manager 에 전달할 컨트롤러 정의 (arm / body 각각)
    #   fake : kaair_moveit_config/config/fake_{arm,body}_controllers.yaml
    #   real : kaair_controller/config/{arm,body}_controllers.yaml
    #
    # MoveIt trajectory execution 설정
    #   fake : config/moveit_controllers.yaml     (/arm, /body prefix)
    #   real : config/real_moveit_controllers.yaml (/arm, /body prefix)
    #
    # arm controller 이름 (MoveIt planGroup 표시용)
    #   fake : arm_controller  / real : xarm7_traj_controller
    if use_fake:
        arm_controllers_yaml   = os.path.join(moveit_pkg_path, 'config', 'fake_arm_controllers.yaml')
        body_controllers_yaml  = os.path.join(moveit_pkg_path, 'config', 'fake_body_controllers.yaml')
        moveit_controllers_yaml = 'config/moveit_controllers.yaml'
    else:
        ctrl_pkg_path          = get_package_share_directory('kaair_controller')
        arm_controllers_yaml   = os.path.join(ctrl_pkg_path, 'config', 'arm_controllers.yaml')
        body_controllers_yaml  = os.path.join(ctrl_pkg_path, 'config', 'body_controllers.yaml')
        moveit_controllers_yaml = 'config/real_moveit_controllers.yaml'

    # ── arm / body 전용 robot_description xacro 경로 ─────────────────────────
    #
    # arm CM: arm_hw.urdf.xacro  (arm ros2_control 블록만 포함)
    # body CM: body_hw.urdf.xacro (lift/head/tool ros2_control 블록만 포함)
    # RSP   : kaair.urdf.xacro  mode=robot (전체 URDF, 키네마틱+TF 용도)
    arm_hw_xacro  = os.path.join(moveit_pkg_path, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg_path, 'config', 'body_hw.urdf.xacro')

    def xacro_to_param(xacro_path, extra_args=''):
        cmd = f'xacro {xacro_path} use_fake_hardware:={use_fake_str} hw_spec_file:={hw_spec_file_str}'
        if extra_args:
            cmd += ' ' + extra_args
        return {'robot_description': ParameterValue(Command(cmd), value_type=str)}

    arm_robot_description  = xacro_to_param(
        arm_hw_xacro,
        f'initial_positions_file:={initial_positions_file}'
    )
    body_robot_description = xacro_to_param(
        body_hw_xacro,
        f'initial_positions_file:={initial_positions_file}'
    )

    # ── MoveIt 설정 빌드 (RSP + move_group 용) ────────────────────────────────
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

    # ── xArm API 파라미터 (실제 드라이버 연결 시 필요; arm CM 에만 전달) ──────
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # ── 노드 정의 ─────────────────────────────────────────────────────────────

    # [A] MoveGroup – 전체 URDF + 두 CM 의 컨트롤러를 모두 인식
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # ── [B] arm_controller_manager (namespace: /arm) ──────────────────────────
    #
    # · robot_description = arm_hw.urdf.xacro (arm ros2_control 블록만)
    # · 다른 HW 를 로드하지 않으므로 body HW 와 리소스 충돌 없음
    # · 토픽/서비스/액션 서버는 모두 /arm/ 접두사로 발행됨
    #   예) /arm/controller_manager
    #       /arm/joint_states          ← arm JSB 발행
    #       /arm/xarm7_traj_controller/follow_joint_trajectory
    arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=[
            arm_robot_description,
            arm_controllers_yaml,
            robot_params,
        ],
        prefix=['nice -n -20'],
        output='both',
    )

    # ── [C] body_controller_manager (namespace: /body) ────────────────────────
    #
    # · robot_description = body_hw.urdf.xacro (lift/head/tool ros2_control 블록만)
    # · 토픽/서비스/액션 서버는 모두 /body/ 접두사로 발행됨
    #   예) /body/controller_manager
    #       /body/joint_states         ← body JSB 발행
    #       /body/tool_controller/gripper_cmd
    body_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='body',
        parameters=[
            body_robot_description,
            body_controllers_yaml,
        ],
        output='both',
    )

    # ── [D] Robot State Publisher – 전체 URDF, 하나만 실행 ────────────────────
    #
    # · moveit_config.robot_description = kaair.urdf.xacro mode=robot (전체)
    # · /joint_states 토픽을 구독 → TF 발행
    # · /joint_states 는 아래 joint_state_merger 가 /arm/joint_states 와
    #   /body/joint_states 를 합쳐서 발행해 준다.
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
        # 기본 구독: /joint_states (merger 가 발행하는 토픽)
    )

    # ── [E] joint_state_merger ────────────────────────────────────────────────
    #
    # joint_state_publisher 패키지를 merger 로 활용한다.
    # source_list 에 두 CM 의 JSB 출력을 등록하면 합쳐서 /joint_states 로 발행.
    #
    # · /arm/joint_states  : joint1-7
    # · /body/joint_states : lift_joint, head_joint1/2, virtual_gripper_joint
    # → /joint_states      : 전체 (RSP 가 이 토픽을 구독)
    with open(initial_positions_file, 'r') as _f:
        _initial_pos = yaml.safe_load(_f).get('initial_positions', {})

    joint_state_merger = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': ['/arm/joint_states', '/body/joint_states'],
            'rate': 50,
            # HW 연결 전 공백 구간(fake: JSB 준비 전 / real: 드라이버 연결 전)에
            # initial_positions.yaml 값을 fallback 으로 사용한다.
            # source 가 해당 관절을 발행하기 시작하면 즉시 실제 값으로 전환된다.
            'initial_positions': _initial_pos,
        }],
        remappings=[
            ('robot_description', '/robot_description'),
        ],
    )

    # ── [F] Static TF (map → base_footprint) ─────────────────────────────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fake_map_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'slamware_map', 'base_footprint'],
    )

    # ── [G] RViz2 ─────────────────────────────────────────────────────────────
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
    #
    # spawner 의 --controller-manager 플래그로 각 CM 을 명시적으로 지정한다.
    # 지정하지 않으면 기본값 /controller_manager 를 찾아 실패한다.

    # /arm/controller_manager 에 속하는 컨트롤러
    arm_jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/arm/controller_manager'],
    )
    arm_ctrl_name = 'arm_controller' if use_fake else 'xarm7_traj_controller'
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[arm_ctrl_name,
                   '--controller-manager', '/arm/controller_manager'],
    )

    # /body/controller_manager 에 속하는 컨트롤러
    body_jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/body/controller_manager'],
    )
    lift_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['lift_controller',
                   '--controller-manager', '/body/controller_manager'],
    )
    head_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['head_controller',
                   '--controller-manager', '/body/controller_manager'],
    )
    tool_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['tool_controller',
                   '--controller-manager', '/body/controller_manager'],
    )

    # ── 이벤트 체인 ───────────────────────────────────────────────────────────
    #
    # arm  경로: arm_control_node  시작 → arm_jsb  → arm_ctrl
    # body 경로: body_control_node 시작 → body_jsb → lift/head/tool
    # move_group: tool_spawner 종료(= body HW 활성화 완료) 후 실행
    #   arm 쪽은 네트워크 연결이라 tool(USB) 보다 빠르게 끝나므로 이 트리거로 충분
    event_arm_cm_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=arm_control_node, on_start=[arm_jsb_spawner])
    )
    event_arm_jsb_to_ctrl = RegisterEventHandler(
        OnProcessStart(target_action=arm_jsb_spawner, on_start=[arm_ctrl_spawner])
    )

    event_body_cm_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=body_control_node, on_start=[body_jsb_spawner])
    )
    event_body_jsb_to_ctrls = RegisterEventHandler(
        OnProcessStart(
            target_action=body_jsb_spawner,
            on_start=[lift_spawner, head_spawner, tool_spawner],
        )
    )

    # tool_spawner 가 exit(= ToolHwInterface on_activate 완료) 되면 MoveGroup 실행
    event_tool_to_movegroup = RegisterEventHandler(
        OnProcessExit(target_action=tool_spawner, on_exit=[move_group_node])
    )

    return [
        # TF / 상태 발행 인프라
        robot_state_pub_node,
        joint_state_merger,
        static_tf_node,

        # 두 개의 controller_manager
        arm_control_node,
        body_control_node,

        # RViz
        rviz_node,

        # 이벤트 체인
        event_arm_cm_to_jsb,
        event_arm_jsb_to_ctrl,
        event_body_cm_to_jsb,
        event_body_jsb_to_ctrls,
        event_tool_to_movegroup,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='false: real HW drivers / true: mock_components FakeSystem'
        ),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='RViz2 실행 여부'
        ),
        DeclareLaunchArgument(
            'spec', default_value='kaair_specs_02.yaml',
            description='로봇 하드웨어 스펙 파일 (.yaml)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
