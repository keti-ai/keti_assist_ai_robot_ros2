"""
kaair_moveit.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2-Controller-Manager + MoveIt 구조:

  ┌─────────────────────────────────────────────────────────────────────┐
  │  robot_state_publisher                                              │
  │  └─ kaair.urdf.xacro  (include_ros2_control=false)                 │
  │     → robot.urdf.xacro mode=robot: 전체 kinematics, ros2_control X │
  ├─────────────────────────────────────────────────────────────────────┤
  │  move_group                                                         │
  │  └─ 동일 URDF + kaair.srdf + moveit_controllers.yaml               │
  │     xarm7_traj_controller / lift / head / tool (fake/real 공통)    │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /arm/controller_manager                                            │
  │  └─ arm_hw.urdf.xacro                                              │
  │       fake: arm.ros2_control.xacro → mock_components/GenericSystem  │
  │       real: xacro:xarm_device      → UFRobotSystemHardware          │
  │     kaair_controller/config/arm_controllers.yaml                   │
  │     spawner: joint_state_broadcaster                                 │
  │             xarm7_traj_controller [ACTIVE]                          │
  │             xarm7_forward_controller [INACTIVE]                     │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /body/controller_manager                                           │
  │  └─ body_hw.urdf.xacro                                             │
  │       fake: kaair.ros2_control.xacro → mock_components/GenericSystem│
  │       real: lift/head/tool HW interfaces                            │
  │     kaair_controller/config/body_controllers.yaml                  │
  │     spawner: joint_state_broadcaster                                │
  │             lift/head/tool_controller       [ACTIVE]               │
  │             lift/head/tool_forward_controller [INACTIVE]           │
  ├─────────────────────────────────────────────────────────────────────┤
  │  controller_mode_switcher                                           │
  │  └─ ~/switch_mode (SetBool)                                         │
  │       true  → FORWARD 모드 (ForwardCommandController, 토픽 제어)    │
  │       false → NORMAL  모드 (JTC/Action, MoveIt 제어) ← 기본값       │
  ├─────────────────────────────────────────────────────────────────────┤
  │  joint_state_publisher (merger)                                     │
  │  └─ /arm/joint_states + /body/joint_states → /joint_states         │
  └─────────────────────────────────────────────────────────────────────┘

xacro 파일 역할 정리 (kaair_moveit_config/config/)
  kaair.urdf.xacro        RSP + move_group 전용. kinematics only (include_ros2_control=false)
  arm_hw.urdf.xacro       arm CM 전용. arm ros2_control 블록만 포함
  body_hw.urdf.xacro      body CM 전용. body ros2_control 블록만 포함
  arm.ros2_control.xacro  arm fake GenericSystem 매크로 (arm_hw.urdf.xacro 에서 사용)
  kaair.ros2_control.xacro body fake GenericSystem 매크로 (body_hw.urdf.xacro 에서 사용)

Launch 인자
  use_fake_hardware  true | false  (default: false)
  use_gui            true | false  (default: true)
  spec               kaair_specs_*.yaml  (default: kaair_specs_02.yaml)
    → config/robots/<spec> 의 mobile_bridge.type 으로 URDF/SRDF 선택
      clobot/clober → kaair_clober.urdf.xacro + kaair_clober.srdf
      slamtec         → kaair.urdf.xacro + kaair.srdf
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

_bringup_launch_dir = os.path.join(
    get_package_share_directory('kaair_bringup'), 'launch')
if _bringup_launch_dir not in sys.path:
    sys.path.insert(0, _bringup_launch_dir)
from robot_spec_utils import (  # noqa: E402
    get_mobile_bridge_type,
    load_robot_spec,
    resolve_moveit_urdf_paths,
)


def launch_setup(context, *args, **kwargs):
    # ── 런타임 인자 resolve ────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake     = use_fake_str.lower() in ('true', '1', 'yes')
    spec_str     = LaunchConfiguration('spec').perform(context)
    use_gui      = LaunchConfiguration('use_gui')

    # ── 패키지 경로 ────────────────────────────────────────────────────────
    moveit_pkg  = get_package_share_directory('kaair_moveit_config')
    ctrl_pkg    = get_package_share_directory('kaair_controller')
    bringup_pkg = get_package_share_directory('kaair_bringup')

    hw_spec_file = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    spec_data, spec_path = load_robot_spec(spec_str)
    kaair_xacro, srdf_file = resolve_moveit_urdf_paths(moveit_pkg, spec_data)
    print(
        f'[kaair_moveit] mobile_bridge.type={get_mobile_bridge_type(spec_data)!r} '
        f'→ {os.path.basename(kaair_xacro)} ← {spec_path}'
    )

    # ── Controller YAML ────────────────────────────────────────────────────
    # fake/real 공통. HW 구분은 URDF xacro 플러그인으로만 처리.
    # 컨트롤러 이름(xarm7_traj_controller)은 fake/real 동일하다.
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── xacro 경로 ─────────────────────────────────────────────────────────
    arm_hw_xacro  = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')

    # ── robot_description 생성 ─────────────────────────────────────────────

    def make_description(xacro_path, extra=''):
        cmd = (
            f'xacro {xacro_path}'
            f' use_fake_hardware:={use_fake_str}'
            f' hw_spec_file:={hw_spec_file}'
        )
        if extra:
            cmd += ' ' + extra
        return {'robot_description': ParameterValue(Command(cmd), value_type=str)}

    # arm CM: arm <ros2_control> 전용 (initial_positions 로 초기 자세 설정)
    arm_description  = make_description(
        arm_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )
    # body CM: body <ros2_control> 전용
    body_description = make_description(
        body_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )

    # ── MoveIt 설정 빌드 ───────────────────────────────────────────────────
    # kaair.urdf.xacro 는 include_ros2_control=false 가 기본값이므로
    # kinematics 전용 URDF 로 사용된다. RSP + move_group 공용.
    moveit_config = (
        MoveItConfigsBuilder('kaair', package_name='kaair_moveit_config')
        .robot_description(
            file_path=kaair_xacro,
            mappings={
                'use_fake_hardware':      use_fake_str,
                'mode':                   'robot',
                'hw_spec_file':           hw_spec_file,
                'initial_positions_file': initial_positions_file,
                # kaair.urdf.xacro 의 기본값이 false 이므로 명시 생략 가능하지만
                # 혹시 외부에서 override 되는 상황을 방지하기 위해 명시한다.
                'include_ros2_control':   'false',
            }
        )
        .robot_description_semantic(file_path=srdf_file)
        # fake/real 모두 동일한 moveit_controllers.yaml 사용
        # (xarm7_traj_controller 이름으로 통일)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['pilz_industrial_motion_planner'],
            default_planning_pipeline='pilz_industrial_motion_planner',
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # ── ros2_control/MoveIt >= Jazzy 파라미터 스키마 보정 ─────────────────────
    # MoveIt 2.12(Jazzy 계열)의 PlanningPipeline 리팩터링(moveit/moveit2#2429)
    # 으로 파이프라인 설정 스키마가 바뀌었다:
    #   planning_plugin(문자열) → planning_plugins(리스트)
    #   request_adapters: 공백구분 문자열 → 리스트, 플러그인 네임스페이스도
    #     default_planner_request_adapters/* → default_planning_request_adapters/*
    # 로 변경되었다. config/pilz_industrial_motion_planner_planning.yaml 은
    # Humble(moveit_ros_planning 2.5.x)이 요구하는 구(舊) 스키마를 그대로
    # 유지하고, Jazzy 에서만 이 자리에서 새 스키마로 패치한다. 구 스키마를
    # 그대로 Jazzy 에 주면 'expected [string_array] got [string]' 예외로
    # move_group 이 죽고, 반대로 새 스키마를 Humble 에 주면 동일하게 타입
    # 예외로 죽으므로, 설정 파일 자체를 Humble 스키마로 고정해두고 Jazzy
    # 실행 시점에만 여기서 변환하는 방식으로 두 배포판에서 launch 파일과
    # yaml 원본을 그대로 공유한다.
    if os.environ.get('ROS_DISTRO') == 'jazzy':
        _pilz_cfg = moveit_config.planning_pipelines.get('pilz_industrial_motion_planner')
        if _pilz_cfg and 'planning_plugin' in _pilz_cfg:
            _pilz_cfg['planning_plugins'] = [_pilz_cfg.pop('planning_plugin')]
            _pilz_cfg['request_adapters'] = [
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ]

    # ════════════════════════════════════════════════════════════════════════
    # 노드 정의
    # ════════════════════════════════════════════════════════════════════════

    # [A] move_group ─ MoveIt 플래너. 모든 HW 활성화 후 기동 (이벤트 체인 끝)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )


    # [B] Robot State Publisher ─ kaair.urdf.xacro (kinematics only)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    # [B'] arm/body 전용 Robot State Publisher
    #   ros2_control 4.x(Jazzy~) 부터 controller_manager 에 robot_description 을
    #   파라미터로 직접 넘기는 방식이 완전히 제거되어, robot_description
    #   토픽(robot_state_publisher 가 transient_local 로 발행)을 통해서만
    #   초기화된다. Humble(ros2_control 2.x)에서는 파라미터 직접 전달이
    #   deprecated 경고와 함께 동작했지만, 토픽 방식은 두 배포판 모두에서
    #   정상 동작하는 공통 경로이므로 이 방식으로 통일한다.
    #
    #   단, controller_manager 가 구독하는 토픽 경로가 배포판마다 다르다
    #   (실제 로그로 확인됨):
    #     Humble (ros2_control 2.x) : '~/robot_description'
    #       → private 네임스페이스, 즉 '/arm/controller_manager/robot_description'
    #     Jazzy  (ros2_control 4.x) : 'robot_description' (네임스페이스 상대)
    #       → '/arm/robot_description'
    #   robot_state_publisher 의 기본 발행 토픽('<ns>/robot_description')은
    #   Jazzy 쪽 규칙과만 일치하므로, Humble 에서는 명시적으로 remap 해
    #   controller_manager 의 실제 구독 경로에 맞춘다.
    if os.environ.get('ROS_DISTRO') == 'humble':
        _arm_rd_topic = '/arm/controller_manager/robot_description'
        _body_rd_topic = '/body/controller_manager/robot_description'
    else:
        _arm_rd_topic = '/arm/robot_description'
        _body_rd_topic = '/body/robot_description'

    arm_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='arm',
        output='both',
        parameters=[arm_description],
        remappings=[('robot_description', _arm_rd_topic)],
    )
    body_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='body',
        output='both',
        parameters=[body_description],
        remappings=[('robot_description', _body_rd_topic)],
    )

    # [C] joint_state_publisher (merger)
    # /arm/joint_states + /body/joint_states → /joint_states
    with open(initial_positions_file, 'r') as _f:
        _initial_pos = yaml.safe_load(_f).get('initial_positions', {})

    merger_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': ['/arm/joint_states', '/body/joint_states'],
            'rate': 50,
            'initial_positions': _initial_pos,
        }],
        remappings=[('robot_description', '/robot_description')],
    )

    # [D] Static TF: slamware_map → base_footprint
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
        output='log',
        arguments=['-d', os.path.join(moveit_pkg, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui),
    )

    # ════════════════════════════════════════════════════════════════════════
    # arm Controller Manager (namespace: /arm)
    # ════════════════════════════════════════════════════════════════════════
    # robot_description 은 arm_rsp_node 가 '/arm/robot_description' 토픽으로
    # 발행하므로 controller_manager 에는 컨트롤러 yaml 만 전달한다.
    arm_cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=[arm_ctrl_yaml],
        # 실제 HW 에서 UFRobotSystemHardware 는 실시간성이 필요하므로 nice 레벨 높임
        output='both',
    )

    arm_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/arm/controller_manager',
        ],
    )

    # fake/real 모두 xarm7_traj_controller 사용
    arm_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm7_traj_controller',
            '--controller-manager', '/arm/controller_manager',
        ],
    )
    xarm7_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm7_forward_controller',
            '--controller-manager', '/arm/controller_manager',
            '--inactive',
        ],
    )

    # ════════════════════════════════════════════════════════════════════════
    # body Controller Manager (namespace: /body)
    # ════════════════════════════════════════════════════════════════════════
    # robot_description 은 body_rsp_node 가 '/body/robot_description' 토픽으로
    # 발행하므로 controller_manager 에는 컨트롤러 yaml 만 전달한다.
    body_cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='body',
        parameters=[body_ctrl_yaml],
        output='both',
    )

    body_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/body/controller_manager',
        ],
    )
    # ── body 정규 컨트롤러 (ACTIVE) ─────────────────────────────────────
    lift_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_controller',
                   '--controller-manager', '/body/controller_manager'],
    )
    head_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller',
                   '--controller-manager', '/body/controller_manager'],
    )
    tool_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tool_controller',
                   '--controller-manager', '/body/controller_manager'],
    )

    # ── body Forward 컨트롤러 (INACTIVE) ─────────────────────────────────
    # 기동 시 configured 상태로만 올라옴. 전환은 controller_mode_switcher 서비스로.
    lift_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_forward_controller',
                   '--controller-manager', '/body/controller_manager',
                   '--inactive'],
    )
    head_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_forward_controller',
                   '--controller-manager', '/body/controller_manager',
                   '--inactive'],
    )
    tool_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tool_forward_controller',
                   '--controller-manager', '/body/controller_manager',
                   '--inactive'],
    )

    # ── 컨트롤러 모드 전환 노드 ───────────────────────────────────────────
    # tool_spawner 종료(모든 정규 컨트롤러 활성화 완료) 후 기동
    ctrl_mode_switcher_node = Node(
        package='kaair_bringup',
        executable='controller_mode_switcher',
        name='controller_mode_switcher',
        output='screen',
    )

    # ════════════════════════════════════════════════════════════════════════
    # 이벤트 체인
    # ════════════════════════════════════════════════════════════════════════
    #
    # RSP 기동 → merger + RViz (robot_description 발행 보장)
    #
    # arm 경로:
    #   arm_cm_node  시작 → arm_jsb_spawner
    #   arm_jsb_spawner 시작 → arm_ctrl_spawner
    #
    # body 경로:
    #   body_cm_node 시작 → body_jsb_spawner
    #   body_jsb_spawner 시작 → lift / head / tool spawner (병렬)
    #
    # move_group:
    #   tool_spawner 종료(= body HW 전체 활성화 완료) → move_group 기동
    #   arm 쪽(네트워크 연결)은 body(USB) 보다 빠르게 완료되므로
    #   tool_spawner exit 을 전체 HW 준비 완료의 트리거로 사용한다.

    return [
        # TF / 상태 발행 인프라
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, static_tf_node, rviz_node],
        )),

        # arm/body robot_description 발행 (controller_manager 가 구독)
        arm_rsp_node,
        body_rsp_node,

        # arm Controller Manager
        arm_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=arm_cm_node,
            on_start=[arm_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=arm_jsb_spawner,
            # xarm7_traj_controller(ACTIVE) + xarm7_forward_controller(INACTIVE) 동시 스폰
            on_start=[arm_ctrl_spawner, xarm7_fwd_spawner],
        )),

        # body Controller Manager
        body_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=body_cm_node,
            on_start=[body_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=body_jsb_spawner,
            # 정규(ACTIVE) + Forward(INACTIVE) 동시 스폰
            on_start=[
                lift_spawner, lift_fwd_spawner,
                head_spawner, head_fwd_spawner,
                tool_spawner, tool_fwd_spawner,
            ],
        )),

        # tool_spawner 종료 = body HW 전체 준비 완료
        #   → move_group + controller_mode_switcher 동시 기동
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node, ctrl_mode_switcher_node],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='false: 실제 HW 드라이버 / true: mock_components FakeSystem',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='RViz2 실행 여부',
        ),
        DeclareLaunchArgument(
            'spec',
            default_value='kaair_specs_02.yaml',
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
