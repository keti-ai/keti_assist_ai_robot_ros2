"""
kaair_moveit.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2-Controller-Manager + MoveIt 구조:

  ┌─────────────────────────────────────────────────────────────────────┐
  │  robot_state_publisher + joint_state_publisher(merger) + RViz2      │  ← controller 와 무관하게 즉시 기동
  │  └─ kaair.urdf.xacro  (include_ros2_control=false)                 │
  │     → robot.urdf.xacro mode=robot: 전체 kinematics, ros2_control X │
  ├─────────────────────────────────────────────────────────────────────┤
  │  (실기체) lift_initializer 가 가장 먼저 단독 실행                   │  ← 이 단계만 직렬
  │  └─ 영점 미확인 시에만 호밍(시간 소요), 확인되면 즉시 스킵          │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /arm/controller_manager                            ─┐              │
  │  └─ arm_hw.urdf.xacro                                │  병렬 기동   │
  │       fake: arm.ros2_control.xacro → mock/GenericSystem │           │
  │       real: xacro:xarm_device      → UFRobotSystemHardware │        │
  │     spawner: joint_state_broadcaster                  │             │
  │             xarm7_traj_controller [ACTIVE]            │             │
  │             xarm7_forward_controller [INACTIVE]       │             │
  ├──────────────────────────────────────────────────────┤             │
  │  /body/controller_manager                             ┘             │
  │  └─ body_hw.urdf.xacro                                             │
  │       fake: kaair.ros2_control.xacro → mock/GenericSystem          │
  │       real: lift/head/tool HW interfaces                            │
  │     spawner: joint_state_broadcaster                                │
  │             lift/head/tool_controller       [ACTIVE]               │
  │             lift/head/tool_forward_controller [INACTIVE]           │
  ├─────────────────────────────────────────────────────────────────────┤
  │  move_group + controller_mode_switcher                              │  ← tool_spawner 종료 후 기동
  │  └─ move_group: 동일 URDF + kaair.srdf + moveit_controllers.yaml   │
  │     controller_mode_switcher: ~/switch_mode (SetBool)               │
  │       true  → FORWARD 모드 (ForwardCommandController, 토픽 제어)    │
  │       false → NORMAL  모드 (JTC/Action, MoveIt 제어) ← 기본값       │
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

_ctrl_launch_dir = os.path.join(
    get_package_share_directory('kaair_controller'), 'launch')
if _ctrl_launch_dir not in sys.path:
    sys.path.insert(0, _ctrl_launch_dir)
from control_managers import build_control_managers  # noqa: E402

_this_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _this_launch_dir not in sys.path:
    sys.path.insert(0, _this_launch_dir)
from moveit_pipeline_compat import (  # noqa: E402
    fix_planning_pipelines_for_jazzy,
    is_humble,
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
    if not is_humble():
        # Jazzy 의 PlanningPipeline 은 planning_plugin(str)/request_adapters(str)
        # 대신 planning_plugins(list)/request_adapters(list) 를 요구한다.
        # (config/*_planning.yaml 자체는 Humble 포맷 그대로 유지)
        fix_planning_pipelines_for_jazzy(moveit_config)

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
    # arm/body Controller Manager (+ lift_initializer 게이트)
    # ════════════════════════════════════════════════════════════════════════
    # control_managers.build_control_managers() 가 arm/body CM, 그 스포너,
    # 그리고 (실기체 + lift 사용 시) lift_initializer → arm/body CM 체인까지
    # 전부 생성한다. kaair_controller/launch/robot_control.launch.py 와
    # 동일한 로직을 공유한다.
    cm_actions, cm_handles = build_control_managers(
        arm_description=arm_description,
        body_description=body_description,
        arm_ctrl_yaml=arm_ctrl_yaml,
        body_ctrl_yaml=body_ctrl_yaml,
        use_fake_hardware=use_fake,
        body_active_controllers=['lift_controller', 'head_controller', 'tool_controller'],
        body_forward_controllers=['lift_forward_controller', 'head_forward_controller', 'tool_forward_controller'],
        arm_forward_controller='xarm7_forward_controller',
    )
    tool_spawner = cm_handles['body_active_spawners']['tool_controller']

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
    # RSP 는 controller 와 무관하므로 즉시 기동 → merger + static_TF
    #
    # arm/body CM 및 (실기체일 때) lift_initializer → arm/body CM 게이팅은
    # build_control_managers() 내부에서 처리된다 (lift_initializer 만 직렬,
    # 이후 arm/body CM 은 병렬).
    #
    # move_group:
    #   tool_spawner 종료(= body HW 전체 활성화 완료) → move_group 기동
    #   arm 쪽(네트워크 연결)은 body(USB) 보다 빠르게 완료되므로
    #   tool_spawner exit 을 전체 HW 준비 완료의 트리거로 사용한다.
    #
    # RViz 는 move_group 기동 후에 띄운다: MotionPlanning 디스플레이가
    # move_group 서비스/액션에 곧바로 연결되지 않으면 처음에 경고가 뜨고
    # 체크박스를 껐다 켜야 정상화되는 문제가 있어, move_group 이 이미 뜬
    # 뒤에 RViz 를 시작해 그 문제를 피한다.

    return [
        # TF / 상태 발행 인프라
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, static_tf_node],
        )),

        # arm/body Controller Manager (+ lift_initializer 게이트)
        *cm_actions,

        # tool_spawner 종료 = body HW 전체 준비 완료
        #   → move_group + controller_mode_switcher 동시 기동
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node, ctrl_mode_switcher_node],
        )),

        # move_group 기동 후 RViz 시작 (MotionPlanning 연결 경고 방지)
        RegisterEventHandler(OnProcessStart(
            target_action=move_group_node,
            on_start=[rviz_node],
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
