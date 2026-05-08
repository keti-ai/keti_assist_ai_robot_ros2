"""
robot_control.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
2-Controller-Manager 구조:

  ┌─────────────────────────────────────────────────────────────────────┐
  │  robot_state_publisher                                              │
  │  └─ robot.urdf.xacro  mode:=<mode>  include_ros2_control:=false   │
  │     (전체 링크/조인트 TF 전용, ros2_control 블록은 RSP 무시)       │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /arm/controller_manager   (mode: robot | arm)                      │
  │  └─ arm_hw.urdf.xacro  → arm <ros2_control> 전용                  │
  │     spawner: joint_state_broadcaster, xarm7_traj_controller         │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /body/controller_manager  (mode: robot | body | lift | head | tool)│
  │  └─ body_hw.urdf.xacro → body <ros2_control> 전용                 │
  │     spawner: joint_state_broadcaster, lift/head/tool_controller     │
  ├─────────────────────────────────────────────────────────────────────┤
  │  joint_state_publisher (merger)                                     │
  │  └─ source_list: /arm/joint_states + /body/joint_states            │
  │     → /joint_states → robot_state_publisher                        │
  └─────────────────────────────────────────────────────────────────────┘

Launch 인자
  mode               robot | arm | body | lift | head | tool  (default: robot)
  use_fake_hardware  true | false                              (default: false)
  use_gui            true | false                              (default: true)
  spec               kaair_specs_*.yaml                       (default: kaair_specs_01.yaml)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


# ── 모드 → 실행 대상 매핑 ───────────────────────────────────────────────────
def _resolve_flags(mode: str) -> dict:
    """mode 문자열로부터 각 CM / spawner 실행 여부를 반환."""
    arm_modes  = {'robot', 'arm'}
    body_modes = {'robot', 'body', 'lift', 'head', 'tool'}
    return {
        'arm_cm':  mode in arm_modes,
        'body_cm': mode in body_modes,
        'lift':    mode in {'robot', 'body', 'lift'},
        'head':    mode in {'robot', 'body', 'head'},
        'tool':    mode in {'robot', 'body', 'tool'},
    }


def launch_setup(context, *args, **kwargs):
    # ── 런타임 인자 resolve ────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake     = use_fake_str.lower() in ('true', '1', 'yes')
    mode_str     = LaunchConfiguration('mode').perform(context)
    spec_str     = LaunchConfiguration('spec').perform(context)
    use_gui      = LaunchConfiguration('use_gui')

    flags = _resolve_flags(mode_str)

    # ── 패키지 경로 ────────────────────────────────────────────────────────
    ctrl_pkg    = get_package_share_directory('kaair_controller')
    desc_pkg    = get_package_share_directory('kaair_description')
    moveit_pkg  = get_package_share_directory('kaair_moveit_config')
    bringup_pkg = get_package_share_directory('kaair_bringup')

    hw_spec_file = os.path.join(bringup_pkg, 'config', 'robots', spec_str)

    # ── Controller YAML ────────────────────────────────────────────────────
    # fake/real 공통으로 동일한 yaml 사용. HW 구분은 URDF xacro 플러그인으로만 처리.
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── URDF xacro 경로 ────────────────────────────────────────────────────
    full_xacro    = os.path.join(desc_pkg,   'urdf',   'robot.urdf.xacro')
    arm_hw_xacro  = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')

    # ── robot_description 생성 ─────────────────────────────────────────────

    # RSP: 전체 링크/조인트 URDF (TF 전용).
    # include_ros2_control:=false 로 body 계열 ros2_control 블록 제외.
    # arm 의 ros2_control 은 xacro:xarm_device 매크로에서 자동 생성되지만 RSP 에 무해.
    rsp_description = {
        'robot_description': Command([
            'xacro ', full_xacro,
            ' mode:=',                 mode_str,
            ' hw_spec_file:=',         hw_spec_file,
            ' use_fake_hardware:=',    use_fake_str,
            ' include_ros2_control:=', 'false',
        ])
    }

    # arm CM: arm <ros2_control> 전용 URDF (arm_hw.urdf.xacro)
    arm_description = {
        'robot_description': Command([
            'xacro ', arm_hw_xacro,
            ' hw_spec_file:=',      hw_spec_file,
            ' use_fake_hardware:=', use_fake_str,
        ])
    }

    # body CM: body <ros2_control> 전용 URDF (body_hw.urdf.xacro)
    body_description = {
        'robot_description': Command([
            'xacro ', body_hw_xacro,
            ' hw_spec_file:=',      hw_spec_file,
            ' use_fake_hardware:=', use_fake_str,
        ])
    }

    # ════════════════════════════════════════════════════════════════════════
    # 노드 정의
    # ════════════════════════════════════════════════════════════════════════

    # [A] Robot State Publisher ─ 하나만 실행, 전체 URDF 사용
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[rsp_description],
    )

    # [B] RViz2
    rviz_cfg = os.path.join(desc_pkg, 'rviz', f'{mode_str}_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_gui),
    )

    # [C] joint_state_publisher (merger)
    # 두 CM 이 각자 발행하는 /arm/joint_states, /body/joint_states 를 합쳐서
    # /joint_states 로 재발행 → RSP 가 TF 를 갱신한다.
    # RSP 가 /robot_description 을 발행한 뒤 시작해야 "Waiting..." 루프를 피할 수 있다.
    source_list = []
    if flags['arm_cm']:
        source_list.append('/arm/joint_states')
    if flags['body_cm']:
        source_list.append('/body/joint_states')

    merger_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{'source_list': source_list, 'rate': 50}],
        remappings=[('robot_description', '/robot_description')],
    )

    # ════════════════════════════════════════════════════════════════════════
    # arm Controller Manager (namespace: /arm)
    # ════════════════════════════════════════════════════════════════════════

    # RSP 시작 직후 merger 와 RViz 를 실행 (robot_description 발행 보장)
    launch_items = [
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, rviz_node],
        )),
    ]

    if flags['arm_cm']:
        arm_cm_params = [arm_description, arm_ctrl_yaml]

        arm_cm_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm',
            parameters=arm_cm_params,
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

        arm_ctrl_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'xarm7_traj_controller',
                '--controller-manager', '/arm/controller_manager',
            ],
        )

        launch_items += [
            arm_cm_node,
            # arm CM 시작 → joint_state_broadcaster 등록 → arm controller 등록
            RegisterEventHandler(OnProcessStart(
                target_action=arm_cm_node,
                on_start=[arm_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=arm_jsb_spawner,
                on_start=[arm_ctrl_spawner],
            )),
        ]

    # ════════════════════════════════════════════════════════════════════════
    # body Controller Manager (namespace: /body)
    # ════════════════════════════════════════════════════════════════════════
    if flags['body_cm']:
        body_cm_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='body',
            parameters=[body_description, body_ctrl_yaml],
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

        body_ctrl_spawners = []
        if flags['lift']:
            body_ctrl_spawners.append(Node(
                package='controller_manager',
                executable='spawner',
                arguments=['lift_controller',
                           '--controller-manager', '/body/controller_manager'],
            ))
        if flags['head']:
            body_ctrl_spawners.append(Node(
                package='controller_manager',
                executable='spawner',
                arguments=['head_controller',
                           '--controller-manager', '/body/controller_manager'],
            ))
        if flags['tool']:
            body_ctrl_spawners.append(Node(
                package='controller_manager',
                executable='spawner',
                arguments=['tool_controller',
                           '--controller-manager', '/body/controller_manager'],
            ))

        launch_items += [
            body_cm_node,
            # body CM 시작 → joint_state_broadcaster 등록 → 각 컨트롤러 등록
            RegisterEventHandler(OnProcessStart(
                target_action=body_cm_node,
                on_start=[body_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=body_jsb_spawner,
                on_start=body_ctrl_spawners,
            )),
        ]

    return launch_items


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='robot',
            description=(
                '실행 모드: '
                'robot(전체) | arm(xArm7 단독) | body(lift+head+tool) | '
                'lift | head | tool'
            ),
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='true: mock_components FakeSystem / false: 실제 HW 드라이버',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='RViz2 실행 여부',
        ),
        DeclareLaunchArgument(
            'spec',
            default_value='kaair_specs_01.yaml',
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
