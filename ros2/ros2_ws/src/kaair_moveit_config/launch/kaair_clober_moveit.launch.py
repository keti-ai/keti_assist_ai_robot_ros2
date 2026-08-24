"""
kaair_clober_moveit.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
kaair_moveit.launch.py 와 동일한 2-Controller-Manager + MoveIt 구조.
모바일 베이스만 Clober (former_description/clober_500) 로 교체한 변형.

  ┌─────────────────────────────────────────────────────────────────────┐
  │  robot_state_publisher                                              │
  │  └─ kaair_clober.urdf.xacro  (include_ros2_control=false)          │
  │     → clober_robot.urdf.xacro mode=robot: 전체 kinematics          │
  ├─────────────────────────────────────────────────────────────────────┤
  │  move_group                                                         │
  │  └─ 동일 URDF + kaair_clober.srdf + moveit_controllers.yaml        │
  │     xarm7_traj_controller / lift / head / tool (fake/real 공통)    │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /arm/controller_manager                                            │
  │  └─ arm_hw.urdf.xacro                                              │
  │       fake: arm.ros2_control.xacro → mock_components/GenericSystem  │
  │       real: xacro:xarm_device      → UFRobotSystemHardware          │
  │     kaair_controller/config/arm_controllers.yaml                   │
  │     spawner: joint_state_broadcaster                                │
  │             xarm7_traj_controller [ACTIVE]                         │
  │             xarm7_forward_controller [INACTIVE]                    │
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
  │       true  → FORWARD 모드 (ForwardCommandController, 토픽 제어)   │
  │       false → NORMAL  모드 (JTC/Action, MoveIt 제어) ← 기본값      │
  ├─────────────────────────────────────────────────────────────────────┤
  │  joint_state_publisher (merger)                                     │
  │  └─ /arm/joint_states + /body/joint_states → /joint_states         │
  └─────────────────────────────────────────────────────────────────────┘

xacro 파일 역할 정리 (kaair_moveit_config/config/)
  kaair_clober.urdf.xacro  RSP + move_group 전용. kinematics only (clober base)
  arm_hw.urdf.xacro        arm CM 전용. arm ros2_control 블록만 포함
  body_hw.urdf.xacro       body CM 전용. body ros2_control 블록만 포함

Launch 인자
  use_fake_hardware  true | false  (default: false)
  use_gui            true | false  (default: true)
  spec               kaair_specs_*.yaml  (default: kaair_specs_02.yaml)
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

_ctrl_launch_dir = os.path.join(
    get_package_share_directory('kaair_controller'), 'launch')
if _ctrl_launch_dir not in sys.path:
    sys.path.insert(0, _ctrl_launch_dir)
from control_managers import build_control_managers  # noqa: E402


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

    hw_spec_file           = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    # ── Controller YAML ────────────────────────────────────────────────────
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── xacro 경로 ─────────────────────────────────────────────────────────
    arm_hw_xacro    = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro   = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')
    # Clober 전용 RSP/move_group URDF
    kaair_xacro     = os.path.join(moveit_pkg, 'config', 'kaair_clober.urdf.xacro')

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

    arm_description  = make_description(
        arm_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )
    body_description = make_description(
        body_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )

    # ── MoveIt 설정 빌드 ───────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder('kaair', package_name='kaair_moveit_config')
        .robot_description(
            file_path=kaair_xacro,
            mappings={
                'use_fake_hardware':      use_fake_str,
                'mode':                   'robot',
                'hw_spec_file':           hw_spec_file,
                'initial_positions_file': initial_positions_file,
                'include_ros2_control':   'false',
            }
        )
        .robot_description_semantic(file_path='config/kaair_clober.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(
            pipelines=['pilz_industrial_motion_planner'],
            default_planning_pipeline='pilz_industrial_motion_planner',
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # ════════════════════════════════════════════════════════════════════════
    # 노드 정의
    # ════════════════════════════════════════════════════════════════════════

    # [A] move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # [B] Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    # [C] joint_state_publisher (merger)
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

    ctrl_mode_switcher_node = Node(
        package='kaair_bringup',
        executable='controller_mode_switcher',
        name='controller_mode_switcher',
        output='screen',
    )

    # ════════════════════════════════════════════════════════════════════════
    # 이벤트 체인
    # ════════════════════════════════════════════════════════════════════════
    return [
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, static_tf_node],
        )),

        # arm/body Controller Manager (+ lift_initializer 게이트)
        *cm_actions,

        # tool_spawner 종료 = body HW 전체 준비 완료 → move_group + switcher 기동
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node, ctrl_mode_switcher_node],
        )),

        # RViz 는 move_group 기동 후에 띄운다: MotionPlanning 디스플레이가
        # move_group 서비스/액션에 곧바로 연결되지 않으면 처음에 경고가 뜨고
        # 체크박스를 껐다 켜야 정상화되는 문제가 있어, move_group 이 이미 뜬
        # 뒤에 RViz 를 시작해 그 문제를 피한다.
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
            default_value='kaair_specs_01.yaml',
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
