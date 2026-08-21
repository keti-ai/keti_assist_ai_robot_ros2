"""
robot_control.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
control_manager 모듈(control_manager.py)만으로 arm/body Controller Manager 를
독립 실행/테스트하기 위한 launch 파일. MoveIt(move_group)은 붙지 않는다.

  ┌─────────────────────────────────────────────────────────────────────┐
  │  robot_state_publisher                                              │
  │  └─ robot.urdf.xacro  mode:=<mode>  include_ros2_control:=false   │
  │     (전체 링크/조인트 TF 전용, ros2_control 블록은 RSP 무시)       │
  ├─────────────────────────────────────────────────────────────────────┤
  │  control_manager 모듈 (kaair_controller/launch/control_manager.py)  │
  │  └─ mode 에 따라 /arm, /body(lift/head/tool) controller_manager,   │
  │     각 spawner, controller_mode_switcher(arm+body 모두 있을 때만)  │
  │     를 구성한다.                                                    │
  ├─────────────────────────────────────────────────────────────────────┤
  │  joint_state_publisher (merger)                                     │
  │  └─ 모듈이 만들고, 이 파일이 RSP 시작에 체이닝한다.                │
  └─────────────────────────────────────────────────────────────────────┘

Launch 인자
  mode               robot | arm | body | lift | head | tool  (default: robot)
  use_fake_hardware  true | false                              (default: false)
  use_gui            true | false                              (default: true)
  spec               kaair_specs_*.yaml                       (default: kaair_specs_01.yaml)
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

_this_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _this_launch_dir not in sys.path:
    sys.path.insert(0, _this_launch_dir)
from control_manager import build_control_manager  # noqa: E402


# ── 모드 → 실행 대상 매핑 ───────────────────────────────────────────────────
def _resolve_flags(mode: str) -> dict:
    """mode 문자열로부터 각 CM / spawner 실행 여부를 반환."""
    arm_modes  = {'robot', 'arm'}
    body_modes = {'robot', 'body', 'lift', 'head', 'tool'}
    return {
        'arm':  mode in arm_modes,
        'body': mode in body_modes,
        'lift': mode in {'robot', 'body', 'lift'},
        'head': mode in {'robot', 'body', 'head'},
        'tool': mode in {'robot', 'body', 'tool'},
    }


def launch_setup(context, *args, **kwargs):
    # ── 런타임 인자 resolve ────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
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
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    # ── URDF xacro 경로 ────────────────────────────────────────────────────
    full_xacro = os.path.join(desc_pkg, 'urdf', 'robot.urdf.xacro')

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

    # [C] arm/body Controller Manager (모듈화)
    cm = build_control_manager(
        use_fake_str=use_fake_str,
        hw_spec_file=hw_spec_file,
        initial_positions_file=initial_positions_file,
        ctrl_pkg=ctrl_pkg,
        moveit_pkg=moveit_pkg,
        include_arm=flags['arm'],
        include_body=flags['body'],
        include_lift=flags['lift'],
        include_head=flags['head'],
        include_tool=flags['tool'],
    )

    # RSP 가 /robot_description 을 발행한 뒤 시작해야 "Waiting..." 루프를 피할 수 있다.
    return [
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[cm['merger_node'], rviz_node],
        )),

        *cm['always_on_actions'],
    ]


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
