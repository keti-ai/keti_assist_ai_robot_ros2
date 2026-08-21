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
  │  kaair_controller/launch/control_manager.py (모듈)                  │
  │  └─ /arm/controller_manager, /body/controller_manager,              │
  │     각 spawner, controller_mode_switcher 를 모두 구성한다.          │
  │     자세한 노드 구성은 그 파일의 docstring 참고.                    │
  ├─────────────────────────────────────────────────────────────────────┤
  │  joint_state_publisher (merger)                                     │
  │  └─ /arm/joint_states + /body/joint_states → /joint_states         │
  │     (control_manager 모듈이 만들고, 이 파일이 RSP 시작에 체이닝)   │
  └─────────────────────────────────────────────────────────────────────┘

xacro 파일 역할 정리 (kaair_moveit_config/config/)
  kaair.urdf.xacro        RSP + move_group 전용. kinematics only (include_ros2_control=false)
  arm_hw.urdf.xacro       arm CM 전용. arm ros2_control 블록만 포함 (control_manager 모듈이 사용)
  body_hw.urdf.xacro      body CM 전용. body ros2_control 블록만 포함 (control_manager 모듈이 사용)

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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
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

_controller_launch_dir = os.path.join(
    get_package_share_directory('kaair_controller'), 'launch')
if _controller_launch_dir not in sys.path:
    sys.path.insert(0, _controller_launch_dir)
from control_manager import build_control_manager  # noqa: E402


def launch_setup(context, *args, **kwargs):
    # ── 런타임 인자 resolve ────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
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

    # [C] arm/body Controller Manager (모듈화)
    cm = build_control_manager(
        use_fake_str=use_fake_str,
        hw_spec_file=hw_spec_file,
        initial_positions_file=initial_positions_file,
        ctrl_pkg=ctrl_pkg,
        moveit_pkg=moveit_pkg,
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
    # 이벤트 체인
    # ════════════════════════════════════════════════════════════════════════
    #
    # RSP 기동 → merger + static_TF + RViz (robot_description 발행 보장)
    #
    # control_manager 모듈 내부의 arm/body 이벤트 체인은 모듈 안에서 이미
    # 구성되어 있다 (cm['always_on_actions']). 이 파일은 그 결과물이 모두
    # 준비된 시점(cm['controllers_ready_action'] 종료)에 move_group 만
    # 얹으면 된다.

    return [
        # TF / 상태 발행 인프라
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[cm['merger_node'], static_tf_node, rviz_node],
        )),

        # arm/body Controller Manager (모듈에서 생성된 액션 전부)
        *cm['always_on_actions'],

        # 컨트롤러 준비 완료 = 전체 HW 활성화 완료 → move_group 기동
        RegisterEventHandler(OnProcessExit(
            target_action=cm['controllers_ready_action'],
            on_exit=[move_group_node],
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
