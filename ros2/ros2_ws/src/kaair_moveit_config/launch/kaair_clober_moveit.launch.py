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
  │  kaair_controller/launch/control_manager.py (모듈)                  │
  │  └─ /arm/controller_manager, /body/controller_manager,              │
  │     각 spawner, controller_mode_switcher 를 모두 구성한다.          │
  └─────────────────────────────────────────────────────────────────────┘

xacro 파일 역할 정리 (kaair_moveit_config/config/)
  kaair_clober.urdf.xacro  RSP + move_group 전용. kinematics only (clober base)
  arm_hw.urdf.xacro        arm CM 전용. arm ros2_control 블록만 포함 (control_manager 모듈이 사용)
  body_hw.urdf.xacro       body CM 전용. body ros2_control 블록만 포함 (control_manager 모듈이 사용)

Launch 인자
  use_fake_hardware  true | false  (default: false)
  use_gui            true | false  (default: true)
  spec               kaair_specs_*.yaml  (default: kaair_specs_01.yaml)
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

    hw_spec_file           = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    # Clober 전용 RSP/move_group URDF
    kaair_xacro = os.path.join(moveit_pkg, 'config', 'kaair_clober.urdf.xacro')

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

    # ── ros2_control/MoveIt >= Jazzy 파라미터 스키마 보정 (kaair_moveit.launch.py 참고) ──
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
    return [
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[cm['merger_node'], static_tf_node, rviz_node],
        )),

        *cm['always_on_actions'],

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
            default_value='kaair_specs_01.yaml',
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
