"""
kaair_moveit_bringup.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
kaair_moveit_config/launch/kaair_moveit_servo.launch.py 와 동일한 구성에
서버 워커(팔/리프트/헤드 액션 서버, xarm_bridge 등)와 비전(헤드/핸드 카메라)을
추가한 완전한 bringup.

  ┌─────────────────────────────────────────────────────────────────────┐
  │  [kaair_moveit_servo.launch.py 와 동일 구성]                        │
  │  RSP · joint_state_merger · static_TF · RViz                       │
  │  control_manager 모듈: /arm, /body controller_manager + 스포너들    │
  │  move_group + servo_module (moveit_servo)                          │
  ├─────────────────────────────────────────────────────────────────────┤
  │  [추가 구성]                                                         │
  │  server_worker_loader.py : xarm_bridge, arm/lift/head 액션서버 등   │
  │  vision_runner.launch.py : 헤드(Femto)/핸드(RealSense) 카메라       │
  └─────────────────────────────────────────────────────────────────────┘

Launch 인자
  use_fake_hardware  true | false  (default: false)
  use_gui            true | false  (default: true)
  spec               kaair_specs_*.yaml  (default: kaair_specs_01.yaml)
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

_bringup_launch_dir = os.path.dirname(os.path.abspath(__file__))
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

_moveit_launch_dir = os.path.join(
    get_package_share_directory('kaair_moveit_config'), 'launch')
if _moveit_launch_dir not in sys.path:
    sys.path.insert(0, _moveit_launch_dir)
from servo_module import build_moveit_servo  # noqa: E402


def launch_setup(context, *args, **kwargs):
    # ── 런타임 인자 resolve ────────────────────────────────────────────────
    use_fake_str    = LaunchConfiguration('use_fake_hardware').perform(context)
    spec_str        = LaunchConfiguration('spec').perform(context)
    use_gui         = LaunchConfiguration('use_gui')

    # ── 패키지 경로 ────────────────────────────────────────────────────────
    moveit_pkg  = get_package_share_directory('kaair_moveit_config')
    ctrl_pkg    = get_package_share_directory('kaair_controller')
    bringup_pkg = get_package_share_directory('kaair_bringup')

    hw_spec_file           = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    spec_data, spec_path = load_robot_spec(spec_str)
    kaair_xacro, srdf_file = resolve_moveit_urdf_paths(moveit_pkg, spec_data)
    print(
        f'[kaair_moveit_bringup] mobile_bridge.type={get_mobile_bridge_type(spec_data)!r} '
        f'→ {os.path.basename(kaair_xacro)} ← {spec_path}'
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
        .robot_description_semantic(file_path=srdf_file)
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

    # [D] Static TF
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

    # [F] MoveIt Servo (모듈화, move_group 기동 후 시작)
    servo = build_moveit_servo(
        moveit_config=moveit_config,
        move_group_node=move_group_node,
    )

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

        *servo['actions'],

        # ── 추가 서비스 / 비전 ──────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kaair_bringup'),
                    'launch', 'server_worker_loader.py',
                )
            ),
            launch_arguments={'spec': LaunchConfiguration('spec')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('kaair_bringup'),
                    'launch', 'vision_runner.launch.py',
                )
            ),
        ),
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
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위, mobile_bridge.type 으로 URDF 선택)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
