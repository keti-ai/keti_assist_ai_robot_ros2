"""
kaair_moveit_servo.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
kaair_moveit.launch.py 의 2-CM 구조에 MoveIt Servo 를 추가한 완전한 환경.

  ┌─────────────────────────────────────────────────────────────────────┐
  │  [kaair_moveit.launch.py 동일 구성]                                  │
  │  RSP · joint_state_merger · static_TF · RViz                       │
  │  /arm/controller_manager  (xarm7_traj_controller ACTIVE)           │
  │  /body/controller_manager (lift/head/tool_controller ACTIVE)       │
  │  move_group (tool_spawner 종료 후 기동)                             │
  ├─────────────────────────────────────────────────────────────────────┤
  │  [추가 구성]                                                         │
  │  servo_server  (standalone servo_node_main)                         │
  │    moveit_servo::servo_node_main                                    │
  │      입력:  /servo_server/delta_twist_cmds  (TwistStamped)         │
  │             /servo_server/delta_joint_cmds  (JointJog)             │
  │      출력:  /arm/xarm7_traj_controller/joint_trajectory            │
  │    서비스:  /servo_server/start_servo   (Trigger)                  │
  │             /servo_server/pause_servo   (Trigger)                  │
  │             /servo_server/unpause_servo (Trigger)                  │
  │             /servo_server/stop_servo    (Trigger)                  │
  ├─────────────────────────────────────────────────────────────────────┤
  │  moveit_servo_mode_switcher                                         │
  │    ~/switch_servo_mode_cmd (Bool)  /  ~/switch_servo_mode (SetBool)│
  │    true  → start_servo   (SERVO 모드)                              │
  │    false → pause_servo   (PLANNING 모드, 기본값)                    │
  │    ~/mode (String latched): "planning" | "servo"                   │
  └─────────────────────────────────────────────────────────────────────┘

모드 전환 개념
  PLANNING 모드 (기본):
    move_group 이 FollowJointTrajectory action 으로 arm 을 제어.
    servo_server 는 paused 상태 (명령을 받아도 출력하지 않음).

  SERVO 모드:
    servo_server 가 /arm/xarm7_traj_controller/joint_trajectory 에 직접 publish.
    move_group 도 계속 실행되지만 servo 명령이 traj controller 를 선점.
    incoming_command_timeout 이후 자동 halting → PLANNING 재개 가능.

robot_variant 인자:
  (제거됨) spec YAML 의 mobile_bridge.type 으로 URDF/SRDF 자동 선택
    clobot/clober → kaair_clober.urdf.xacro + kaair_clober.srdf
    slamtec         → kaair.urdf.xacro + kaair.srdf

Launch 인자
  use_fake_hardware  true | false  (default: false)
  use_gui            true | false  (default: true)
  spec               kaair_specs_*.yaml  (default: kaair_specs_01.yaml)
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def _load_yaml(package_name: str, relative_path: str) -> dict:
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, relative_path), 'r') as f:
        return yaml.safe_load(f)


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

    # ── Controller YAML ────────────────────────────────────────────────────
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── xacro 경로 (mobile_bridge.type 로 선택) ───────────────────────────
    arm_hw_xacro  = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')

    # ── robot_description 생성 헬퍼 ────────────────────────────────────────
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
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # ── MoveIt Servo 설정 ─────────────────────────────────────────────────
    servo_yaml = _load_yaml('kaair_moveit_config', 'config/kaair_servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # ════════════════════════════════════════════════════════════════════════
    # 노드 정의 (kaair_moveit.launch.py 와 동일한 기반 구성)
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

    # ── arm Controller Manager ────────────────────────────────────────────
    arm_cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=[arm_description, arm_ctrl_yaml],
        output='both',
    )
    arm_jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/arm/controller_manager'],
    )
    arm_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm7_traj_controller',
                   '--controller-manager', '/arm/controller_manager'],
    )


    # ── body Controller Manager ───────────────────────────────────────────
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
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/body/controller_manager'],
    )
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
    tool_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tool_forward_controller',
                   '--controller-manager', '/body/controller_manager',
                   '--inactive'],
    )


    # ════════════════════════════════════════════════════════════════════════
    # [추가] MoveIt Servo 관련 노드
    # ════════════════════════════════════════════════════════════════════════

    # [F] servo_node_main (standalone)
    #   ComposableNodeContainer 대신 독립 프로세스로 실행.
    #   move_group 기동 후 시작해 planning scene monitor 를 secondary 로 attach.
    #   is_primary_planning_scene_monitor: false (servo config 에 설정됨).
    #
    #   서비스:
    #     /servo_server/start_servo   → SERVO 모드 시작
    #     /servo_server/pause_servo   → 일시정지 (PLANNING 모드 복귀)
    #     /servo_server/unpause_servo → 일시정지에서 재개
    #     /servo_server/stop_servo    → 완전 중지 (재시작 불가)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',
        output='screen',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )


    # ════════════════════════════════════════════════════════════════════════
    # 이벤트 체인
    # ════════════════════════════════════════════════════════════════════════
    #
    # RSP 기동 → merger + static_TF + RViz
    #
    # arm:  arm_cm_node → arm_jsb_spawner → arm_ctrl_spawner + xarm7_fwd_spawner
    # body: body_cm_node → body_jsb_spawner → lift/head/tool + fwd spawners
    #
    # tool_spawner 종료 (body HW 전체 준비 완료)
    #   → move_group + ctrl_mode_switcher 동시 기동
    #
    # move_group 기동
    #   → servo_container + servo_mode_switcher 기동
    #     (planning scene monitor 구독 가능 시점)

    return [
        # ── 기반 인프라 ────────────────────────────────────────────────────
        rsp_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, static_tf_node, rviz_node],
        )),

        # ── arm CM 이벤트 체인 ─────────────────────────────────────────────
        arm_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=arm_cm_node,
            on_start=[arm_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=arm_jsb_spawner,
            on_start=[arm_ctrl_spawner],
        )),

        # ── body CM 이벤트 체인 ────────────────────────────────────────────
        body_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=body_cm_node,
            on_start=[body_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=body_jsb_spawner,
            on_start=[
                lift_spawner,
                head_spawner,
                tool_spawner, tool_fwd_spawner,
            ],
        )),

        # ── tool_spawner 종료 → move_group + controller_mode_switcher ──────
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node],
        )),

        # ── move_group 기동 → servo_node + servo_mode_switcher ─────────────
        RegisterEventHandler(OnProcessStart(
            target_action=move_group_node,
            on_start=[servo_node],
        )),

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
