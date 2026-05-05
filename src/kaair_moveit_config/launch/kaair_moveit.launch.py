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
  │     spawner: joint_state_broadcaster, xarm7_traj_controller         │
  ├─────────────────────────────────────────────────────────────────────┤
  │  /body/controller_manager                                           │
  │  └─ body_hw.urdf.xacro                                             │
  │       fake: kaair.ros2_control.xacro → mock_components/GenericSystem│
  │       real: lift/head/tool HW interfaces                            │
  │     kaair_controller/config/body_controllers.yaml                  │
  │     spawner: joint_state_broadcaster, lift/head/tool_controller     │
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
"""

import os
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
from uf_ros_lib.uf_robot_utils import generate_robot_api_params


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

    # ── Controller YAML ────────────────────────────────────────────────────
    # fake/real 공통. HW 구분은 URDF xacro 플러그인으로만 처리.
    # 컨트롤러 이름(xarm7_traj_controller)은 fake/real 동일하다.
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── xacro 경로 ─────────────────────────────────────────────────────────
    arm_hw_xacro  = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')
    kaair_xacro   = os.path.join(moveit_pkg, 'config', 'kaair.urdf.xacro')

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
        .robot_description_semantic(file_path='config/kaair.srdf')
        # fake/real 모두 동일한 moveit_controllers.yaml 사용
        # (xarm7_traj_controller 이름으로 통일)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # ── xArm API 파라미터 (실제 HW + arm CM 에서만 사용) ─────────────────
    xarm_api_params = {}
    if not use_fake:
        xarm_api_params = generate_robot_api_params(
            os.path.join(get_package_share_directory('xarm_api'),
                         'config', 'xarm_params.yaml'),
            os.path.join(get_package_share_directory('xarm_api'),
                         'config', 'xarm_user_params.yaml'),
            '',
            node_name='ufactory_driver',
        )

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
    # arm Controller Manager (namespace: /arm)
    # ════════════════════════════════════════════════════════════════════════
    arm_cm_params = [arm_description, arm_ctrl_yaml]
    if xarm_api_params:
        arm_cm_params.append(xarm_api_params)

    arm_cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=arm_cm_params,
        # 실제 HW 에서 UFRobotSystemHardware 는 실시간성이 필요하므로 nice 레벨 높임
        prefix=[] if use_fake else ['nice -n -20'],
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

    # ════════════════════════════════════════════════════════════════════════
    # body Controller Manager (namespace: /body)
    # ════════════════════════════════════════════════════════════════════════
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

        # arm Controller Manager
        arm_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=arm_cm_node,
            on_start=[arm_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=arm_jsb_spawner,
            on_start=[arm_ctrl_spawner],
        )),

        # body Controller Manager
        body_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=body_cm_node,
            on_start=[body_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=body_jsb_spawner,
            on_start=[lift_spawner, head_spawner, tool_spawner],
        )),

        # 모든 HW 활성화 완료 후 move_group 기동
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
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
