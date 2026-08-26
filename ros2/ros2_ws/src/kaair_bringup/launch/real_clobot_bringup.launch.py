"""
real_clobot_bringup.launch.py

real_slamtec_bringup.launch.py 와 동일한 2-Controller-Manager + MoveIt +
MoveIt Servo 구조를 사용한다. 제어(controller/servo) 측면은 완전히 동일하고,
차이는 모바일 베이스 연동 방식뿐이다:
  - Slamware는 실제 SLAMTEC/맵 TF를 도메인 브리징으로 직접 받아썼지만,
    Clobot은 로봇 PC의 rosbridge_server(websocket)에 접속하는
    kaair_mobile_bridge의 clobot_bridge_node를 이 launch 파일에서 같이 띄운다.
    이 노드가 slamware_map -> base_footprint TF와 /map,
    /navigate_to_pose 액션을 이 도메인에 그대로 노출해준다
    (자세한 내용은 kaair_mobile_bridge/scripts/clobot_bridge.py 참고).
  - RViz 설정 파일은 real_slamtec_bringup과 동일한 slamtec_moveit.rviz를
    그대로 쓴다 (Map 디스플레이 Topic이 이미 /map이라 그대로 맞는다).
  - MoveIt URDF/SRDF는 kaair_moveit_bringup.launch.py와 동일하게
    robot_spec_utils.resolve_moveit_urdf_paths()로 spec의
    mobile_bridge.type을 보고 선택한다 (clobot/clober →
    kaair_clober.urdf.xacro + kaair_clober.srdf, 그 외 → kaair.urdf.xacro +
    kaair.srdf). real_slamtec_bringup.launch.py처럼 kaair.srdf를 하드코딩하지
    않는다 — spec 기본값(kaair_specs_01.yaml)의 mobile_bridge.type이
    "clobot"이므로 하드코딩하면 실제로는 안 맞는 SRDF를 불러오게 된다.

헤드·핸드 카메라(vision_runner) 및 server_worker_loader 는 기존과 동일하게 포함한다.
"""

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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


def _load_yaml(package_name: str, relative_path: str) -> dict:
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, relative_path), 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake = use_fake_str.lower() in ('true', '1', 'yes')
    spec_str = LaunchConfiguration('spec').perform(context)
    use_gui = LaunchConfiguration('use_gui')

    moveit_pkg = get_package_share_directory('kaair_moveit_config')
    ctrl_pkg = get_package_share_directory('kaair_controller')
    bringup_pkg = get_package_share_directory('kaair_bringup')

    hw_spec_file = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    spec_data, spec_path = load_robot_spec(spec_str)
    kaair_xacro, srdf_file = resolve_moveit_urdf_paths(moveit_pkg, spec_data)
    print(
        f'[real_clobot_bringup] mobile_bridge.type={get_mobile_bridge_type(spec_data)!r} '
        f'→ {os.path.basename(kaair_xacro)} ← {spec_path}'
    )

    arm_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    arm_hw_xacro = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')

    def make_description(xacro_path, extra=''):
        cmd = (
            f'xacro {xacro_path}'
            f' use_fake_hardware:={use_fake_str}'
            f' hw_spec_file:={hw_spec_file}'
        )
        if extra:
            cmd += ' ' + extra
        return {'robot_description': ParameterValue(Command(cmd), value_type=str)}

    arm_description = make_description(
        arm_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )
    body_description = make_description(
        body_hw_xacro,
        f'initial_positions_file:={initial_positions_file}',
    )

    moveit_config = (
        MoveItConfigsBuilder('kaair', package_name='kaair_moveit_config')
        .robot_description(
            file_path=kaair_xacro,
            mappings={
                'use_fake_hardware': use_fake_str,
                'mode': 'robot',
                'hw_spec_file': hw_spec_file,
                'initial_positions_file': initial_positions_file,
                'include_ros2_control': 'false',
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

    # ── MoveIt Servo 설정 ─────────────────────────────────────────────────
    servo_yaml = _load_yaml('kaair_moveit_config', 'config/kaair_servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output={'stdout': 'log', 'stderr': 'log'},
        parameters=[moveit_config.to_dict()],
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[moveit_config.robot_description],
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(bringup_pkg, 'rviz', 'clobot_moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui),
    )

    # ── MoveIt Servo (standalone servo_node_main) ─────────────────────────
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

    # ── Clobot 모바일 브리지 (rosbridge websocket, kaair_mobile_bridge) ────
    # 로봇 PC에서 rosbridge_server가 떠 있어야 한다(ws://<robot_host>:<robot_port>).
    # slamware_map -> base_footprint TF, /map, /navigate_to_pose 액션을
    # 이 도메인에 노출한다.
    clobot_bridge_node = Node(
        package='kaair_mobile_bridge',
        executable='clobot_bridge_node',
        output='screen',
        arguments=[
            '--robot-host', LaunchConfiguration('robot_host'),
            '--robot-port', LaunchConfiguration('robot_port'),
        ],
    )

    # ── arm/body Controller Manager (+ lift_initializer 게이트) ───────────
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
        arm_cm_prefix=None if use_fake else ['nice -n -20'],
        arm_cm_output={'stdout': 'log', 'stderr': 'log'},
        body_cm_output={'stdout': 'log', 'stderr': 'log'},
    )
    tool_spawner = cm_handles['body_active_spawners']['tool_controller']

    use_head_camera = LaunchConfiguration('use_head_camera')
    use_hand_camera = LaunchConfiguration('use_hand_camera')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kaair_bringup'), 'launch', 'vision_runner.launch.py',
            ])
        ]),
        launch_arguments={
            'use_head_camera': use_head_camera,
            'use_hand_camera': use_hand_camera,
        }.items(),
    )

    spec_cfg = LaunchConfiguration('spec')

    server_worker_loader_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kaair_bringup'), 'launch', 'server_worker_loader.py',
            ])
        ]),
        launch_arguments={'spec': spec_cfg}.items(),
    )

    return [
        rsp_node,
        vision_launch,
        server_worker_loader_node,
        clobot_bridge_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node],
        )),

        # arm/body Controller Manager (+ lift_initializer 게이트)
        *cm_actions,

        # body side controllers ready → start MoveIt move_group
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node],
        )),

        # move_group 기동 → servo_node + RViz
        # RViz 는 move_group 기동 후에 띄운다: MotionPlanning 디스플레이가
        # move_group 서비스/액션에 곧바로 연결되지 않으면 처음에 경고가 뜨고
        # 체크박스를 껐다 켜야 정상화되는 문제가 있어, move_group 이 이미 뜬
        # 뒤에 RViz 를 시작해 그 문제를 피한다.
        RegisterEventHandler(OnProcessStart(
            target_action=move_group_node,
            on_start=[servo_node, rviz_node],
        )),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_head_camera',
            default_value='true',
            description='헤드 카메라 사용 여부',
        ),
        DeclareLaunchArgument(
            'use_hand_camera',
            default_value='true',
            description='핸드 카메라 사용 여부',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='false: 실제 HW / true: mock_components FakeSystem',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='RViz2 실행 여부',
        ),
        DeclareLaunchArgument(
            'spec',
            default_value='kaair_specs_01.yaml',
            description='로봇 스펙 파일 (kaair_bringup/config/robots/)',
        ),
        DeclareLaunchArgument(
            'robot_host',
            default_value='192.168.11.200',
            description='Clobot 로봇 PC의 rosbridge_server 주소',
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='9090',
            description='rosbridge_server websocket 포트',
        ),
        OpaqueFunction(function=launch_setup),
    ])
