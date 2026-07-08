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

_bringup_launch_dir = os.path.dirname(os.path.abspath(__file__))
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
        f'[vla_moveit_bringup] mobile_bridge.type={get_mobile_bridge_type(spec_data)!r} '
        f'→ {os.path.basename(kaair_xacro)} ← {spec_path}'
    )

    # ── Controller YAML ────────────────────────────────────────────────────
    arm_ctrl_yaml  = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    # ── xacro 경로 (robot_variant 로 선택) ────────────────────────────────
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
        .planning_pipelines(
            pipelines=['pilz_industrial_motion_planner'],
            default_planning_pipeline='pilz_industrial_motion_planner',
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .sensors_3d(file_path='config/sensors_3d.yaml')
        .to_moveit_configs()
    )

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
    xarm7_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm7_forward_controller',
            '--controller-manager', '/arm/controller_manager',
            '--inactive',
        ],
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
            on_start=[arm_ctrl_spawner, xarm7_fwd_spawner],
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