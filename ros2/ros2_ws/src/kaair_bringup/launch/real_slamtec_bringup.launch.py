"""
real_slamtec_bringup.launch.py

kaair_moveit.launch.py 와 동일한 2-Controller-Manager + MoveIt 구조를 사용한다.
가상 TF(slamware_map → base_footprint)는 실제 SLAMTEC/맵 TF 를 쓰므로 포함하지 않는다.

헤드·핸드 카메라(vision_runner), server_worker_loader, MoveIt Servo(servo_module,
move_group 기동 후 시작)를 기존과 동일하게 포함한다.
"""

import os
import sys
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

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
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake = use_fake_str.lower() in ('true', '1', 'yes')
    spec_str = LaunchConfiguration('spec').perform(context)
    use_gui = LaunchConfiguration('use_gui')

    moveit_pkg = get_package_share_directory('kaair_moveit_config')
    ctrl_pkg = get_package_share_directory('kaair_controller')
    bringup_pkg = get_package_share_directory('kaair_bringup')

    hw_spec_file = os.path.join(bringup_pkg, 'config', 'robots', spec_str)
    initial_positions_file = os.path.join(moveit_pkg, 'config', 'initial_positions.yaml')

    kaair_xacro = os.path.join(moveit_pkg, 'config', 'kaair.urdf.xacro')

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
        .robot_description_semantic(file_path='config/kaair.srdf')
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

    # 실제 HW 에서 UFRobotSystemHardware 는 실시간성이 필요하므로 nice 레벨 높임
    cm = build_control_manager(
        use_fake_str=use_fake_str,
        hw_spec_file=hw_spec_file,
        initial_positions_file=initial_positions_file,
        ctrl_pkg=ctrl_pkg,
        moveit_pkg=moveit_pkg,
        arm_cm_prefix=[] if use_fake else ['nice -n -20'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(bringup_pkg, 'rviz', 'slamtec_moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui),
    )

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

    # MoveIt Servo (모듈화, move_group 기동 후 시작)
    servo = build_moveit_servo(
        moveit_config=moveit_config,
        move_group_node=move_group_node,
    )

    return [
        rsp_node,
        vision_launch,
        server_worker_loader_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[cm['merger_node'], rviz_node],
        )),

        *cm['always_on_actions'],

        # body side controllers ready → start MoveIt move_group
        RegisterEventHandler(OnProcessExit(
            target_action=cm['controllers_ready_action'],
            on_exit=[move_group_node],
        )),

        *servo['actions'],
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
        OpaqueFunction(function=launch_setup),
    ])
