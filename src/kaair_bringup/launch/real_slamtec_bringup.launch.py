"""
real_slamtec_bringup.launch.py

kaair_moveit.launch.py 와 동일한 2-Controller-Manager + MoveIt 구조를 사용한다.
가상 TF(slamware_map → base_footprint)는 실제 SLAMTEC/맵 TF 를 쓰므로 포함하지 않는다.

헤드·핸드 카메라(vision_runner) 및 server_worker_loader 는 기존과 동일하게 포함한다.
"""

import os
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

    arm_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
    body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')

    arm_hw_xacro = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')
    kaair_xacro = os.path.join(moveit_pkg, 'config', 'kaair.urdf.xacro')

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



    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )


    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
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
        arguments=['-d', os.path.join(bringup_pkg, 'rviz', 'slamtec_moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui),
    )

    arm_cm_params = [arm_description, arm_ctrl_yaml]

    arm_cm_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=arm_cm_params,
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

    arm_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm7_traj_controller',
            '--controller-manager', '/arm/controller_manager',
        ],
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
        arguments=['lift_controller', '--controller-manager', '/body/controller_manager'],
    )
    head_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['head_controller', '--controller-manager', '/body/controller_manager'],
    )
    tool_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tool_controller', '--controller-manager', '/body/controller_manager'],
    )

    lift_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'lift_forward_controller',
            '--controller-manager', '/body/controller_manager',
            '--inactive',
        ],
    )
    head_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'head_forward_controller',
            '--controller-manager', '/body/controller_manager',
            '--inactive',
        ],
    )
    tool_fwd_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'tool_forward_controller',
            '--controller-manager', '/body/controller_manager',
            '--inactive',
        ],
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

    return [
        rsp_node,
        vision_launch,
        server_worker_loader_node,
        RegisterEventHandler(OnProcessStart(
            target_action=rsp_node,
            on_start=[merger_node, rviz_node],
        )),

        arm_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=arm_cm_node,
            on_start=[arm_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=arm_jsb_spawner,
            on_start=[arm_ctrl_spawner, xarm7_fwd_spawner],
        )),

        body_cm_node,
        RegisterEventHandler(OnProcessStart(
            target_action=body_cm_node,
            on_start=[body_jsb_spawner],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=body_jsb_spawner,
            on_start=[
                lift_spawner, lift_fwd_spawner,
                head_spawner, head_fwd_spawner,
                tool_spawner, tool_fwd_spawner,
            ],
        )),

        # body side controllers ready → start MoveIt move_group
        RegisterEventHandler(OnProcessExit(
            target_action=tool_spawner,
            on_exit=[move_group_node],
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
        OpaqueFunction(function=launch_setup),
    ])
