import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import generate_robot_api_params


def launch_setup(context, *args, **kwargs):
    # ── Runtime 값 resolve ────────────────────────────────────────────────────
    use_fake_str = LaunchConfiguration('use_fake_hardware').perform(context)
    use_fake     = use_fake_str.lower() in ('true', '1', 'yes')
    use_gui      = LaunchConfiguration('use_gui')
    mode         = LaunchConfiguration('mode')
    spec         = LaunchConfiguration('spec')

    # ── 경로 ─────────────────────────────────────────────────────────────────
    ctrl_pkg_path    = get_package_share_directory('kaair_controller')
    moveit_pkg_path  = get_package_share_directory('kaair_moveit_config')
    desc_pkg_path    = get_package_share_directory('kaair_description')

    hw_spec_file = PathJoinSubstitution([
        FindPackageShare('kaair_bringup'), 'config', 'robots', spec
    ])
    initial_positions_file = os.path.join(
        moveit_pkg_path, 'config', 'initial_positions.yaml'
    )

    # ── 모드별 controller YAML 선택 ───────────────────────────────────────────
    #   fake : kaair_moveit_config/config/fake_{arm,body}_controllers.yaml
    #   real : kaair_controller/config/{arm,body}_controllers.yaml
    if use_fake:
        arm_controllers_yaml  = os.path.join(moveit_pkg_path, 'config', 'fake_arm_controllers.yaml')
        body_controllers_yaml = os.path.join(moveit_pkg_path, 'config', 'fake_body_controllers.yaml')
    else:
        arm_controllers_yaml  = os.path.join(ctrl_pkg_path, 'config', 'arm_controllers.yaml')
        body_controllers_yaml = os.path.join(ctrl_pkg_path, 'config', 'body_controllers.yaml')

    # ── xacro 경로 ────────────────────────────────────────────────────────────
    full_xacro    = os.path.join(desc_pkg_path, 'urdf', 'robot.urdf.xacro')
    arm_hw_xacro  = os.path.join(moveit_pkg_path, 'config', 'arm_hw.urdf.xacro')
    body_hw_xacro = os.path.join(moveit_pkg_path, 'config', 'body_hw.urdf.xacro')

    # ── robot_description 생성 ────────────────────────────────────────────────
    # RSP: mode 기반 전체 URDF (TF / 시각화 용도)
    full_robot_description = {
        'robot_description': Command([
            'xacro ', full_xacro,
            ' mode:=', mode,
            ' hw_spec_file:=', hw_spec_file,
            ' use_fake_hardware:=', use_fake_str,
        ])
    }

    # arm CM: arm ros2_control 블록만 포함
    arm_robot_description = {
        'robot_description': Command([
            'xacro ', arm_hw_xacro,
            ' hw_spec_file:=', hw_spec_file,
            ' use_fake_hardware:=', use_fake_str,
        ])
    }

    # body CM: lift / head / tool ros2_control 블록만 포함
    body_robot_description = {
        'robot_description': Command([
            'xacro ', body_hw_xacro,
            ' hw_spec_file:=', hw_spec_file,
            ' use_fake_hardware:=', use_fake_str,
            ' initial_positions_file:=', initial_positions_file,
        ])
    }

    # ── xArm API 파라미터 (arm CM 에만 전달) ─────────────────────────────────
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # ── 모드별 조건 ───────────────────────────────────────────────────────────
    is_arm_cm_mode  = PythonExpression(["'", mode, "' in ['arm', 'robot']"])
    is_body_cm_mode = PythonExpression(["'", mode, "' in ['lift', 'head', 'tool', 'robot']"])

    is_arm_mode  = PythonExpression(["'", mode, "' in ['arm', 'robot']"])
    is_lift_mode = PythonExpression(["'", mode, "' in ['lift', 'robot']"])
    is_head_mode = PythonExpression(["'", mode, "' in ['head', 'robot']"])
    is_tool_mode = PythonExpression(["'", mode, "' in ['tool', 'robot']"])

    # ── 노드 정의 ─────────────────────────────────────────────────────────────

    # [A] Robot State Publisher – mode 기반 URDF, 하나만 실행
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[full_robot_description],
    )

    # [B] RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('kaair_description'),
            'rviz',
            PythonExpression(["'", mode, "_config.rviz'"])
        ])],
        condition=IfCondition(use_gui),
    )

    # [C] joint_state_merger
    #     /arm/joint_states (joint1-7) + /body/joint_states (lift/head/tool)
    #     → /joint_states (RSP 가 구독)
    #
    #     mode 가 arm 전용이면 /body/joint_states 소스가 비어도 무해하며,
    #     body 전용이면 /arm/joint_states 가 비어도 무해하다.
    joint_state_merger = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': ['/arm/joint_states', '/body/joint_states'],
            'rate': 50,
        }],
        remappings=[('robot_description', '/robot_description')],
    )

    # [D] arm_controller_manager (namespace: /arm)
    arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='arm',
        parameters=[arm_robot_description, arm_controllers_yaml, robot_params],
        output='both',
        condition=IfCondition(is_arm_cm_mode),
    )

    # [E] body_controller_manager (namespace: /body)
    body_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='body',
        parameters=[body_robot_description, body_controllers_yaml],
        output='both',
        condition=IfCondition(is_body_cm_mode),
    )

    # ── Spawner 노드 ──────────────────────────────────────────────────────────
    arm_ctrl_name = 'arm_controller' if use_fake else 'xarm7_traj_controller'

    arm_jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/arm/controller_manager'],
        condition=IfCondition(is_arm_cm_mode),
    )
    arm_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[arm_ctrl_name,
                   '--controller-manager', '/arm/controller_manager'],
        condition=IfCondition(is_arm_mode),
    )

    body_jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/body/controller_manager'],
        condition=IfCondition(is_body_cm_mode),
    )
    lift_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['lift_controller',
                   '--controller-manager', '/body/controller_manager'],
        condition=IfCondition(is_lift_mode),
    )
    head_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['head_controller',
                   '--controller-manager', '/body/controller_manager'],
        condition=IfCondition(is_head_mode),
    )
    tool_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['tool_controller',
                   '--controller-manager', '/body/controller_manager'],
        condition=IfCondition(is_tool_mode),
    )

    # ── 이벤트 체인 ───────────────────────────────────────────────────────────
    #
    # arm  경로: arm_control_node  시작 → arm_jsb  → arm_ctrl
    # body 경로: body_control_node 시작 → body_jsb → lift / head / tool
    #
    # condition 이 False 인 노드는 프로세스가 시작되지 않으므로
    # 해당 OnProcessStart 이벤트도 발생하지 않아 안전하다.
    event_arm_cm_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=arm_control_node, on_start=[arm_jsb_spawner])
    )
    event_arm_jsb_to_ctrl = RegisterEventHandler(
        OnProcessStart(target_action=arm_jsb_spawner, on_start=[arm_ctrl_spawner])
    )

    event_body_cm_to_jsb = RegisterEventHandler(
        OnProcessStart(target_action=body_control_node, on_start=[body_jsb_spawner])
    )
    event_body_jsb_to_ctrls = RegisterEventHandler(
        OnProcessStart(
            target_action=body_jsb_spawner,
            on_start=[lift_spawner, head_spawner, tool_spawner],
        )
    )

    return [
        robot_state_pub_node,
        rviz_node,
        joint_state_merger,
        arm_control_node,
        body_control_node,
        event_arm_cm_to_jsb,
        event_arm_jsb_to_ctrl,
        event_body_cm_to_jsb,
        event_body_jsb_to_ctrls,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='true: mock_components FakeSystem / false: real HW drivers'
        ),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='RViz2 실행 여부'
        ),
        DeclareLaunchArgument(
            'mode', default_value='robot',
            description='mode: robot | arm | lift | head | tool | slamtec'
        ),
        DeclareLaunchArgument(
            'spec', default_value='kaair_specs_01.yaml',
            description='로봇 하드웨어 스펙 파일 (.yaml)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
