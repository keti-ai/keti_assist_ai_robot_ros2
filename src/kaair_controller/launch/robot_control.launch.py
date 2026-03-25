import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# xArm의 파라미터 파싱 유틸리티 임포트
from uf_ros_lib.uf_robot_utils import generate_robot_api_params


def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Start robot with fake hardware (mock_components)"
    )
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start RViz2 and Joint State Publisher GUI"
    )
    use_gui = LaunchConfiguration("use_gui")

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='robot',
        description='Visualization mode: robot(default), lift, head, arm, slamtec'
    )
    mode = LaunchConfiguration('mode')
    is_arm_mode = PythonExpression(["'", mode, "' in ['arm', 'robot']"])
    is_lift_mode = PythonExpression(["'", mode, "' in ['lift', 'robot']"])
    is_head_mode = PythonExpression(["'", mode, "' in ['head', 'robot']"])
    is_tool_mode = PythonExpression(["'", mode, "' in ['tool', 'robot']"])


    spec_arg = DeclareLaunchArgument(
        'spec',default_value='kaair_specs_01.yaml',
        description='Robot Hardware Spec for URDF'
    )
    spec = LaunchConfiguration('spec')

    # Spec 파일 경로
    hw_spec_file = PathJoinSubstitution([FindPackageShare('kaair_bringup'), 'config', 'robots', spec])

    xacro_path = PathJoinSubstitution([FindPackageShare('kaair_description'), 'urdf', 'robot.urdf.xacro'])
    robot_description_content = Command([
        'xacro ', xacro_path, 
        ' mode:=', mode,
        ' hw_spec_file:=', hw_spec_file,
        ' use_fake_hardware:=', use_fake_hardware,
    ])
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("kaair_controller"), "config", "kaair_controllers.yaml"]
    )
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('kaair_description'),
        'rviz',
        PythonExpression(["'", mode, "_config.rviz'"])
    ])


    # 3. 공통 노드
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_gui)
    )


    # 4. xArm API 구동을 위한 전용 파라미터 로드
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # 컨트롤러 노드
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, robot_params],
        output="both",
    )

    # # [A] 단독 초기화 노드: 가짜 하드웨어가 "아닐 때(Unless)"만 실행됨
    # lift_initializer_node = Node(
    #     package="kaair_driver",
    #     executable="lift_initializer",
    #     name="lift_initializer",
    #     output="screen",
    #     condition=UnlessCondition(use_fake_hardware)
    # )


    # 스포너 노드
    spawner_nodes = []
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=["joint_state_broadcaster"],
    )
    lift_spawner = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["lift_controller"],
        condition=IfCondition(is_lift_mode)
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm7_traj_controller"],
        condition=IfCondition(is_arm_mode)
    )
    head_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller"],
        condition=IfCondition(is_head_mode)
    )
    tool_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tool_controller"],
        condition=IfCondition(is_tool_mode)
    )

    spawner_nodes.append(lift_spawner)
    spawner_nodes.append(arm_spawner)
    spawner_nodes.append(head_spawner)
    spawner_nodes.append(tool_spawner)


    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        mode_arg,
        spec_arg,

        robot_state_pub_node,
        rviz_node,

        control_node,
        # 💡 스포너 연쇄 실행 1 (Fake 컨트롤러 시작 시)
        RegisterEventHandler(
            OnProcessStart(target_action=control_node, on_start=[jsb_spawner])
        ),
        # 💡 스포너 연쇄 실행 2 (Real 컨트롤러 시작 시)
        RegisterEventHandler(
            OnProcessStart(target_action=jsb_spawner, on_start=[*spawner_nodes])
        ),
    ])