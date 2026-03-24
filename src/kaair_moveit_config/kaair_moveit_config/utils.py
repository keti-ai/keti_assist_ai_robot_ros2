import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_robot_api_params

def get_kaair_full_stack(use_fake_hardware, spec, mode="robot"):
    # 1. MoveIt Config Builder (Mappings 포함)
    moveit_config = (
        MoveItConfigsBuilder("kaair", package_name="kaair_moveit_config")
        .robot_description(
            file_path=os.path.join(get_package_share_directory("kaair_description"), "urdf", "robot.urdf.xacro"),
            mappings={
                "use_fake_hardware": use_fake_hardware, 
                "mode": mode,
                "hw_spec_file": spec, # Substitution 객체 그대로 전달
            }
        )
        .robot_description_semantic(file_path="config/kaair.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"], default_planning_pipeline="ompl")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # 2. xArm API Params
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        '', node_name='ufactory_driver'
    )

    # 3. Core Nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("kaair_controller"), "config", "kaair_controllers.yaml"),
            robot_params,
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    lift_initializer_node = Node(
        package="kaair_driver",
        executable="lift_initializer",
        name="lift_initializer",
        output="screen",
        condition=UnlessCondition(use_fake_hardware)
    )

    # 4. Spawners
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    spawner_list = [
        Node(package="controller_manager", executable="spawner", arguments=["lift_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["xarm7_traj_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["head_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["tool_controller"]),
    ]

    # 5. Chained Events
    # [Real HW] Initializer Exit -> Control Node Start
    real_hw_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=lift_initializer_node,
            on_exit=[LogInfo(msg="Homing Done! Starting Manager..."), ros2_control_node]
        ),
        condition=UnlessCondition(use_fake_hardware)
    )

    # [Fake HW] RSP Start -> Control Node Start (Immediate)
    fake_hw_chain = RegisterEventHandler(
        OnProcessStart(target_action=robot_state_pub_node, on_start=[ros2_control_node]),
        condition=IfCondition(use_fake_hardware)
    )

    # Manager -> JSB -> Others -> MoveGroup
    spawner_chain = [
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node, on_start=[jsb_spawner])),
        RegisterEventHandler(OnProcessStart(target_action=jsb_spawner, on_start=spawner_list)),
        RegisterEventHandler(OnProcessStart(target_action=spawner_list[1], on_start=[move_group_node])), # arm_spawner 트리거
    ]

    # Return common nodes and the sequence
    sequence = [
        robot_state_pub_node,
        lift_initializer_node,
        real_hw_chain,
        fake_hw_chain,
        *spawner_chain
    ]
    
    return sequence, moveit_config