from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node

def generate_launch_description():
    # 1. MoveIt 설정 빌드
    moveit_config = MoveItConfigsBuilder("kaair", package_name="kaair_moveit_config").to_moveit_configs()
    
    # 2. 기본 데모 런치 생성 (RViz, MoveGroup, 기본 스패너 포함)
    ld = generate_demo_launch(moveit_config)

    # 3. 추가하고 싶은 Head Controller 스패너 노드 정의
    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. 기존 LaunchDescription에 추가
    ld.add_action(head_controller_spawner)

    return ld