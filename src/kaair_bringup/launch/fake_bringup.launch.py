import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 🌟 중요: kaair_moveit_config 패키지의 share 경로를 명시적으로 추가
package_name = 'kaair_moveit_config'
try:
    pkg_share_path = get_package_share_directory(package_name)
    # 현재 구조: share/kaair_moveit_config/kaair_moveit_config/utils.py
    # 따라서 pkg_share_path를 추가해야 'import kaair_moveit_config.utils'가 가능합니다.
    if pkg_share_path not in sys.path:
        sys.path.insert(0, pkg_share_path)
    
    from kaair_moveit_config.utils import get_kaair_full_stack
except Exception as e:
    # 빌드 전 src 경로에서도 찾을 수 있게 fallback 추가 (선택 사항)
    print(f"DEBUG: {package_name} share directory not found, checking src...")
    # sys.path.insert(0, os.path.join(os.getcwd(), 'src', package_name))
    # from kaair_moveit_config.utils import get_kaair_full_stack


def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", 
        default_value="true",
        description="가짜 하드웨어(mock_components) 사용 여부"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", 
        default_value="true",
        description="RViz2 실행 여부"
    )
    spec_arg = DeclareLaunchArgument(
        "spec", 
        default_value="kaair_specs_01.yaml",
        description="로봇 하드웨어 스펙 파일 (.yaml)"
    )

    # LaunchConfiguration 매핑
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_gui = LaunchConfiguration("use_gui")
    spec = LaunchConfiguration("spec")

    # 2. Spec 파일 경로 (Substitution 객체 생성)
    hw_spec_file = PathJoinSubstitution([
        FindPackageShare('kaair_bringup'), 'config', 'robots', spec
    ])

    # 3. 매크로 함수 호출 (핵심 제어 시퀀스 및 MoveIt 설정 획득)
    # utils.py 내 정의된 get_kaair_full_stack을 사용합니다.
    control_sequence, moveit_config = get_kaair_full_stack(use_fake_hardware, hw_spec_file)


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kaair_bringup"), "rviz", "slam_moveit.rviz"]
    )


    # 4. RViz Node 정의
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )



    # 5. 최종 LaunchDescription 구성
    return LaunchDescription([
        use_fake_hardware_arg,
        use_gui_arg,
        spec_arg,
        
        # utils에서 반환된 [initializer, manager, spawners, move_group] 시퀀스 언패킹
        *control_sequence,
        
        # 시각화를 위한 RViz 노드 추가
        rviz_node
    ])