import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# --- [중요] 파일 구조(tree)에 맞춘 임포트 경로 설정 ---
package_name = 'kaair_moveit_config'

try:
    # 1. 빌드 후 설치된(install/share) 패키지 루트 경로를 가져옵니다.
    pkg_share_dir = get_package_share_directory(package_name)
    
    # 🌟 파일 구조가 share/kaair_moveit_config/kaair_moveit_config/utils.py 이므로
    # pkg_share_dir를 path에 추가해야 'import kaair_moveit_config.utils'가 가능합니다.
    if pkg_share_dir not in sys.path:
        sys.path.insert(0, pkg_share_dir)
    
    # 이제 패키지 경로를 포함하여 임포트합니다.
    from kaair_moveit_config.utils import get_kaair_full_stack
    
except Exception as e:
    print(f" [주의] utils.py를 임포트하는 중 오류 발생: {e}")
    # src 폴더에서 직접 실행하는 환경을 위한 추가 경로 (선택 사항)
    # sys.path.insert(0, os.path.join(os.getcwd(), 'src', package_name))
    # from kaair_moveit_config.utils import get_kaair_full_stack

# --------------------------------------------------

def generate_launch_description():
    # 1. Launch Arguments 선언
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", 
        default_value="false",
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

    # 4. RViz Node 정의
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", 
            os.path.join(get_package_share_directory(package_name), "config", "moveit.rviz")
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_gui)
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