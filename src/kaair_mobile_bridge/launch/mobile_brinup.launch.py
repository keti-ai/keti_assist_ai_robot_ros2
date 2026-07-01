import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 3. 외부 XML 런치 파일 포함 (Slamware SDK Server)
    # slamware_ros_sdk 패키지에 포함된 xml 파일을 실행합니다.
    slamware_sdk_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slamware_ros_sdk'),
                'launch', 'slamware_ros_sdk_server_node.xml'
            )
        ])
    )

    # 4. Bridge 노드 실행
    # 이전에 작성한 slamtec_bridge_node를 실행하며 yaml 경로를 인자로 넘깁니다.
    slamtec_bridge_node = Node(
        package="kaair_mobile_bridge",
        executable="slamtec_bridge_node",
        output="screen",
    )

    # 5. 최종 LaunchDescription 구성
    return LaunchDescription([
        slamware_sdk_launch,
        slamtec_bridge_node
    ])