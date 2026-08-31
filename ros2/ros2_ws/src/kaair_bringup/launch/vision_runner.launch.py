from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_head_camera_arg = DeclareLaunchArgument(
        "use_head_camera", default_value="true",
        description="헤드 카메라 사용 여부"
    )

    use_hand_camera_arg = DeclareLaunchArgument(
        "use_hand_camera", default_value="true",
        description="핸드 카메라 사용 여부"
    )

    enable_depth_compressed_arg = DeclareLaunchArgument(
        "enable_depth_compressed", default_value="false",
        description="depth_resizer의 compressedDepth 구독/재발행 여부. "
                     "compressedDepth 구독자가 카메라 드라이버 쪽에 부하를 줘 "
                     "RGB 등 다른 스트림 지연을 유발할 때 false로 끈다."
    )

    use_head_camera = LaunchConfiguration("use_head_camera")
    use_hand_camera = LaunchConfiguration("use_hand_camera")
    enable_depth_compressed = LaunchConfiguration("enable_depth_compressed")

    # Include launch files
    head_package_dir = get_package_share_directory('orbbec_camera')
    head_launch_file_dir = os.path.join(head_package_dir, 'launch')
    head_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(head_launch_file_dir, 'femto_bolt.launch.py')
        ),
        launch_arguments={
            'camera_name': 'femto',
            'depth_registration': 'true',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'color_qos': 'default',
            'depth_qos': 'default',
            'point_cloud_qos': 'default',
            'publish_tf': 'false',
            'output': 'own_log',
        }.items(),
        condition=IfCondition(use_head_camera),
    )

    # depth_registration(true)으로 카메라 드라이버가 이미 720x1280으로 정렬된
    # depth를 출력하므로, 리사이즈 없이 해상도(720x1280) 검증 후 그대로
    # /femto/depth/aligned 로 재발행 (해상도가 튀는 경우 drop)
    depth_align_node = Node(
        package='kaair_bringup',
        executable='depth_resizer',
        name='depth_resizer',
        output='screen',
        condition=IfCondition(use_head_camera),
        parameters=[{
            'input_topic': '/femto/depth/image_raw',
            'output_topic': '/femto/depth/aligned',
            'expected_height': 720,
            'expected_width': 1280,
            'enable_compressed_depth': ParameterValue(enable_depth_compressed, value_type=bool),
        }],
    )

    hand_package_dir = get_package_share_directory('realsense2_camera')
    hand_launch_dir = os.path.join(hand_package_dir, 'launch')

    hand_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '/hand',
            'pointcloud.enable': 'true',
            'publish_tf': 'false',
            'output': 'own_log',
        }.items(),
        condition=IfCondition(use_hand_camera),
    )

    # Launch description
    ld = LaunchDescription([
        use_head_camera_arg,
        use_hand_camera_arg,
        enable_depth_compressed_arg,
        TimerAction(period=0.0, actions=[GroupAction([hand_launch_include])]),
        TimerAction(period=2.0, actions=[GroupAction([head_launch_include])]),
        # femto 카메라 드라이버가 완전히 올라와 토픽을 publish하기 시작할 시간을
        # 확보한 뒤 depth_align_node를 실행 (head 카메라 2.0s + 여유 3.0s)
        TimerAction(period=5.0, actions=[GroupAction([depth_align_node])]),
    ])

    return ld