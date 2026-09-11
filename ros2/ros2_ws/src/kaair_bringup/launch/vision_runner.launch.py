from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # true면 femto_bolt.launch.py(네이티브 헤드 카메라 드라이버)와 depth_resizer를
    # 끄고, 대신 azure_kinect_container_humble(ROS_DOMAIN_ID=25)의
    # azure_kinect_ros_driver node가 만드는 원본 토픽 + body tracking 결과를
    # kaair_bringup의 azure_bridge_node로 도메인 브리지해서 /femto/*, /body_* 자리를
    # 그대로 대체한다. 둘 다 물리적으로 같은 Femto Bolt를 열려고 하면 USB 단독
    # 클레임이 충돌하므로 use_head_camera와 동시에 true로 두지 않는다.
    use_azure_arg = DeclareLaunchArgument(
        "use_azure", default_value="false",
        description="true면 네이티브 헤드 카메라(femto_bolt.launch.py) + depth_resizer 대신 "
                     "azure_kinect_container_humble(도메인 25)의 body tracking 결과를 "
                     "azure_bridge_node로 이 도메인에 브리지해서 femto/body_* 토픽을 대체한다."
    )

    azure_main_domain_id_arg = DeclareLaunchArgument(
        "azure_main_domain_id", default_value="25",
        description="use_azure=true일 때 azure_bridge_node가 구독할 "
                     "azure_kinect_ros_driver node의 ROS_DOMAIN_ID."
    )

    enable_depth_compressed_arg = DeclareLaunchArgument(
        "enable_depth_compressed", default_value="false",
        description="depth_resizer의 compressedDepth 구독/재발행 여부. "
                     "compressedDepth 구독자가 카메라 드라이버 쪽에 부하를 줘 "
                     "RGB 등 다른 스트림 지연을 유발할 때 false로 끈다."
    )

    # realsense2_camera_node는 color_qos/depth_qos 파라미터를 런타임에 동적으로
    # 선언하기 때문에 rs_launch.py의 launch_arguments로는 값이 전달되지 않는다
    # (rs_launch.py가 인식하는 launch 인자 목록에 없어서 무시됨). 대신
    # rs_launch.py가 지원하는 config_file 인자를 통해 파라미터 yaml을 그대로
    # 노드에 병합시켜 적용한다.
    hand_camera_qos_arg = DeclareLaunchArgument(
        "hand_camera_qos", default_value="reliable",
        choices=["reliable", "best_effort"],
        description="핸드 카메라(realsense) color/depth 이미지 스트림 QoS. "
                     "best_effort면 SENSOR_DATA(best effort) 프로파일을, "
                     "reliable이면 realsense2_camera 기본값(SYSTEM_DEFAULT, reliable)을 사용한다."
    )

    use_head_camera = LaunchConfiguration("use_head_camera")
    use_hand_camera = LaunchConfiguration("use_hand_camera")
    use_azure = LaunchConfiguration("use_azure")
    azure_main_domain_id = LaunchConfiguration("azure_main_domain_id")
    enable_depth_compressed = LaunchConfiguration("enable_depth_compressed")
    hand_camera_qos = LaunchConfiguration("hand_camera_qos")

    # use_head_camera가 true여도 use_azure가 true면 네이티브 헤드 카메라 쪽은 끈다
    # (azure_bridge_node가 그 자리를 대체하므로 같은 물리 카메라를 두 번 열지 않는다).
    head_camera_native_active = PythonExpression([
        "'", use_head_camera, "' == 'true' and '", use_azure, "' == 'false'"
    ])

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
        condition=IfCondition(head_camera_native_active),
    )

    # depth_registration(true)으로 카메라 드라이버가 이미 720x1280으로 정렬된
    # depth를 출력하므로, 리사이즈 없이 해상도(720x1280) 검증 후 그대로
    # /femto/depth/aligned 로 재발행 (해상도가 튀는 경우 drop)
    depth_align_node = Node(
        package='kaair_bringup',
        executable='depth_resizer',
        name='depth_resizer',
        output='screen',
        condition=IfCondition(head_camera_native_active),
        parameters=[{
            'input_topic': '/femto/depth/image_raw',
            'output_topic': '/femto/depth/aligned',
            'expected_height': 720,
            'expected_width': 1280,
            'enable_compressed_depth': ParameterValue(enable_depth_compressed, value_type=bool),
        }],
    )

    # use_azure=true일 때 femto_bolt.launch.py + depth_resizer 자리를 완전히 대체한다.
    # AZURE_TARGET_DOMAIN_ID는 azure_bridge_node가 별도로 안 주면 이 launch 프로세스의
    # 앰비언트 ROS_DOMAIN_ID로 알아서 fallback 하므로 여기서 따로 넘기지 않는다.
    # name= 을 주지 않는다: azure_bridge_node 실행 파일은 내부적으로 별도 이름의
    # rclcpp 노드 2개(azure_bridge_source, azure_bridge_target)를 만드는데, name=
    # 을 주면 __node: 리매핑이 전역으로 걸려 둘 다 같은 이름으로 충돌한다
    # ("Publisher already registered for provided node name" 경고).
    azure_bridge_node = Node(
        package='kaair_bringup',
        executable='azure_bridge_node',
        output='screen',
        condition=IfCondition(use_azure),
        additional_env={'AZURE_MAIN_DOMAIN_ID': azure_main_domain_id},
    )

    hand_package_dir = get_package_share_directory('realsense2_camera')
    hand_launch_dir = os.path.join(hand_package_dir, 'launch')

    # color_qos/depth_qos는 rs_launch.py의 launch 인자로 노출되어 있지 않아
    # launch_arguments로 직접 넘겨도 무시된다. rs_launch.py가 그대로 노드 파라미터로
    # 병합해주는 config_file 인자를 통해 best_effort일 때만 SENSOR_DATA(best effort)
    # yaml을 얹고, reliable이면 config_file을 비워 realsense2_camera 기본값(reliable)을 쓴다.
    hand_camera_qos_config_file = os.path.join(
        get_package_share_directory('kaair_bringup'),
        'config', 'common', 'realsense_hand_qos_best_effort.yaml'
    )
    hand_camera_config_file = PythonExpression([
        "'", hand_camera_qos_config_file, "' if '", hand_camera_qos, "' == 'best_effort' else \"''\""
    ])

    hand_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '/hand',
            'pointcloud.enable': 'true',
            'publish_tf': 'false',
            'output': 'own_log',
            'config_file': hand_camera_config_file,
        }.items(),
        condition=IfCondition(use_hand_camera),
    )

    # Launch description
    ld = LaunchDescription([
        use_head_camera_arg,
        use_hand_camera_arg,
        use_azure_arg,
        azure_main_domain_id_arg,
        enable_depth_compressed_arg,
        hand_camera_qos_arg,
        TimerAction(period=0.0, actions=[GroupAction([hand_launch_include])]),
        TimerAction(period=0.0, actions=[GroupAction([azure_bridge_node])]),
        TimerAction(period=2.0, actions=[GroupAction([head_launch_include])]),
        # femto 카메라 드라이버가 완전히 올라와 토픽을 publish하기 시작할 시간을
        # 확보한 뒤 depth_align_node를 실행 (head 카메라 2.0s + 여유 3.0s)
        TimerAction(period=5.0, actions=[GroupAction([depth_align_node])]),
    ])

    return ld