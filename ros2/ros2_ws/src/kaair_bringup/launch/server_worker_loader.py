import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uf_ros_lib.uf_robot_utils import generate_robot_api_params

_bringup_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _bringup_launch_dir not in sys.path:
    sys.path.insert(0, _bringup_launch_dir)
from robot_spec_utils import load_robot_spec  # noqa: E402


def _load_arm_robot_ip_from_spec(spec_data):
    """스펙의 arm.robot_ip 를 반환 (없으면 None)."""
    arm = spec_data.get("arm") or {}
    ip = arm.get("robot_ip")
    if ip is None or str(ip).strip() == "":
        return None
    return str(ip).strip().strip('"').strip("'")


def _driver_ns(context):
    """드라이버가 서비스/토픽을 여는 네임스페이스.

    xarm_driver_node 는 hw_ns 파라미터를 prefix+hw_ns 로 받아 그 아래에
    clean_error, set_mode 같은 서비스를 연다. xarm_bridge 가 그 서비스를
    호출해야 하므로 두 노드가 같은 문자열을 봐야 한다.
    """
    prefix = LaunchConfiguration("xarm_prefix").perform(context).strip("/")
    hw_ns = LaunchConfiguration("xarm_hw_ns").perform(context).strip("/")
    return "{}{}".format(prefix, hw_ns)


def _xarm_driver_node(context, robot_ip):
    """xarm_api 드라이버(ufactory_driver) 노드.

    `ros2 launch xarm_api xarm7_driver.launch.py` 를 따로 띄우는 대신 여기서
    같이 올린다. 그 런치파일이 하는 일은 결국 xarm_api/launch/_robot_driver.launch.py
    의 xarm_driver_node 하나이고, 나머지는 dof/robot_type 같은 인자를 채워주는
    래퍼라서 노드를 직접 구성하는 편이 파라미터 출처가 분명하다.

    파라미터 파일은 xarm_api 의 config/xarm_params.yaml (첫 줄에 "수정하지 말라"고
    적혀 있고, xarm_ws 를 다시 받으면 덮어써진다) 대신 이 패키지의
    config/common/kaair_xarm7_param.yaml 을 기본값으로 넘긴다.

    generate_robot_api_params 를 그대로 쓰는 이유는 ros_namespace 를 붙이고
    최상위 키를 노드 이름(ufactory_driver)에 맞춰주는 처리가 그 안에 있어서,
    yaml 을 parameters 에 직접 꽂는 것보다 상위 런치의 네임스페이스 변화에 강하다.
    """
    bringup_pkg = get_package_share_directory("kaair_bringup")
    params_path = os.path.join(bringup_pkg, "config", "common", "kaair_xarm7_param.yaml")

    robot_params = generate_robot_api_params(
        params_path,
        LaunchConfiguration("xarm_user_params_file", default="").perform(context),
        LaunchConfiguration("ros_namespace", default="").perform(context),
        node_name="ufactory_driver",
    )

    prefix = LaunchConfiguration("xarm_prefix").perform(context).strip("/")

    print(f"[server_worker_loader] ufactory_driver params ← {params_path}")

    return Node(
        package="xarm_api",
        name="ufactory_driver",
        executable="xarm_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            robot_params,
            {
                "robot_ip": robot_ip,
                # report_type 은 드라이버가 컨트롤러에서 상태를 받아오는 스트림이다.
                # normal(30001)/rich(30002) 는 5Hz, dev(30003) 는 약 100Hz.
                "report_type": LaunchConfiguration("xarm_report_type").perform(context),
                "dof": 7,
                "add_gripper": False,
                "add_bio_gripper": False,
                "hw_ns": _driver_ns(context),
                "prefix": prefix,
                "baud_checkset": True,
                "default_gripper_baud": 2000000,
                # 평면 파라미터 joint_states_rate 는 0 보다 클 때만 이긴다
                # (xarm_driver.cpp: joint_state_rate_ = rate > 0 ? rate : joint_state_rate_).
                # -1 로 두면 yaml 의 joint_states.rate 가 그대로 살아난다.
                "joint_states_rate": -1,
            },
        ],
    )


def _load_place_config_from_spec(spec_data):
    """스펙의 mobile_bridge.POI/PIO 값으로 config/maps/<yaml> 경로를 반환한다."""
    mobile_bridge = spec_data.get("mobile_bridge") or {}
    poi_filename = mobile_bridge.get("POI") or mobile_bridge.get("PIO")
    if poi_filename is None or str(poi_filename).strip() == "":
        return None

    poi_filename = str(poi_filename).strip().strip('"').strip("'")
    bringup_pkg = get_package_share_directory("kaair_bringup")
    if os.path.isabs(poi_filename):
        return poi_filename
    return os.path.join(bringup_pkg, "config", "maps", poi_filename)


def launch_setup(context, *args, **kwargs):
    spec_str = LaunchConfiguration("spec").perform(context)

    spec_data, spec_path = load_robot_spec(spec_str)
    robot_ip = _load_arm_robot_ip_from_spec(spec_data)
    place_config_file = _load_place_config_from_spec(spec_data)

    worker_nodes = []
    driver_ns = "/" + _driver_ns(context)

    if LaunchConfiguration("use_xarm_driver").perform(context).lower() in ("true", "1"):
        if robot_ip:
            worker_nodes.append(_xarm_driver_node(context, robot_ip))
        else:
            print(
                "[server_worker_loader] arm.robot_ip 가 없어 ufactory_driver 를 띄우지 않는다 "
                "— 드라이버는 robot_ip 없이는 뜨지 못한다"
            )

    location_server_params = {}
    if place_config_file:
        location_server_params["config_file"] = place_config_file
        print(
            f"[server_worker_loader] location_server config_file ← "
            f"{place_config_file} (mobile_bridge.POI/PIO)"
        )
    else:
        print(
            f"[server_worker_loader] 스펙에 mobile_bridge.POI/PIO 없음: "
            f"{spec_path} — location_server 기본 config 사용"
        )

    # /arm/init_set 은 이제 ufactory_driver 의 서비스(clean_error, motion_enable,
    # set_mode, set_state)를 호출하는 방식이라 robot_ip 를 알 필요가 없다.
    # 컨트롤러 IP 를 아는 노드는 드라이버 하나뿐이다.
    worker_nodes.append(
        Node(
            package="kaair_bringup",
            executable="xarm_bridge",
            name="xarm_bridge",
            output="screen",
            parameters=[{"driver_ns": driver_ns}],
        )
    )

    # 툴 끝점(tool_tcp_link)의 절대좌표 + 6DOF 속도를 데이터 수집/VLA 용으로 발행.
    # flange→TCP 오프셋은 robot_state_publisher 의 robot_description 에서
    # link_eef→tool_tcp_link 를 읽어 쓰므로 별도 설정값이 없다.
    worker_nodes.append(
        Node(
            package="kaair_bringup",
            executable="eef_state_publisher",
            name="eef_state_publisher",
            output="screen",
            parameters=[{
                "twist_frame": LaunchConfiguration("eef_twist_frame"),
            }],
        )
    )

    worker_nodes.extend(
        [
            Node(
                package="kaair_bringup",
                executable="arm_move_action_server",
                name="arm_move_action_server",
                output="screen",
            ),
            Node(
                package="kaair_bringup",
                executable="lift_move_action_server",
                name="lift_move_action_server",
                output="screen",
            ),
            Node(
                package="kaair_bringup",
                executable="object_marker_server",
                name="object_marker_server",
                output="screen",
            ),
            # Node(
            #     package="kaair_bringup",
            #     executable="depth_pointcloud_publisher",
            #     name="depth_pointcloud_publisher",
            #     output="screen",
            # ),
            Node(  
                package="kaair_bringup",
                executable="robot_pose_publisher",
                name="robot_pose_publisher",
                output="screen",
            ),
            Node(
                package="kaair_bringup",
                executable="head_move_server",
                name="head_move_server",
                output="screen",
            ),
            Node(
                package="kaair_bringup",
                executable="controller_mode_switcher",
                name="controller_mode_switcher",
                output="screen",
            ),
            # servo on/off 상태를 RViz 우측 하단에 텍스트로 표시.
            # 3d_master_controller(SpaceMouse)는 3D 마우스가 연결됐을 때만
            # 실행되는 노드라 그쪽에서 직접 RViz 로 발행하면 평상시(3D 마우스
            # 미연결)에는 상태를 알 수 없다. 이 노드는 항상 떠 있으면서
            # default_text 로 기본값(PLANNING)을 먼저 보여주고, 3d_master_controller
            # 가 실행 중일 때만 /servo_mode/state_text 로 실제 상태를 갱신받는다.
            # (범용 String→OverlayText 브리지라 나중에 다른 상태 문자열도
            #  input_topic/overlay_topic 파라미터만 바꿔 재사용할 수 있다.)
            Node(
                package="kaair_bringup",
                executable="text_status_overlay",
                name="servo_status_overlay",
                output="screen",
                parameters=[{
                    "input_topic": "/servo_mode/state_text",
                    "overlay_topic": "/servo_mode/status_overlay",
                    "default_level": "info",
                    "default_text": "ARM: PLANNING (config)",
                }],
            ),
        ]
    )

    return worker_nodes


def generate_launch_description():
    spec_arg = DeclareLaunchArgument(
        "spec",
        default_value="kaair_specs_01.yaml",
        description="kaair_bringup/config/robots/ 하위 스펙 YAML (arm.robot_ip, mobile_bridge.type/POI/PIO 사용)",
    )
    return LaunchDescription(
        [
            spec_arg,
            DeclareLaunchArgument(
                "use_xarm_driver",
                default_value="true",
                description="xarm_api 드라이버(ufactory_driver) 실행 여부. "
                            "false 로 두면 예전처럼 xarm7_driver.launch.py 를 따로 띄운다.",
            ),
            DeclareLaunchArgument(
                "xarm_report_type",
                default_value="normal",
                description="ufactory_driver 리포트 스트림: normal/rich/dev",
            ),
            DeclareLaunchArgument(
                "xarm_hw_ns",
                default_value="xarm",
                description="드라이버 토픽/서비스 네임스페이스 (기본값 기준 /xarm/joint_states)",
            ),
            DeclareLaunchArgument(
                "xarm_prefix",
                default_value="",
                description="조인트 이름 프리픽스. URDF 쪽 prefix 와 맞춰야 한다.",
            ),
            DeclareLaunchArgument(
                "eef_twist_frame",
                default_value="tool",
                description="/observation/eef_twist 를 어떤 축으로 쓸지. "
                            "tool=tool_tcp_link 축(기본), base=arm_base 축. "
                            "둘 다 TCP 한 점의 속도이고 표현 축만 다르다.",
            ),
            DeclareLaunchArgument(
                "xarm_user_params_file",
                default_value="",
                description="kaair_xarm7_param.yaml 위에 덮어쓸 추가 파라미터 YAML (선택).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
