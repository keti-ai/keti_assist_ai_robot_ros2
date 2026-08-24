import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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

    xarm_bridge_params = {}
    if robot_ip:
        xarm_bridge_params["robot_ip"] = robot_ip
        print(f"[server_worker_loader] xarm_bridge robot_ip ← {spec_path} (arm.robot_ip)")
    else:
        print(
            f"[server_worker_loader] 스펙에 arm.robot_ip 없음 또는 파일 없음: "
            f"{spec_path} — xarm_bridge 는 XARM_BRIDGE_ROBOT_IP 또는 빈 파라미터로 동작"
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

    worker_nodes.append(
        Node(
            package="kaair_bringup",
            executable="xarm_bridge",
            name="xarm_bridge",
            output="screen",
            parameters=[xarm_bridge_params] if xarm_bridge_params else [],
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
            OpaqueFunction(function=launch_setup),
        ]
    )
