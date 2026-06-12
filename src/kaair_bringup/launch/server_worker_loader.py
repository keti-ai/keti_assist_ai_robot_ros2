import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_spec(spec_filename: str):
    """kaair_bringup/config/robots/<spec> YAML을 로드한다."""
    bringup_pkg = get_package_share_directory("kaair_bringup")
    spec_path = os.path.join(bringup_pkg, "config", "robots", spec_filename)
    if not os.path.isfile(spec_path):
        return {}, spec_path
    try:
        with open(spec_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}, spec_path
    if not isinstance(data, dict):
        return {}, spec_path
    return data, spec_path


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

    spec_data, spec_path = _load_spec(spec_str)
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
            )
        ]
    )

    return worker_nodes


def generate_launch_description():
    spec_arg = DeclareLaunchArgument(
        "spec",
        default_value="kaair_specs_01.yaml",
        description="kaair_bringup/config/robots/ 하위 스펙 YAML (arm.robot_ip, mobile_bridge.POI/PIO 사용)",
    )
    return LaunchDescription(
        [
            spec_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
