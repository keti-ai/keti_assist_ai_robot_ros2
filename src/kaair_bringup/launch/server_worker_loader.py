import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_arm_robot_ip_from_spec(spec_filename: str):
    """kaair_bringup/config/robots/<spec> 의 arm.robot_ip 를 반환 (없으면 None)."""
    bringup_pkg = get_package_share_directory("kaair_bringup")
    spec_path = os.path.join(bringup_pkg, "config", "robots", spec_filename)
    if not os.path.isfile(spec_path):
        return None, spec_path
    try:
        with open(spec_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return None, spec_path
    arm = data.get("arm") or {}
    ip = arm.get("robot_ip")
    if ip is None or str(ip).strip() == "":
        return None, spec_path
    return str(ip).strip().strip('"').strip("'"), spec_path


def launch_setup(context, *args, **kwargs):
    spec_str = LaunchConfiguration("spec").perform(context)

    robot_ip, spec_path = _load_arm_robot_ip_from_spec(spec_str)

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
                executable="location_server",
                name="location_server",
                output="screen",
            ),
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
                executable="controller_mode_switcher",
                name="controller_mode_switcher",
                output="screen",
            ),
            Node(
                package="kaair_bringup",
                executable="object_marker_server",
                name="object_marker_server",
                output="screen",
            ),
        ]
    )

    return worker_nodes


def generate_launch_description():
    spec_arg = DeclareLaunchArgument(
        "spec",
        default_value="kaair_specs_01.yaml",
        description="kaair_bringup/config/robots/ 하위 스펙 YAML (arm.robot_ip 사용)",
    )
    return LaunchDescription(
        [
            spec_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
