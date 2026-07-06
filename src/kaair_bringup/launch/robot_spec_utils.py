"""Helpers for reading kaair_bringup/config/robots/*.yaml robot spec files."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

CLOBOT_BRIDGE_TYPES = frozenset({'clobot', 'clober'})
SLAMTEC_BRIDGE_TYPES = frozenset({'slamtec'})


def load_robot_spec(spec_filename: str) -> tuple[dict, str]:
    """Load a robot spec YAML. Returns (data, absolute_path)."""
    bringup_pkg = get_package_share_directory('kaair_bringup')
    spec_path = os.path.join(bringup_pkg, 'config', 'robots', spec_filename)
    if not os.path.isfile(spec_path):
        return {}, spec_path
    try:
        with open(spec_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}, spec_path
    if not isinstance(data, dict):
        return {}, spec_path
    return data, spec_path


def get_mobile_bridge_type(spec_data: dict) -> str:
    """Return normalized mobile_bridge.type (defaults to slamtec)."""
    mobile_bridge = spec_data.get('mobile_bridge') or {}
    bridge_type = mobile_bridge.get('type')
    if bridge_type is None or str(bridge_type).strip() == '':
        return 'slamtec'
    return str(bridge_type).strip().strip('"').strip("'").lower()


def use_clober_base(spec_data: dict) -> bool:
    """True when mobile_bridge.type selects the Clober/Clobot mobile base."""
    bridge_type = get_mobile_bridge_type(spec_data)
    if bridge_type in CLOBOT_BRIDGE_TYPES:
        return True
    if bridge_type in SLAMTEC_BRIDGE_TYPES:
        return False
    raise ValueError(
        f"Unknown mobile_bridge.type: {bridge_type!r}. "
        f"Expected one of: {sorted(CLOBOT_BRIDGE_TYPES | SLAMTEC_BRIDGE_TYPES)}"
    )


def resolve_moveit_urdf_paths(moveit_pkg: str, spec_data: dict) -> tuple[str, str]:
    """Return (kaair_xacro_abs_path, srdf_relative_path) from robot spec."""
    if use_clober_base(spec_data):
        return (
            os.path.join(moveit_pkg, 'config', 'kaair_clober.urdf.xacro'),
            'config/kaair_clober.srdf',
        )
    return (
        os.path.join(moveit_pkg, 'config', 'kaair.urdf.xacro'),
        'config/kaair.srdf',
    )
