#!/usr/bin/env python3
"""
kaair_moveit_bringup.launch.py
────────────────────────────────
kaair_moveit_config/launch/kaair_moveit.launch.py 와 동일 인자(use_fake_hardware, use_gui, spec)로
MoveIt 스택을 기동하고, vision_runner.launch.py(헤드/핸드 카메라 드라이버)를 포함하며,
실제 팔 HW(use_fake_hardware=false)일 때만 kaair_bringup 의 xarm_bridge 를 추가한다.

xarm_bridge 의 robot_ip 는 kaair_bringup/config/robots/<spec> 의 arm.robot_ip 와 동일 규칙으로 로드한다
(server_worker_loader.py 와 동일).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node


def _load_arm_robot_ip_from_spec(spec_filename: str):
    """kaair_bringup/config/robots/<spec> 의 arm.robot_ip (없으면 None)."""
    bringup_pkg = get_package_share_directory('kaair_bringup')
    spec_path = os.path.join(bringup_pkg, 'config', 'robots', spec_filename)
    if not os.path.isfile(spec_path):
        return None, spec_path
    try:
        with open(spec_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return None, spec_path
    arm = data.get('arm') or {}
    ip = arm.get('robot_ip')
    if ip is None or str(ip).strip() == '':
        return None, spec_path
    return str(ip).strip().strip('"').strip("'"), spec_path


def _launch_setup(context, *args, **kwargs):
    spec_str = LaunchConfiguration('spec').perform(context)
    robot_ip, spec_path = _load_arm_robot_ip_from_spec(spec_str)

    bringup_share = get_package_share_directory('kaair_bringup')

    vision_launch_py = os.path.join(bringup_share, 'launch', 'vision_runner.launch.py')
    include_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vision_launch_py]),
        launch_arguments=[
            ('use_head_camera', LaunchConfiguration('use_head_camera')),
            ('use_hand_camera', LaunchConfiguration('use_hand_camera')),
        ],
    )

    moveit_share = get_package_share_directory('kaair_moveit_config')
    moveit_launch_py = os.path.join(moveit_share, 'launch', 'kaair_moveit.launch.py')

    include_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_launch_py]),
        launch_arguments=[
            ('use_fake_hardware', LaunchConfiguration('use_fake_hardware')),
            ('use_gui', LaunchConfiguration('use_gui')),
            ('spec', LaunchConfiguration('spec')),
        ],
    )

    xarm_bridge_params = {}
    if robot_ip:
        xarm_bridge_params['robot_ip'] = robot_ip
        print(f'[kaair_moveit_bringup] xarm_bridge robot_ip ← {spec_path} (arm.robot_ip)')
    else:
        print(
            f'[kaair_moveit_bringup] 스펙에 arm.robot_ip 없음 또는 파일 없음: {spec_path} — '
            'xarm_bridge 는 XARM_BRIDGE_ROBOT_IP 또는 노드 기본 파라미터로 동작'
        )

    xarm_bridge_node = Node(
        package='kaair_bringup',
        executable='xarm_bridge',
        name='xarm_bridge',
        output='screen',
        parameters=[xarm_bridge_params] if xarm_bridge_params else [],
        condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
    )

    return [include_moveit, include_vision, xarm_bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='false: 실제 HW / true: mock_components (xarm_bridge 는 false 일 때만 기동)',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='RViz2 실행 여부 (kaair_moveit.launch.py 로 전달)',
        ),
        DeclareLaunchArgument(
            'spec',
            default_value='kaair_specs_01.yaml',
            description='로봇 스펙 파일 이름 (kaair_bringup/config/robots/ 하위)',
        ),
        DeclareLaunchArgument(
            'use_head_camera',
            default_value='true',
            description='vision_runner.launch.py — 헤드 카메라(Orbbec) 포함',
        ),
        DeclareLaunchArgument(
            'use_hand_camera',
            default_value='true',
            description='vision_runner.launch.py — 핸드 카메라(RealSense) 포함',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
