# people_marker.launch.py
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────
# 설정 (여기만 수정)
# ─────────────────────────────────────────
TF_FRAME_ID = 'slamware_map'
TF_CHILD_FRAME_ID = 'people'

POSES = {
    'spot_b': {'x': 5.2, 'y': 0.8,  'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    'spot_a': {'x': 0.64, 'y': 2.89, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57},
}

MARKER = {
    'topic': '/people_marker',
    'mesh_resource': 'package://kaair_description/meshes/sitting.stl',
    'scale': 0.001,
    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.7071, 'w': 0.7071},  # 메시 정면을 TF +x에 맞추는 보정
    'color': {'r': 0.2, 'g': 0.6, 'b': 1.0, 'a': 1.0},
}
# ─────────────────────────────────────────


def launch_setup(context, *args, **kwargs):
    pose_name = LaunchConfiguration('pose').perform(context)
    if pose_name not in POSES:
        raise RuntimeError(
            f"Unknown pose '{pose_name}'. Available: {', '.join(POSES.keys())}"
        )
    p = POSES[pose_name]

    # static TF
    tf_args = []
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        tf_args += [f'--{key}', str(float(p[key]))]
    tf_args += ['--frame-id', TF_FRAME_ID, '--child-frame-id', TF_CHILD_FRAME_ID]

    # marker
    s = float(MARKER['scale'])
    marker = {
        'header': {'frame_id': TF_CHILD_FRAME_ID},
        'ns': 'person',
        'id': 0,
        'type': 10,      # MESH_RESOURCE
        'action': 0,
        'mesh_resource': MARKER['mesh_resource'],
        'pose': {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': MARKER['orientation'],
        },
        'scale': {'x': s, 'y': s, 'z': s},
        'color': MARKER['color'],
    }
    marker_yaml = yaml.safe_dump(
        {'markers': [marker]}, default_flow_style=True, width=10**9
    ).strip()

    return [
        LogInfo(msg=f"[people_marker] pose='{pose_name}' -> {p}"),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='people_static_tf',
            arguments=tf_args,
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                '--qos-durability', 'transient_local',
                '--keep-alive', '86400',
                MARKER['topic'],
                'visualization_msgs/msg/MarkerArray',
                marker_yaml,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pose', default_value='spot_a',
            choices=list(POSES.keys()),
            description='사용할 사람 위치 프리셋',
        ),
        OpaqueFunction(function=launch_setup),
    ])