from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('joy_dev',            default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_deadzone',        default_value='0.3'),
        DeclareLaunchArgument('joy_autorepeat_rate', default_value='50.0'),
        DeclareLaunchArgument('servo_node_name',     default_value='servo_server',
                              description='MoveIt Servo 노드 이름'),
        DeclareLaunchArgument('command_frame_id',    default_value='tool_tcp_link',
                              description='TwistStamped frame_id'),
        DeclareLaunchArgument('linear_scale',        default_value='1.0'),
        DeclareLaunchArgument('angular_scale',       default_value='1.0'),
        DeclareLaunchArgument('stop_to_start_sec',   default_value='0.4',
                              description='stop_servo → start_servo 사이 대기(초). 버퍼 비우기.'),
        DeclareLaunchArgument('hold_after_start_sec', default_value='1.0',
                              description='start_servo 후 zero-twist 강제 최소 시간(초).'),
        DeclareLaunchArgument('servo_cmd_frame',      default_value='link_base',
                              description='servo robot_link_command_frame (kaair_servo_config.yaml 과 일치)'),
        DeclareLaunchArgument('gripper_open',          default_value='0.05'),
        DeclareLaunchArgument('gripper_close',         default_value='0.0'),
        DeclareLaunchArgument('btn_gripper_toggle',    default_value='0',
                              description='gripper open/close 토글 버튼 인덱스'),
        DeclareLaunchArgument('gripper_init_open',     default_value='false',
                              description='시작 시 gripper 초기 상태 (true=open, false=close)'),
        DeclareLaunchArgument('btn_servo_toggle',      default_value='1',
                              description='servo start/stop + controller 전환 토글 버튼 인덱스'),
        DeclareLaunchArgument('tool_forward_controller', default_value='tool_forward_controller',
                              description='노드 시작 시 활성화할 tool forward controller 이름'),
        DeclareLaunchArgument('tool_original_controller', default_value='tool_controller',
                              description='노드 종료 시 복원할 원래 tool controller 이름'),
    ]

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev':             LaunchConfiguration('joy_dev'),
            'deadzone':        LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }],
    )

    spacemouse_servo_node = Node(
        package='kaair_bringup',
        executable='3d_master_controller',
        name='spacemouse_servo_controller',
        output='screen',
        parameters=[{
            'servo_node_name':     LaunchConfiguration('servo_node_name'),
            'command_frame_id':    LaunchConfiguration('command_frame_id'),
            'linear_scale':        LaunchConfiguration('linear_scale'),
            'angular_scale':       LaunchConfiguration('angular_scale'),
            'stop_to_start_sec':   LaunchConfiguration('stop_to_start_sec'),
            'hold_after_start_sec': LaunchConfiguration('hold_after_start_sec'),
            'servo_cmd_frame':      LaunchConfiguration('servo_cmd_frame'),
            'gripper_open':        LaunchConfiguration('gripper_open'),
            'gripper_close':       LaunchConfiguration('gripper_close'),
            'btn_gripper_toggle':        LaunchConfiguration('btn_gripper_toggle'),
            'gripper_init_open':         LaunchConfiguration('gripper_init_open'),
            'btn_servo_toggle':          LaunchConfiguration('btn_servo_toggle'),
            'tool_forward_controller':   LaunchConfiguration('tool_forward_controller'),
            'tool_original_controller':  LaunchConfiguration('tool_original_controller'),
            'publish_hz':  50.0,
            'deadband':    0.05,
            # SpaceMouse 축 인덱스 (필요시 조정)
            'axis_tx': 0, 'axis_ty': 2, 'axis_tz': 1,
            'axis_rx': 3, 'axis_ry': 5, 'axis_rz': 4,
            # 방향 반전 (+1 or -1)
            'sign_tx': 1, 'sign_ty': 1, 'sign_tz': 1,
            'sign_rx': 1, 'sign_ry': 1, 'sign_rz': 1,
        }],
    )

    return LaunchDescription(args + [joy_node, spacemouse_servo_node])
