"""
Keyboard → MoveIt Servo 텔레오퍼레이션 런치

키 입력은 이 런치를 실행한 터미널(/dev/tty)에서 직접 읽는다.
  w/s : +x/-x   a/d : -y/+y   r/f : -z/+z   q/e : -rz/+rz
  1   : gripper toggle         2   : servo ON/OFF toggle
  space : stop

  ros2 launch kaair_bringup keyboard_master_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('servo_node_name',     default_value='servo_server',
                              description='MoveIt Servo 노드 이름'),
        DeclareLaunchArgument('command_frame_id',    default_value='tool_tcp_link',
                              description='TwistStamped frame_id'),
        DeclareLaunchArgument('linear_speed',        default_value='0.05',
                              description='w/s, a/d, r/f 키 선속도 (m/s)'),
        DeclareLaunchArgument('angular_speed',       default_value='0.3',
                              description='q/e 키 각속도 (rad/s)'),
        DeclareLaunchArgument('key_first_timeout_sec',  default_value='0.6',
                              description='첫 키 입력 후 hold 유지 시간 (OS key repeat delay 보다 커야 함)'),
        DeclareLaunchArgument('key_repeat_timeout_sec', default_value='0.15',
                              description='key repeat 중 이 시간 입력이 없으면 키를 뗀 것으로 판단'),
        DeclareLaunchArgument('stall_detect_sec',    default_value='1.0',
                              description='키 명령 중 로봇 정지가 이 시간 이상이면 명령 차단 (0 이하면 끔)'),
        DeclareLaunchArgument('stop_to_start_sec',   default_value='0.4',
                              description='stop_servo → start_servo 사이 대기(초). 버퍼 비우기.'),
        DeclareLaunchArgument('hold_after_start_sec', default_value='1.0',
                              description='start_servo 후 zero-twist 강제 최소 시간(초).'),
        DeclareLaunchArgument('gripper_open',          default_value='0.05'),
        DeclareLaunchArgument('gripper_close',         default_value='0.0'),
        DeclareLaunchArgument('gripper_init_open',     default_value='false',
                              description='시작 시 gripper 초기 상태 (true=open, false=close)'),
        DeclareLaunchArgument('auto_start_servo',      default_value='true',
                              description='true 면 노드 기동 시 바로 SERVO ON'),
        DeclareLaunchArgument('tool_forward_controller', default_value='tool_forward_controller',
                              description='SERVO ON 시 활성화할 tool forward controller 이름'),
        DeclareLaunchArgument('tool_original_controller', default_value='tool_controller',
                              description='SERVO OFF/종료 시 복원할 원래 tool controller 이름'),
    ]

    keyboard_servo_node = Node(
        package='kaair_bringup',
        executable='keyboard_master_controller',
        name='keyboard_servo_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'servo_node_name':        LaunchConfiguration('servo_node_name'),
            'command_frame_id':       LaunchConfiguration('command_frame_id'),
            'linear_speed':           LaunchConfiguration('linear_speed'),
            'angular_speed':          LaunchConfiguration('angular_speed'),
            'key_first_timeout_sec':  LaunchConfiguration('key_first_timeout_sec'),
            'key_repeat_timeout_sec': LaunchConfiguration('key_repeat_timeout_sec'),
            'stall_detect_sec':       LaunchConfiguration('stall_detect_sec'),
            'stop_to_start_sec':      LaunchConfiguration('stop_to_start_sec'),
            'hold_after_start_sec':   LaunchConfiguration('hold_after_start_sec'),
            'gripper_open':           LaunchConfiguration('gripper_open'),
            'gripper_close':          LaunchConfiguration('gripper_close'),
            'gripper_init_open':      LaunchConfiguration('gripper_init_open'),
            'auto_start_servo':       LaunchConfiguration('auto_start_servo'),
            'tool_forward_controller':  LaunchConfiguration('tool_forward_controller'),
            'tool_original_controller': LaunchConfiguration('tool_original_controller'),
            'publish_hz': 50.0,
        }],
    )

    return LaunchDescription(args + [keyboard_servo_node])
