"""
master_bringup.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
조이스틱(joy_node) + 8축 다이나믹셀 마스터 장치(master_dxl_hw) +
master_controller 를 기동한다. master_controller 는 이제 arm(joint1~7)을
MoveIt Servo 의 delta_joint_cmds(JointJog)로 추종시킨다.

  ┌─────────────────────────────────────────────────────────────────────┐
  │  joy_node          조이스틱 입력                                    │
  │  master_dxl_hw     8축 Dynamixel 마스터 장치(/master/joint_states)  │
  │  master_controller                                                   │
  │    - lift/head: forward controller 로 직접 제어(기존과 동일)        │
  │    - tool(gripper): forward controller 로 직접 제어(기존과 동일)    │
  │    - arm(joint1~7): servo_server 의 /servo_server/delta_joint_cmds  │
  │        (JointJog) 로 마스터 절대 조인트 값을 추종한다.               │
  │        teleop 진입/종료 시 /servo_server/start_servo,               │
  │        /servo_server/pause_servo (Trigger) 를 호출한다.             │
  └─────────────────────────────────────────────────────────────────────┘

⚠ 사전 조건: 이 launch 파일은 arm/body controller_manager + move_group +
  MoveIt Servo 가 이미 실행 중이어야 한다(예: real_slamtec_bringup.launch.py,
  kaair_moveit_bringup.launch.py 등 servo_module 을 포함한 bringup).
  이 파일 자체는 마스터 장치 쪽만 담당하며 MoveIt/컨트롤러 매니저를
  기동하지 않는다.

Launch 인자
  joy_dev / joy_deadzone / joy_autorepeat_rate : joy_node 파라미터
  arm_servo_kp             : P 제어 게인 (rad/s per rad of error, default 4.0)
  arm_servo_max_vel_rad_s  : 속도 클램프 (안전 상한, default 1.0)
  arm_servo_deadband_rad   : 이 이하 오차는 0 속도 (default 0.001)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='joystick device path',
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.05',
        description='joy deadzone',
    )
    joy_autorepeat_arg = DeclareLaunchArgument(
        'joy_autorepeat_rate',
        default_value='50.0',
        description='joy autorepeat rate (Hz)',
    )
    arm_servo_kp_arg = DeclareLaunchArgument(
        'arm_servo_kp',
        default_value='4.0',
        description='arm servo P 제어 게인 (rad/s per rad of error)',
    )
    arm_servo_max_vel_arg = DeclareLaunchArgument(
        'arm_servo_max_vel_rad_s',
        default_value='1.0',
        description='arm servo 속도 클램프 (안전 상한, rad/s)',
    )
    arm_servo_deadband_arg = DeclareLaunchArgument(
        'arm_servo_deadband_rad',
        default_value='0.001',
        description='arm servo 데드밴드 (이 이하 오차는 0 속도, rad)',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': LaunchConfiguration('joy_autorepeat_rate'),
        }],
    )

    master_controller_node = Node(
        package='kaair_bringup',
        executable='master_controller',
        name='master_controller',
        output='screen',
        parameters=[{
            'arm_servo_kp': LaunchConfiguration('arm_servo_kp'),
            'arm_servo_max_vel_rad_s': LaunchConfiguration('arm_servo_max_vel_rad_s'),
            'arm_servo_deadband_rad': LaunchConfiguration('arm_servo_deadband_rad'),
        }],
    )

    master_dxl_hw_node = Node(
        package='kaair_bringup',
        executable='master_dxl_hw',
        name='master_dxl_hw',
        output='screen',
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_deadzone_arg,
        joy_autorepeat_arg,
        arm_servo_kp_arg,
        arm_servo_max_vel_arg,
        arm_servo_deadband_arg,
        joy_node,
        master_controller_node,
        master_dxl_hw_node,
    ])
