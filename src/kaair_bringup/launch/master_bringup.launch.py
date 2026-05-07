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
        joy_node,
        master_controller_node,
        master_dxl_hw_node,
    ])
