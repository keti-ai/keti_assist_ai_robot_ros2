from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(package="kaair_bringup", executable="xarm_bridge", name="xarm_bridge", output="screen"),
        Node(package="kaair_bringup", executable="location_server", name="location_server", output="screen"),
    ])  