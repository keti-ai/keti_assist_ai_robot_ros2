"""
servo_module.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MoveIt Servo 를 기존 move_group + 2-CM 환경에 추가하는 재사용 가능한
launch 모듈.

kaair_moveit_config/launch/kaair_moveit_servo.launch.py 와
kaair_bringup/launch/kaair_moveit_bringup.launch.py 에 동일하게 복붙되어
있던 servo_node_main 노드 + "move_group 기동 후 servo 시작" 이벤트체인을
하나로 모은 것이다.

  servo_server (standalone servo_node_main)
    입력:  /servo_server/delta_twist_cmds  (TwistStamped)
           /servo_server/delta_joint_cmds  (JointJog)
    출력:  /arm/xarm7_traj_controller/joint_trajectory
    서비스: /servo_server/start_servo   (Trigger)
            /servo_server/pause_servo   (Trigger)
            /servo_server/unpause_servo (Trigger)
            /servo_server/stop_servo    (Trigger)

kaair_controller/launch/control_manager.py 와 동일하게 IncludeLaunchDescription
대신 순수 함수로 만든 이유: 호출자가 이미 가지고 있는 move_group_node 액션
객체에 이벤트를 체이닝해야 하므로, 함수 반환값으로 그대로 servo_node 참조를
넘겨받아야 한다.
"""

from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

import os
import yaml


def _load_yaml(package_name: str, relative_path: str) -> dict:
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, relative_path), 'r') as f:
        return yaml.safe_load(f)


def build_moveit_servo(*, moveit_config, move_group_node):
    """MoveIt Servo 노드 + move_group 시작 후 기동하는 이벤트체인을 반환한다.

    반환 dict:
      actions    : LaunchDescription 에 그대로 append 가능한 액션 목록
                   (servo_node 자체는 move_group_node 의 OnProcessStart 에서
                   기동되므로 이 목록에는 이벤트핸들러만 들어있다).
      servo_node : Node.
    """
    servo_yaml = _load_yaml('kaair_moveit_config', 'config/kaair_servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # servo_node_main (standalone)
    #   ComposableNodeContainer 대신 독립 프로세스로 실행.
    #   move_group 기동 후 시작해 planning scene monitor 를 secondary 로 attach.
    #   is_primary_planning_scene_monitor: false (servo config 에 설정됨).
    #
    #   moveit_servo 실행 파일 이름이 배포판마다 다르다(실제 설치 확인됨):
    #     Humble (moveit_servo 0.x) : servo_node_main
    #     Jazzy  (moveit_servo 2.12.x) : servo_node
    servo_executable = (
        'servo_node' if os.environ.get('ROS_DISTRO') == 'jazzy' else 'servo_node_main'
    )
    servo_node = Node(
        package='moveit_servo',
        executable=servo_executable,
        name='servo_server',
        output='screen',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return {
        'actions': [
            RegisterEventHandler(OnProcessStart(
                target_action=move_group_node,
                on_start=[servo_node],
            )),
        ],
        'servo_node': servo_node,
    }
