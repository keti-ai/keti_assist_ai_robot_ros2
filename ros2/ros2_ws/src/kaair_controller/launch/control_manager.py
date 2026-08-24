"""
control_manager.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
arm/body Controller Manager 를 만드는 재사용 가능한 launch 모듈.

kaair_moveit_config, kaair_bringup 쪽 여러 launch 파일에 동일하게 복붙되어
있던 "arm_rsp_node/body_rsp_node + arm/body ros2_control_node + 스포너들 +
controller_mode_switcher" 블록을 하나로 모은 것이다.

호출자(kaair_controller/launch/robot_control.launch.py,
kaair_moveit_config/launch/kaair_moveit.launch.py 등)는 전체바디용
robot_state_publisher, static TF, RViz, move_group 등 자신만의 구성 요소를
별도로 갖고, 이 모듈이 반환하는 merger_node 를 자신의 robot_state_publisher
OnProcessStart 에, controllers_ready_action 을 move_group 등의 시작 트리거로
체이닝하면 된다.

IncludeLaunchDescription 대신 순수 함수로 만든 이유: 호출자가 내부 Node
객체(merger_node, controllers_ready_action)를 직접 참조해 자신의 이벤트체인에
엮어야 하는데, IncludeLaunchDescription 은 그 참조를 돌려주지 않는다.
kaair_bringup/launch/robot_spec_utils.py 와 동일한 "순수 함수 모듈 + sys.path
주입으로 타 패키지에서 import" 패턴을 따른다.
"""

import os

import yaml
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def build_control_manager(
    *,
    use_fake_str,
    hw_spec_file,
    initial_positions_file,
    ctrl_pkg,
    moveit_pkg,
    include_arm=True,
    include_body=True,
    include_lift=True,
    include_head=True,
    include_tool=True,
    arm_cm_prefix=None,
    spawn_controller_mode_switcher=True,
):
    """arm/body controller manager 구성 요소를 만들어 반환한다.

    반환 dict:
      always_on_actions          : 그대로 LaunchDescription 에 append 가능한
                                    액션 목록 (rsp_node, cm_node, 각 spawner
                                    이벤트체인, controller_mode_switcher
                                    이벤트체인 포함).
      merger_node                : joint_state_publisher(merger). 호출자가
                                    자신의 (전체바디) robot_state_publisher 의
                                    OnProcessStart 에 직접 얹어야 한다
                                    (always_on_actions 에는 포함되지 않음 —
                                    시작 시점을 호출자가 결정해야 하므로).
      controllers_ready_action   : OnProcessExit 체이닝용 트리거 노드.
                                    body 포함 시 tool_spawner, 아니면 arm 쪽
                                    xarm7_traj_controller 스포너, 둘 다 없으면
                                    None.
      arm_cm_node, body_cm_node  : Node | None.
    """
    actions = []

    def make_description(xacro_path, extra=''):
        cmd = (
            f'xacro {xacro_path}'
            f' use_fake_hardware:={use_fake_str}'
            f' hw_spec_file:={hw_spec_file}'
        )
        if extra:
            cmd += ' ' + extra
        return {'robot_description': ParameterValue(Command(cmd), value_type=str)}

    # ros2_control_node 가 robot_description 을 구독하는 토픽 경로가
    # 배포판마다 다르다 (실제 로그로 확인됨):
    #   Humble (ros2_control 2.x) : '~/robot_description'
    #     → private 네임스페이스, 즉 '/arm/controller_manager/robot_description'
    #   Jazzy  (ros2_control 4.x) : 'robot_description' (네임스페이스 상대)
    #     → '/arm/robot_description'
    # robot_state_publisher 의 기본 발행 토픽('<ns>/robot_description')은
    # Jazzy 쪽 규칙과만 일치하므로, Humble 에서는 명시적으로 remap 한다.
    is_humble = os.environ.get('ROS_DISTRO') == 'humble'

    arm_cm_node = None
    body_cm_node = None
    arm_ctrl_spawner = None
    tool_spawner = None

    # ════════════════════════════════════════════════════════════════════
    # arm Controller Manager (namespace: /arm)
    # ════════════════════════════════════════════════════════════════════
    if include_arm:
        arm_hw_xacro = os.path.join(moveit_pkg, 'config', 'arm_hw.urdf.xacro')
        arm_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'arm_controllers.yaml')
        arm_description = make_description(
            arm_hw_xacro, f'initial_positions_file:={initial_positions_file}',
        )
        arm_rd_topic = (
            '/arm/controller_manager/robot_description' if is_humble
            else '/arm/robot_description'
        )

        # robot_description 은 arm_rsp_node 가 위 토픽으로 발행하므로
        # controller_manager 에는 컨트롤러 yaml 만 전달한다.
        #
        # arm_hw.urdf.xacro 는 arm CM 전용 stub 트리(arm_hw_base 루트, 실제
        # 로봇 트리와 link1~link7 이름이 겹침)라서, tf2_ros 브로드캐스터가
        # 네임스페이스와 무관하게 항상 절대 경로 '/tf', '/tf_static' 에 쏘는
        # 특성상 remap 없이는 전체바디 robot_state_publisher 가 발행하는
        # 진짜 TF 와 같은 frame_id 를 두고 충돌해 (arm_hw_base 는 부모가
        # 없으므로) RViz 에 "No transform ... to [slamware_map]" 경고가
        # 뜬다. 이 노드는 robot_description 토픽 발행 용도일 뿐 TF 를 쓰지
        # 않으므로 자신의 네임스페이스 하위로 격리한다.
        arm_rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='arm',
            output='both',
            parameters=[arm_description],
            remappings=[
                ('robot_description', arm_rd_topic),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )

        arm_cm_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm',
            parameters=[arm_ctrl_yaml],
            prefix=arm_cm_prefix or [],
            output='both',
        )

        arm_jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/arm/controller_manager',
            ],
        )
        # fake/real 모두 xarm7_traj_controller 사용
        arm_ctrl_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'xarm7_traj_controller',
                '--controller-manager', '/arm/controller_manager',
            ],
        )
        xarm7_fwd_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'xarm7_forward_controller',
                '--controller-manager', '/arm/controller_manager',
                '--inactive',
            ],
        )

        actions += [
            arm_rsp_node,
            arm_cm_node,
            RegisterEventHandler(OnProcessStart(
                target_action=arm_cm_node,
                on_start=[arm_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=arm_jsb_spawner,
                # xarm7_traj_controller(ACTIVE) + xarm7_forward_controller(INACTIVE) 동시 스폰
                on_start=[arm_ctrl_spawner, xarm7_fwd_spawner],
            )),
        ]

    # ════════════════════════════════════════════════════════════════════
    # body Controller Manager (namespace: /body)
    # ════════════════════════════════════════════════════════════════════
    if include_body:
        body_hw_xacro = os.path.join(moveit_pkg, 'config', 'body_hw.urdf.xacro')
        body_ctrl_yaml = os.path.join(ctrl_pkg, 'config', 'body_controllers.yaml')
        body_description = make_description(
            body_hw_xacro, f'initial_positions_file:={initial_positions_file}',
        )
        body_rd_topic = (
            '/body/controller_manager/robot_description' if is_humble
            else '/body/robot_description'
        )

        # arm_rsp_node 와 동일한 이유(body_hw.urdf.xacro 의 body_hw_base
        # stub 트리가 전체바디 TF 와 frame_id 충돌)로 tf/tf_static 을
        # 네임스페이스 하위로 격리한다.
        body_rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='body',
            output='both',
            parameters=[body_description],
            remappings=[
                ('robot_description', body_rd_topic),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        )

        body_cm_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='body',
            parameters=[body_ctrl_yaml],
            output='both',
        )

        body_jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/body/controller_manager',
            ],
        )

        # ── body 정규(ACTIVE) + Forward(INACTIVE) 컨트롤러 ─────────────────
        # Forward 컨트롤러는 기동 시 configured 상태로만 올라온다.
        # 전환은 controller_mode_switcher 서비스로 한다.
        body_start_spawners = []
        if include_lift:
            body_start_spawners += [
                Node(
                    package='controller_manager', executable='spawner',
                    arguments=['lift_controller',
                               '--controller-manager', '/body/controller_manager'],
                ),
                Node(
                    package='controller_manager', executable='spawner',
                    arguments=['lift_forward_controller',
                               '--controller-manager', '/body/controller_manager',
                               '--inactive'],
                ),
            ]
        if include_head:
            body_start_spawners += [
                Node(
                    package='controller_manager', executable='spawner',
                    arguments=['head_controller',
                               '--controller-manager', '/body/controller_manager'],
                ),
                Node(
                    package='controller_manager', executable='spawner',
                    arguments=['head_forward_controller',
                               '--controller-manager', '/body/controller_manager',
                               '--inactive'],
                ),
            ]
        if include_tool:
            tool_spawner = Node(
                package='controller_manager', executable='spawner',
                arguments=['tool_controller',
                           '--controller-manager', '/body/controller_manager'],
            )
            tool_fwd_spawner = Node(
                package='controller_manager', executable='spawner',
                arguments=['tool_forward_controller',
                           '--controller-manager', '/body/controller_manager',
                           '--inactive'],
            )
            body_start_spawners += [tool_spawner, tool_fwd_spawner]

        actions += [
            body_rsp_node,
            body_cm_node,
            RegisterEventHandler(OnProcessStart(
                target_action=body_cm_node,
                on_start=[body_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=body_jsb_spawner,
                on_start=body_start_spawners,
            )),
        ]

    # ════════════════════════════════════════════════════════════════════
    # 준비 완료 트리거 + controller_mode_switcher
    # ════════════════════════════════════════════════════════════════════
    # body(USB)가 arm(네트워크)보다 느리게 준비되므로 tool_spawner exit 을
    # "전체 HW 준비 완료"의 대표 트리거로 쓴다(기존 kaair_moveit.launch.py와
    # 동일한 휴리스틱). body 를 안 쓰는 호출자는 arm 쪽을 대신 사용한다.
    controllers_ready_action = (
        tool_spawner if tool_spawner is not None else arm_ctrl_spawner
    )

    if (
        spawn_controller_mode_switcher
        and include_arm and include_body
        and controllers_ready_action is not None
    ):
        ctrl_mode_switcher_node = Node(
            package='kaair_bringup',
            executable='controller_mode_switcher',
            name='controller_mode_switcher',
            output='screen',
        )
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=controllers_ready_action,
            on_exit=[ctrl_mode_switcher_node],
        )))

    # ── joint_state_publisher (merger) ─────────────────────────────────
    # /arm/joint_states + /body/joint_states → /joint_states
    # 시작 시점은 호출자의 (전체바디) robot_state_publisher 에 달려있으므로
    # always_on_actions 에 넣지 않고 별도로 반환한다.
    merger_source_list = []
    if include_arm:
        merger_source_list.append('/arm/joint_states')
    if include_body:
        merger_source_list.append('/body/joint_states')

    with open(initial_positions_file, 'r') as _f:
        _initial_pos = yaml.safe_load(_f).get('initial_positions', {})

    merger_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': merger_source_list,
            'rate': 50,
            'initial_positions': _initial_pos,
        }],
        remappings=[('robot_description', '/robot_description')],
    )

    return {
        'always_on_actions': actions,
        'merger_node': merger_node,
        'controllers_ready_action': controllers_ready_action,
        'arm_cm_node': arm_cm_node,
        'body_cm_node': body_cm_node,
    }
