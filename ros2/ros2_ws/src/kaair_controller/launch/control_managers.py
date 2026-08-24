"""
control_managers.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
kaair 로봇의 arm/body 2-Controller-Manager 구성을 만드는 공용 빌더.

전원을 처음 켰을 때의 표준 기동 순서 (body 에 lift_controller 가 포함되고
실기체(use_fake_hardware=false) 인 경우):

  1) lift_initializer 노드가 가장 먼저(단독으로) 실행된다. 이 단계만 직렬이다.
     - 이미 영점(homing)이 잡혀 있으면 즉시 종료하고 재호밍을 생략한다.
     - 잡혀 있지 않으면 호밍 시퀀스를 수행한 뒤 영점을 기록하고 종료한다.
       (호밍에는 시간이 걸리므로, 이 단계가 끝나기 전에 body controller_manager
        를 올리면 lift 하드웨어 인터페이스가 "영점 미확인" 에러로 활성화에
        실패한다 — 그래서 이 단계만큼은 반드시 먼저 끝나야 한다.)
  2) lift_initializer 가 종료(exit)된 뒤에는 arm_controller_manager 와
     body_controller_manager 가 동시에(병렬로) 기동된다. 각 CM 내부의
     스포너들(joint_state_broadcaster → 활성/포워드 컨트롤러)도 서로 병렬로
     기동된다 — lift 초기화 이후에는 굳이 순서를 강제할 이유가 없기 때문에
     불필요한 대기를 만들지 않는다.

fake hardware 이거나 body 에 lift_controller 가 없는 구성에서는 1) 단계의
게이팅이 아예 적용되지 않고 arm/body CM 이 곧바로(동시에) 기동된다.

kaair_controller/launch/robot_control.launch.py 와 kaair_moveit_config,
kaair_bringup 의 각 bringup launch 파일이 모두 이 빌더를 통해 동일한
게이팅 로직을 공유한다 (더 이상 각 launch 파일에 CM 노드를 복붙하지 않는다).
"""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def build_control_managers(
    *,
    arm_description=None,
    body_description=None,
    arm_ctrl_yaml=None,
    body_ctrl_yaml=None,
    use_fake_hardware: bool,
    include_arm: bool = True,
    include_body: bool = True,
    body_active_controllers=('lift_controller', 'head_controller', 'tool_controller'),
    body_forward_controllers=None,
    arm_forward_controller=None,
    arm_cm_prefix=None,
    arm_cm_output='both',
    body_cm_output='both',
    gate_lift_init: bool = True,
):
    """arm/body controller_manager + spawner 액션들과, 이벤트 체이닝에 필요한
    노드 핸들을 함께 반환한다.

    Returns
    -------
    (actions, handles)
      actions: LaunchDescription 에 그대로 추가할 수 있는 액션 리스트
      handles: 호출부에서 추가 이벤트(예: move_group 기동)를 체이닝할 때
               쓰는 노드 딕셔너리
                 arm_cm_node, arm_jsb_spawner, arm_ctrl_spawner, arm_fwd_spawner
                 body_cm_node, body_jsb_spawner,
                 body_active_spawners (dict: controller 이름 → Node),
                 body_fwd_spawners    (dict: controller 이름 → Node),
                 lift_initializer_node (게이팅이 적용된 경우에만 값 존재, 아니면 None)
    """
    actions = []
    handles = {
        'arm_cm_node': None,
        'arm_jsb_spawner': None,
        'arm_ctrl_spawner': None,
        'arm_fwd_spawner': None,
        'body_cm_node': None,
        'body_jsb_spawner': None,
        'body_active_spawners': {},
        'body_fwd_spawners': {},
        'lift_initializer_node': None,
    }

    # 실기체 + lift_controller 사용 시에만 lift_initializer 로 게이팅한다.
    # (fake hardware 는 호밍이 필요 없고, lift 를 쓰지 않는 구성도 게이팅 불필요)
    needs_lift_init = (
        gate_lift_init
        and not use_fake_hardware
        and include_body
        and 'lift_controller' in body_active_controllers
    )

    lift_initializer_node = None
    if needs_lift_init:
        lift_initializer_node = Node(
            package='kaair_driver',
            executable='lift_initializer',
            name='lift_initializer',
            output='screen',
        )
        actions.append(lift_initializer_node)
        handles['lift_initializer_node'] = lift_initializer_node

    # lift_initializer 종료 후에 뒤따라 기동돼야 하는 controller_manager 들
    # (게이팅이 필요 없으면 그냥 즉시 시작 목록으로 쓰인다). arm/body CM 은
    # 서로 순서를 기다리지 않고 동시에 시작된다.
    cm_start_actions = []

    # ════════════════════════════════════════════════════════════════════
    # arm Controller Manager (namespace: /arm)
    # ════════════════════════════════════════════════════════════════════
    if include_arm:
        arm_cm_kwargs = dict(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='arm',
            parameters=[arm_description, arm_ctrl_yaml],
            output=arm_cm_output,
        )
        if arm_cm_prefix:
            arm_cm_kwargs['prefix'] = arm_cm_prefix
        arm_cm_node = Node(**arm_cm_kwargs)

        arm_jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/arm/controller_manager',
            ],
        )
        arm_ctrl_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'xarm7_traj_controller',
                '--controller-manager', '/arm/controller_manager',
            ],
        )

        # ctrl_spawner 와 (있다면) fwd_spawner 는 jsb 시작과 동시에 병렬로 스폰
        arm_after_jsb = [arm_ctrl_spawner]
        arm_fwd_spawner = None
        if arm_forward_controller:
            arm_fwd_spawner = Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    arm_forward_controller,
                    '--controller-manager', '/arm/controller_manager',
                    '--inactive',
                ],
            )
            arm_after_jsb.append(arm_fwd_spawner)

        cm_start_actions.append(arm_cm_node)
        actions += [
            RegisterEventHandler(OnProcessStart(
                target_action=arm_cm_node,
                on_start=[arm_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=arm_jsb_spawner,
                on_start=arm_after_jsb,
            )),
        ]

        handles.update(
            arm_cm_node=arm_cm_node,
            arm_jsb_spawner=arm_jsb_spawner,
            arm_ctrl_spawner=arm_ctrl_spawner,
            arm_fwd_spawner=arm_fwd_spawner,
        )

    # ════════════════════════════════════════════════════════════════════
    # body Controller Manager (namespace: /body)
    # ════════════════════════════════════════════════════════════════════
    if include_body:
        body_cm_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='body',
            parameters=[body_description, body_ctrl_yaml],
            output=body_cm_output,
        )

        body_jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/body/controller_manager',
            ],
        )

        body_active_spawners = {
            ctrl_name: Node(
                package='controller_manager',
                executable='spawner',
                arguments=[ctrl_name, '--controller-manager', '/body/controller_manager'],
            )
            for ctrl_name in body_active_controllers
        }

        body_fwd_spawners = {
            ctrl_name: Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    ctrl_name,
                    '--controller-manager', '/body/controller_manager',
                    '--inactive',
                ],
            )
            for ctrl_name in (body_forward_controllers or [])
        }

        # 정규(ACTIVE) + Forward(INACTIVE) 스포너들은 jsb 시작과 동시에 병렬로 스폰
        body_after_jsb = list(body_active_spawners.values()) + list(body_fwd_spawners.values())

        cm_start_actions.append(body_cm_node)
        actions += [
            RegisterEventHandler(OnProcessStart(
                target_action=body_cm_node,
                on_start=[body_jsb_spawner],
            )),
            RegisterEventHandler(OnProcessStart(
                target_action=body_jsb_spawner,
                on_start=body_after_jsb,
            )),
        ]

        handles.update(
            body_cm_node=body_cm_node,
            body_jsb_spawner=body_jsb_spawner,
            body_active_spawners=body_active_spawners,
            body_fwd_spawners=body_fwd_spawners,
        )

    # ════════════════════════════════════════════════════════════════════
    # lift_initializer → (완료 후) arm/body controller_manager 동시 기동
    # ════════════════════════════════════════════════════════════════════
    if needs_lift_init:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=lift_initializer_node,
            on_exit=cm_start_actions,
        )))
    else:
        actions += cm_start_actions

    return actions, handles
