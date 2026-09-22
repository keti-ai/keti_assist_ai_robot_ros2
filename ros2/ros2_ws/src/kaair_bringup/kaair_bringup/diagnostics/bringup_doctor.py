#!/usr/bin/env python3
"""bringup_doctor — kaair_bringup 실행 후 전체 시스템 점검 노드.

    ros2 run kaair_bringup bringup_doctor
    ros2 run kaair_bringup bringup_doctor --ros-args -p use_azure:=true -p watch:=true

real_clobot_bringup / real_slamtec_bringup 이 띄우는 것들이 실제로 살아 있는지를
ROS 그래프 + 실데이터 수신으로 확인한다.

  1. config   : 스펙 YAML 로드, arm.robot_ip / mobile_bridge.type
  2. network  : xArm 컨트롤러(TCP 30001), Clobot rosbridge(TCP robot_port)
  3. nodes    : 필수 노드 존재 / 같은 이름의 중복 노드
  4. ctrl     : /arm, /body controller_manager 의 컨트롤러 상태(active)
  5. topics   : 주요 토픽의 publisher 유무 + 실제 수신 Hz
  6. joints   : /joint_states 에 기대 조인트가 있고 값이 유한한지, xArm err/warn
  7. tf       : 핵심 TF 체인 lookup
  8. actions  : 액션 서버 (kaair_worker/*, controller FJT, move_group, nav)
  9. services : 서비스 (servo, xarm_bridge, marker, ...)

결과 코드: 0=전부 정상(WARN 포함), 1=FAIL 있음, 2=--strict(strict:=true)에서 WARN 있음.

이 파일은 다른 노드처럼 스크립트로 직접 설치되므로(CMake PYTHON_NODES) 단독 실행 가능해야
한다 — 패키지 내부 모듈을 import 하지 않는다.
"""

import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message, get_service
from tf2_ros import Buffer, TransformListener

PASS, WARN, FAIL, SKIP = 'PASS', 'WARN', 'FAIL', 'SKIP'
_COLOR = {PASS: '\033[32m', WARN: '\033[33m', FAIL: '\033[31m', SKIP: '\033[90m'}
_RESET = '\033[0m'

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
_LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

ARM_JOINTS = [f'joint{i}' for i in range(1, 8)]
BODY_JOINTS = ['lift_joint', 'head_joint1', 'head_joint2']

# name, ns 는 실제 launch 의 name=/namespace= 값 기준 (실행 파일명이 아니다).
# (fq_name, 설명)
_CORE_NODES = [
    ('/robot_state_publisher', 'URDF → TF'),
    ('/joint_state_merger', '/arm+/body joint_states 병합'),
    ('/arm/controller_manager', 'arm ros2_control'),
    ('/body/controller_manager', 'body ros2_control (lift/head/tool)'),
    ('/move_group', 'MoveIt move_group'),
    ('/servo_server', 'MoveIt Servo'),
    ('/xarm_bridge', '/arm/init_set'),
    ('/eef_state_publisher', 'EEF pose/twist'),
    ('/arm_move_action_server', 'kaair_worker/arm_move*'),
    ('/lift_move_action_server', 'kaair_worker/lift_move'),
    ('/head_move_server', 'kaair_worker/head_move'),
    ('/controller_mode_switcher', 'body controller 전환'),
    ('/object_marker_server', 'object marker 서비스'),
    ('/robot_pose_publisher', '/robot_pose/*'),
    ('/servo_status_overlay', 'RViz servo 상태 텍스트'),
]

# 컨트롤러: 반드시 active / 로드만 되어 있으면 되는 것 (forward 는 전환용이라 inactive 가 정상)
_CONTROLLERS = {
    '/arm/controller_manager': {
        'active': ['joint_state_broadcaster', 'xarm7_traj_controller'],
        'loaded': ['xarm7_forward_controller'],
    },
    '/body/controller_manager': {
        'active': ['joint_state_broadcaster', 'lift_controller',
                   'head_controller', 'tool_controller'],
        'loaded': ['lift_forward_controller', 'head_forward_controller',
                   'tool_forward_controller'],
    },
}

_ACTIONS = [
    ('kaair_worker/arm_moveJ', 'arm_move_action_server'),
    ('kaair_worker/arm_moveL', 'arm_move_action_server'),
    ('kaair_worker/arm_moveT', 'arm_move_action_server'),
    ('kaair_worker/arm_task', 'arm_move_action_server'),
    ('kaair_worker/lift_move', 'lift_move_action_server'),
    ('kaair_worker/head_move', 'head_move_server'),
    ('/arm/xarm7_traj_controller/follow_joint_trajectory', 'arm controller'),
    ('/body/lift_controller/follow_joint_trajectory', 'lift controller'),
    ('/body/head_controller/follow_joint_trajectory', 'head controller'),
    ('/body/tool_controller/gripper_cmd', 'tool controller'),
    ('/move_action', 'move_group'),
    ('/execute_trajectory', 'move_group'),
]

_SERVICES = [
    ('/arm/init_set', 'xarm_bridge'),
    ('/servo_server/start_servo', 'servo_server'),
    ('/servo_server/pause_servo', 'servo_server'),
    ('/compute_cartesian_path', 'move_group'),
    ('/create_object_marker', 'object_marker_server'),
    ('/clear_object_markers', 'object_marker_server'),
    ('/controller_mode_switcher/switch_mode', 'controller_mode_switcher'),
]


class Probe:
    """토픽 하나에 대한 수신 통계."""

    def __init__(self, topic, min_hz, group, latched=False, keep_msg=False,
                 required=True):
        self.topic = topic
        self.min_hz = min_hz
        self.group = group
        self.latched = latched
        self.keep_msg = keep_msg
        self.required = required
        self.stamps = deque(maxlen=2000)
        self.count = 0
        self.last_msg = None
        self.sub = None
        self.error = None


class BringupDoctor(Node):
    def __init__(self):
        super().__init__('bringup_doctor')

        p = self.declare_parameter
        p('spec', 'kaair_specs_01.yaml')
        p('use_fake_hardware', False)
        p('use_head_camera', True)
        p('use_hand_camera', True)
        p('use_azure', False)
        p('use_xarm_driver', True)
        p('robot_host', '192.168.0.104')   # real_clobot_bringup 기본값
        p('robot_port', 9090)
        p('xarm_hw_ns', 'xarm')
        p('map_frame', 'slamware_map')
        p('base_frame', 'base_footprint')
        p('arm_base_frame', 'arm_base')
        p('tool_frame', 'tool_tcp_link')
        p('discovery_sec', 2.0)   # 그래프 발견 대기
        p('sample_sec', 4.0)      # 토픽 Hz 측정 창
        p('watch', False)         # true 면 주기적으로 반복 점검
        p('watch_period_sec', 5.0)
        p('strict', False)        # true 면 WARN 도 실패(종료코드 2)
        p('json_out', '')         # 결과를 JSON 으로 저장할 경로
        p('sections', '')         # 쉼표 구분 부분 실행, 예: nodes,tf

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.g = g
        self.spec_name = g('spec')
        self.fake = bool(g('use_fake_hardware'))
        loaded, self.spec_path = self._load_spec(self.spec_name)
        self.spec_ok = loaded is not None
        self.spec = loaded or {}
        mb = (self.spec.get('mobile_bridge') or {})
        self.bridge_type = str(mb.get('type') or 'slamtec').strip().strip('"\'').lower()
        self.is_clobot = self.bridge_type in ('clobot', 'clober')
        self.robot_ip = str((self.spec.get('arm') or {}).get('robot_ip') or '').strip()

        self.driver_ns = '/' + str(g('xarm_hw_ns')).strip('/')
        self.probes = []
        self._build_probes()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    # ── 설정/프로브 구성 ───────────────────────────────────────────────
    @staticmethod
    def _load_spec(name):
        try:
            path = os.path.join(get_package_share_directory('kaair_bringup'),
                                'config', 'robots', name)
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            return (data if isinstance(data, dict) else {}), path
        except Exception:  # noqa: BLE001 - 리포트에서 FAIL 로 표시
            return None, name

    def _build_probes(self):
        g = self.g
        P = self.probes
        P.append(Probe('/joint_states', 20.0, 'joints', keep_msg=True))
        P.append(Probe('/arm/joint_states', 20.0, 'joints'))
        P.append(Probe('/body/joint_states', 20.0, 'joints'))
        if g('use_xarm_driver'):
            P.append(Probe(f'{self.driver_ns}/joint_states', 5.0, 'joints'))
            P.append(Probe(f'{self.driver_ns}/xarm_states', 1.0, 'joints',
                           keep_msg=True, required=False))
        P.append(Probe('/observation/eef_pose', 5.0, 'joints'))
        P.append(Probe('/robot_pose/mobile_pose', 5.0, 'joints'))
        P.append(Probe('/scan', 1.0, 'mobile'))
        P.append(Probe('/map', 0.0, 'mobile', latched=True))
        if g('use_head_camera') or g('use_azure'):
            P.append(Probe('/femto/color/image_raw', 5.0, 'camera'))
            P.append(Probe('/femto/depth/aligned', 5.0, 'camera'))
            P.append(Probe('/femto/depth/camera_info', 5.0, 'camera'))
        if g('use_hand_camera'):
            P.append(Probe('/hand/camera/color/image_raw', 5.0, 'camera'))
            P.append(Probe('/hand/camera/depth/image_rect_raw', 5.0, 'camera'))
            P.append(Probe('/hand/camera/depth/camera_info', 5.0, 'camera'))

    def attach_probes(self):
        """그래프에서 타입을 알아낸 뒤 구독을 만든다 (타입 하드코딩 회피)."""
        types = dict(self.get_topic_names_and_types())
        for pr in self.probes:
            if pr.sub is not None:
                continue
            tnames = types.get(pr.topic)
            if not tnames:
                continue  # publisher 가 아직 없음 — 평가 단계에서 FAIL/WARN 처리
            try:
                msg_type = get_message(tnames[0])
                qos = _LATCHED_QOS if pr.latched else _SENSOR_QOS
                # raw=True: 이미지처럼 큰 메시지를 역직렬화하지 않아 CPU/메모리 부담이 없다.
                raw = not pr.keep_msg
                pr.sub = self.create_subscription(
                    msg_type, pr.topic, self._make_cb(pr), qos, raw=raw)
            except Exception as e:  # noqa: BLE001
                pr.error = f'구독 실패: {e}'

    def _make_cb(self, pr):
        def cb(msg):
            pr.stamps.append(time.monotonic())
            pr.count += 1
            if pr.keep_msg:
                pr.last_msg = msg
        return cb

    # ── 결과 헬퍼 ─────────────────────────────────────────────────────
    @staticmethod
    def _r(section, name, status, detail=''):
        return {'section': section, 'name': name, 'status': status, 'detail': detail}

    def _enabled(self, section):
        wanted = [s.strip() for s in str(self.g('sections')).split(',') if s.strip()]
        return not wanted or section in wanted

    # ── 개별 점검 ─────────────────────────────────────────────────────
    def check_config(self):
        s = 'config'
        if not self.spec_ok:
            return [self._r(s, f'spec {self.spec_name}', FAIL,
                            f'스펙 파일을 읽을 수 없음: {self.spec_path}')]
        out = [self._r(s, f'spec {self.spec_name}', PASS,
                       f'mobile_bridge.type={self.bridge_type}')]
        if self.g('use_xarm_driver') and not self.robot_ip:
            out.append(self._r(s, 'arm.robot_ip', FAIL,
                               '스펙에 없음 → ufactory_driver 가 뜨지 못한다'))
        else:
            out.append(self._r(s, 'arm.robot_ip', PASS, self.robot_ip or '(driver 미사용)'))
        if self.g('use_azure') and self.g('use_head_camera'):
            out.append(self._r(s, 'camera 옵션', WARN,
                               'use_azure 와 use_head_camera 가 동시에 true — 같은 카메라를 두 번 열 수 있음'))
        return out

    @staticmethod
    def _tcp(host, port, timeout=1.5):
        try:
            with socket.create_connection((host, int(port)), timeout=timeout):
                return True, ''
        except OSError as e:
            return False, str(e)

    def check_network(self):
        s = 'network'
        out = []
        if self.fake:
            return [self._r(s, 'arm 컨트롤러', SKIP, 'use_fake_hardware')]
        if self.g('use_xarm_driver') and self.robot_ip:
            ok, err = self._tcp(self.robot_ip, 30001)
            out.append(self._r(s, f'xArm {self.robot_ip}:30001',
                               PASS if ok else FAIL, '' if ok else err))
        if self.is_clobot:
            host, port = self.g('robot_host'), self.g('robot_port')
            ok, err = self._tcp(host, port)
            out.append(self._r(s, f'Clobot rosbridge {host}:{port}',
                               PASS if ok else FAIL, '' if ok else err))
        return out

    def _node_names(self):
        names = []
        for name, ns in self.get_node_names_and_namespaces():
            names.append(('/' + name) if ns == '/' else f'{ns}/{name}')
        return names

    def check_nodes(self):
        s = 'nodes'
        names = self._node_names()
        out = []
        expected = list(_CORE_NODES)
        if self.g('use_xarm_driver'):
            expected.append(('/ufactory_driver', 'xarm_api 드라이버'))
        expected.append(('/clobot_bridge' if self.is_clobot else '/mobile_bridge',
                         'mobile bridge'))
        if self.g('use_hand_camera'):
            expected.append(('/hand/camera', 'realsense 핸드 카메라'))
        if self.g('use_azure'):
            expected.append(('/azure_bridge_target', 'azure 도메인 브리지'))
        elif self.g('use_head_camera'):
            expected.append(('/femto/femto', 'orbbec 헤드 카메라'))
            expected.append(('/depth_resizer', 'depth 정렬'))
        for fq, desc in expected:
            n = names.count(fq)
            if n == 1:
                out.append(self._r(s, fq, PASS, desc))
            elif n == 0:
                # 카메라 노드 이름은 드라이버 버전마다 달라 접미사로 한 번 더 찾는다.
                leaf = fq.rsplit('/', 1)[-1]
                alt = [x for x in names if x.rsplit('/', 1)[-1] == leaf]
                if alt:
                    out.append(self._r(s, fq, WARN, f'{alt[0]} 로 발견됨 ({desc})'))
                else:
                    out.append(self._r(s, fq, FAIL, f'노드 없음 ({desc})'))
            else:
                out.append(self._r(s, fq, WARN, f'같은 이름 노드 {n}개 — 이전 세션 잔존 가능'))
        return out

    def _call(self, srv_type, name, request, timeout=3.0):
        cli = self.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None, '서비스 없음'
            ev, box = threading.Event(), {}
            fut = cli.call_async(request)
            fut.add_done_callback(lambda f: (box.setdefault('r', f.result()), ev.set()))
            if not ev.wait(timeout):
                return None, '응답 시간 초과'
            return box['r'], ''
        except Exception as e:  # noqa: BLE001
            return None, str(e)
        finally:
            self.destroy_client(cli)

    def check_controllers(self):
        s = 'ctrl'
        try:
            srv = get_service('controller_manager_msgs/srv/ListControllers')
        except Exception as e:  # noqa: BLE001
            return [self._r(s, 'controller_manager_msgs', WARN, f'import 실패: {e}')]
        out = []
        for cm, spec in _CONTROLLERS.items():
            resp, err = self._call(srv, f'{cm}/list_controllers', srv.Request())
            if resp is None:
                out.append(self._r(s, cm, FAIL, err))
                continue
            state = {c.name: c.state for c in resp.controller}
            for c in spec['active']:
                st = state.get(c)
                out.append(self._r(s, f'{cm.split("/")[1]}/{c}',
                                   PASS if st == 'active' else FAIL,
                                   st or '로드되지 않음'))
            for c in spec['loaded']:
                st = state.get(c)
                out.append(self._r(s, f'{cm.split("/")[1]}/{c}',
                                   PASS if st else FAIL, st or '로드되지 않음'))
        return out

    def _rate(self, pr, window):
        now = time.monotonic()
        n = sum(1 for t in pr.stamps if now - t <= window)
        return n / window

    def check_topics(self, group):
        """group: joints/mobile/camera 중 어느 프로브 묶음을 평가할지."""
        s = 'topics' if group != 'joints' else 'joints'
        window = float(self.g('sample_sec'))
        graph = dict(self.get_topic_names_and_types())
        out = []
        for pr in self.probes:
            if pr.group != group:
                continue
            bad = WARN if not pr.required else FAIL
            if pr.topic not in graph:
                out.append(self._r(s, pr.topic, bad, 'publisher 없음'))
                continue
            if pr.error:
                out.append(self._r(s, pr.topic, WARN, pr.error))
                continue
            npub = len(self.get_publishers_info_by_topic(pr.topic))
            if pr.count == 0:
                out.append(self._r(s, pr.topic, bad,
                                   f'publisher {npub}개 있으나 데이터 미수신 (정지/QoS 확인)'))
                continue
            if pr.latched:
                out.append(self._r(s, pr.topic, PASS, '수신됨(latched)'))
                continue
            hz = self._rate(pr, window)
            if hz + 1e-9 >= pr.min_hz:
                out.append(self._r(s, pr.topic, PASS, f'{hz:.1f} Hz'))
            else:
                out.append(self._r(s, pr.topic, WARN if hz > 0 else bad,
                                   f'{hz:.1f} Hz (기준 ≥ {pr.min_hz:g})'))
        return out

    def check_joint_content(self):
        s = 'joints'
        pr = next((p for p in self.probes if p.topic == '/joint_states'), None)
        out = []
        msg = pr.last_msg if pr else None
        if msg is None:
            out.append(self._r(s, '/joint_states 내용', FAIL, '수신된 메시지 없음'))
        else:
            missing = [j for j in ARM_JOINTS + BODY_JOINTS if j not in msg.name]
            nan = [n for n, v in zip(msg.name, msg.position) if not math.isfinite(v)]
            out.append(self._r(s, '/joint_states 조인트 구성',
                               FAIL if missing else PASS,
                               f'누락: {missing}' if missing else f'{len(msg.name)}개'))
            if nan:
                out.append(self._r(s, '/joint_states 값', FAIL, f'NaN/inf: {nan}'))
        xs = next((p for p in self.probes if p.topic.endswith('/xarm_states')), None)
        if xs is not None and xs.last_msg is not None:
            m = xs.last_msg
            err, warn = getattr(m, 'err', None), getattr(m, 'warn', None)
            if err:
                out.append(self._r(s, 'xArm 상태', FAIL,
                                   f'err={err} warn={warn} (/arm/init_set 로 clean 필요할 수 있음)'))
            elif warn:
                out.append(self._r(s, 'xArm 상태', WARN, f'warn={warn}'))
            else:
                out.append(self._r(s, 'xArm 상태', PASS,
                                   f'state={getattr(m, "state", "?")} mode={getattr(m, "mode", "?")} err=0'))
        return out

    def check_tf(self):
        s = 'tf'
        g = self.g
        pairs = [(g('base_frame'), g('arm_base_frame')),
                 (g('base_frame'), g('tool_frame')),
                 (g('base_frame'), 'ti_rader'),
                 (g('map_frame'), g('base_frame'))]
        out = []
        for target, source in pairs:
            label = f'{target} → {source}'
            try:
                t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
                is_static_like = t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0
                if not is_static_like and age > 2.0 and (target, source) == (g('map_frame'), g('base_frame')):
                    out.append(self._r(s, label, WARN, f'마지막 갱신 {age:.1f}s 전'))
                else:
                    out.append(self._r(s, label, PASS, ''))
            except Exception as e:  # noqa: BLE001
                out.append(self._r(s, label, FAIL, str(e).split('\n')[0][:110]))
        return out

    def _topics_with_publishers(self, topic):
        return len(self.get_publishers_info_by_topic(topic))

    def check_actions(self):
        s = 'actions'
        out = []
        graph = dict(self.get_topic_names_and_types())
        actions = list(_ACTIONS)
        if self.is_clobot:
            actions.append(('/navigate_to_pose', 'clobot_bridge'))
        for name, owner in actions:
            full = name if name.startswith('/') else '/' + name
            topic = f'{full}/_action/status'
            n = self._topics_with_publishers(topic) if topic in graph else 0
            out.append(self._r(s, full, PASS if n else FAIL,
                               owner if n else f'액션 서버 없음 ({owner})'))
        return out

    def check_services(self):
        s = 'services'
        names = {n for n, _ in self.get_service_names_and_types()}
        out = []
        for name, owner in _SERVICES:
            out.append(self._r(s, name, PASS if name in names else FAIL,
                               owner if name in names else f'서비스 없음 ({owner})'))
        return out

    # ── 실행/출력 ─────────────────────────────────────────────────────
    def run_once(self):
        results = []
        sections = [
            ('config', self.check_config),
            ('network', self.check_network),
            ('nodes', self.check_nodes),
            ('ctrl', self.check_controllers),
            ('topics', lambda: self.check_topics('mobile') + self.check_topics('camera')),
            ('joints', lambda: self.check_topics('joints') + self.check_joint_content()),
            ('tf', self.check_tf),
            ('actions', self.check_actions),
            ('services', self.check_services),
        ]
        for name, fn in sections:
            if self._enabled(name):
                try:
                    results.extend(fn())
                except Exception as e:  # noqa: BLE001 - 한 섹션 실패가 전체를 막지 않게
                    results.append(self._r(name, '(점검 중 예외)', FAIL, repr(e)))
        return results

    def report(self, results):
        color = sys.stdout.isatty()

        def paint(st):
            return f'{_COLOR[st]}{st:<4}{_RESET}' if color else f'{st:<4}'

        last = None
        print()
        print(f'=== bringup_doctor  spec={self.spec_name}  bridge={self.bridge_type}'
              f'  fake={self.fake}  domain={os.environ.get("ROS_DOMAIN_ID", "0")} ===')
        for r in results:
            if r['section'] != last:
                last = r['section']
                print(f'\n[{last}]')
            print(f'  {paint(r["status"])}  {r["name"]}'
                  + (f'  — {r["detail"]}' if r['detail'] else ''))
        cnt = {k: sum(1 for r in results if r['status'] == k) for k in (PASS, WARN, FAIL, SKIP)}
        print(f'\n요약: PASS {cnt[PASS]} / WARN {cnt[WARN]} / FAIL {cnt[FAIL]} / SKIP {cnt[SKIP]}')
        fails = [r for r in results if r['status'] == FAIL]
        if fails:
            print('실패 항목:')
            for r in fails:
                print(f'  - [{r["section"]}] {r["name"]}: {r["detail"]}')
        out = self.g('json_out')
        if out:
            try:
                with open(out, 'w', encoding='utf-8') as f:
                    json.dump({'summary': cnt, 'results': results}, f,
                              ensure_ascii=False, indent=2)
            except OSError as e:
                print(f'JSON 저장 실패: {e}')
        return cnt

    def exit_code(self, cnt):
        if cnt[FAIL]:
            return 1
        if cnt[WARN] and self.g('strict'):
            return 2
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = BringupDoctor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    code = 1
    try:
        # 그래프 발견 대기 → 구독 부착 → Hz 측정 창
        time.sleep(float(node.g('discovery_sec')))
        node.attach_probes()
        time.sleep(float(node.g('sample_sec')))
        while True:
            node.attach_probes()  # 늦게 뜬 토픽도 잡는다 (watch 모드)
            cnt = node.report(node.run_once())
            code = node.exit_code(cnt)
            if not node.g('watch'):
                break
            time.sleep(float(node.g('watch_period_sec')))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
