#!/usr/bin/env python3
"""
픽셀 클릭 → create_object_marker → arm_moveT 캘리브레이션 확인 GUI

1) /femto/color/image_raw RGB 를 OpenCV 창으로 띄운다.
2) 창에서 픽셀을 좌클릭하면 /create_object_marker 서비스로 해당 (u, v) 의
   3D 좌표를 받아온다 (base_frame=link_base, RViz 에 마커 생성).
3) 'm' 키를 누르면 현재 TCP 자세 기준으로 해당 좌표까지의 tool-delta 를 계산해
   /kaair_worker/arm_moveT (MoveTool) 액션으로 이동한다. 회전은 유지(quat=identity).
4) 이동 완료 후 final_pose 와 목표 좌표의 차이(=캘리브레이션 오차)를 출력한다.

키:
  좌클릭 : 픽셀 선택 + 마커 생성
  m      : 선택 좌표로 이동 (실행)
  p      : plan_only 로 이동 테스트 (실제 이동 X)
  c      : 오브젝트 마커 전체 삭제 (/clear_object_markers)
  q/ESC  : 종료

standoff : 목표점 앞에서 멈출 거리 [m]. TCP 의 approach 축(tool z) 기준으로
           목표점에서 standoff 만큼 뺀 위치로 이동한다. 0.0 이면 목표점에 TCP 를 맞춘다.

실행 예:
  python3 pixel_marker_moveT_gui.py
  python3 pixel_marker_moveT_gui.py --ros-args \
    -p standoff:=0.05 -p velocity_scale:=0.1 -p acceleration_scale:=0.1

의존: rclpy, kaair_msgs, tf2_ros, opencv-python, numpy, scipy
"""

import threading

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from kaair_msgs.action import MoveTool
from kaair_msgs.srv import CreateObjectMarker

WINDOW = 'pixel -> marker -> moveT'

# object_marker_server 가 기대하는 aligned depth 해상도 (u, v 는 이 해상도 기준)
DEPTH_W, DEPTH_H = 1280, 720

# arm_move_action_server 의 MoveTool 기준 프레임
BASE_FRAME = 'link_base'
EEF_LINK = 'tool_tcp_link'


class PixelMarkerMoveTGui(Node):
    def __init__(self):
        super().__init__('pixel_marker_movet_gui')
        self.declare_parameter('image_topic', '/femto/color/image_raw')
        self.declare_parameter('marker_service', '/create_object_marker')
        self.declare_parameter('action_name', '/kaair_worker/arm_moveT')
        self.declare_parameter('object_label', 'calib_target')
        self.declare_parameter('standoff', 0.05)
        # 안전용: tool-delta 크기가 이보다 크면 이동 거부 [m]
        self.declare_parameter('max_distance', 0.6)
        self.declare_parameter('velocity_scale', 0.1)
        self.declare_parameter('acceleration_scale', 0.1)

        self.label = self.get_parameter('object_label').value
        self.standoff = float(self.get_parameter('standoff').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        self.velocity_scale = float(self.get_parameter('velocity_scale').value)
        self.acceleration_scale = float(self.get_parameter('acceleration_scale').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_cb, sensor_qos
        )

        self.marker_cli = self.create_client(
            CreateObjectMarker, self.get_parameter('marker_service').value
        )
        self.clear_cli = self.create_client(Trigger, '/clear_object_markers')
        self.move_cli = ActionClient(self, MoveTool, self.get_parameter('action_name').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()
        self.latest_bgr = None
        self.clicked_uv = None          # GUI 이미지 좌표
        self.target_base = None         # link_base 기준 목표점 (np.array)
        self.busy = False
        self.status = '좌클릭으로 픽셀 선택'

    # ------------------------------------------------------------------ image
    def _image_cb(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().error(f'Unsupported encoding: {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.step // 3, 3))[:, :msg.width]
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        with self._lock:
            self.latest_bgr = img.copy()

    def get_frame(self):
        with self._lock:
            return None if self.latest_bgr is None else self.latest_bgr.copy()

    def _set_status(self, text, error=False):
        self.status = text
        (self.get_logger().error if error else self.get_logger().info)(text)

    # ----------------------------------------------------------- marker srv
    def request_marker(self, u_img, v_img, img_w, img_h):
        if self.busy:
            self._set_status('이전 요청 처리 중...', error=True)
            return
        if not self.marker_cli.service_is_ready():
            self._set_status('create_object_marker 서비스 없음', error=True)
            return

        # 컬러 이미지 해상도가 depth 와 다르면 depth 픽셀 좌표로 스케일링
        u = int(round(u_img * DEPTH_W / img_w))
        v = int(round(v_img * DEPTH_H / img_h))

        req = CreateObjectMarker.Request()
        req.object_label = self.label
        req.u_array = [u]
        req.v_array = [v]
        req.base_frame = BASE_FRAME

        self.busy = True
        self.clicked_uv = (u_img, v_img)
        self.target_base = None
        self._set_status(f'marker 요청 (u={u}, v={v})...')
        self.marker_cli.call_async(req).add_done_callback(self._marker_done)

    def _marker_done(self, future):
        self.busy = False
        res = future.result()
        if res is None:
            self._set_status('서비스 호출 실패', error=True)
            return
        if not res.success or not np.isfinite(res.x_array[0]):
            self._set_status(f'marker 실패: {res.message}', error=True)
            return
        self.target_base = np.array([res.x_array[0], res.y_array[0], res.z_array[0]])
        x, y, z = self.target_base
        self._set_status(
            f'target({BASE_FRAME}) = ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'depth={res.depth_raw_array[0]:.3f}m  -> m:이동 p:plan'
        )

    def clear_markers(self):
        if self.clear_cli.service_is_ready():
            self.clear_cli.call_async(Trigger.Request())
        self.clicked_uv = None
        self.target_base = None
        self._set_status('마커 삭제')

    # ------------------------------------------------------------- moveT
    def _lookup_tcp(self):
        tf = self.tf_buffer.lookup_transform(
            BASE_FRAME, EEF_LINK, rclpy.time.Time(), timeout=Duration(seconds=1.0)
        )
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), R.from_quat([q.x, q.y, q.z, q.w])

    def move_to_target(self, plan_only=False):
        if self.busy:
            self._set_status('이전 요청 처리 중...', error=True)
            return
        if self.target_base is None:
            self._set_status('먼저 픽셀을 선택하세요', error=True)
            return
        if not self.move_cli.server_is_ready():
            self._set_status('arm_moveT 액션 서버 없음', error=True)
            return

        try:
            tcp_p, tcp_rot = self._lookup_tcp()
        except Exception as e:
            self._set_status(f'TF lookup 실패 {BASE_FRAME}<-{EEF_LINK}: {e}', error=True)
            return

        # 서버: target = tcp_p + tcp_rot.apply(delta_tool) → delta_tool = R^T (target - tcp_p)
        delta_tool = tcp_rot.inv().apply(self.target_base - tcp_p)
        delta_tool[2] -= self.standoff
        dist = float(np.linalg.norm(delta_tool))
        if dist > self.max_distance:
            self._set_status(
                f'이동 거리 {dist:.3f}m > max_distance {self.max_distance:.3f}m, 거부',
                error=True)
            return

        # standoff 반영된 실제 목표 (link_base) — 결과 오차 비교용
        self._commanded_base = tcp_p + tcp_rot.apply(delta_tool)

        goal = MoveTool.Goal()
        goal.dx, goal.dy, goal.dz = (float(d) for d in delta_tool)
        goal.qx, goal.qy, goal.qz, goal.qw = 0.0, 0.0, 0.0, 1.0
        goal.plan_only = plan_only
        goal.velocity_scale = self.velocity_scale
        goal.acceleration_scale = self.acceleration_scale

        self.busy = True
        self._set_status(
            f'MoveT {"plan_only " if plan_only else ""}delta_tool='
            f'({goal.dx:.3f}, {goal.dy:.3f}, {goal.dz:.3f}) |{dist:.3f}m|'
        )
        self.move_cli.send_goal_async(goal, self._feedback_cb).add_done_callback(
            self._goal_response)

    def _feedback_cb(self, msg):
        self.get_logger().info(f'feedback: {msg.feedback.status}', throttle_duration_sec=0.5)

    def _goal_response(self, future):
        gh = future.result()
        if gh is None or not gh.accepted:
            self.busy = False
            self._set_status('MoveT goal 거절됨', error=True)
            return
        gh.get_result_async().add_done_callback(self._move_done)

    def _move_done(self, future):
        self.busy = False
        wrapped = future.result()
        res = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not res.success:
            self._set_status(f'MoveT 실패: {res.message}', error=True)
            return

        p = res.final_pose.position
        final = np.array([p.x, p.y, p.z])
        if np.allclose(final, 0.0):
            self._set_status(f'MoveT 성공: {res.message}')
            return
        err = final - self._commanded_base
        self._set_status(
            f'MoveT 성공 final=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'err=({err[0]*1000:.1f}, {err[1]*1000:.1f}, {err[2]*1000:.1f})mm '
            f'|{np.linalg.norm(err)*1000:.1f}mm|'
        )


def draw_overlay(img, node: PixelMarkerMoveTGui):
    if node.clicked_uv is not None:
        u, v = node.clicked_uv
        color = (0, 255, 0) if node.target_base is not None else (0, 200, 255)
        cv2.drawMarker(img, (u, v), color, cv2.MARKER_CROSS, 30, 2)
        cv2.circle(img, (u, v), 6, color, 2)
    cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
    # OpenCV putText 는 한글 미지원 → 상태는 터미널 로그로 확인, 창에는 ASCII 만 표시
    text = node.status if node.status.isascii() else ''
    if node.target_base is not None:
        x, y, z = node.target_base
        text = f'target {BASE_FRAME}: ({x:.3f}, {y:.3f}, {z:.3f})  [m]move [p]plan [c]clear [q]quit'
    elif not text:
        text = '[click] select pixel  [c]clear  [q]quit'
    if node.busy:
        text = 'busy... ' + text
    cv2.putText(img, text, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                cv2.LINE_AA)
    return img


def main(args=None):
    rclpy.init(args=args)
    node = PixelMarkerMoveTGui()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    frame_size = {'w': DEPTH_W, 'h': DEPTH_H}

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            node.request_marker(x, y, frame_size['w'], frame_size['h'])

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, on_mouse)

    try:
        while rclpy.ok():
            frame = node.get_frame()
            if frame is None:
                frame = np.zeros((DEPTH_H, DEPTH_W, 3), np.uint8)
                cv2.putText(frame, 'waiting for image...', (40, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            frame_size['h'], frame_size['w'] = frame.shape[:2]
            cv2.imshow(WINDOW, draw_overlay(frame, node))

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('m'):
                node.move_to_target(plan_only=False)
            elif key == ord('p'):
                node.move_to_target(plan_only=True)
            elif key == ord('c'):
                node.clear_markers()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
