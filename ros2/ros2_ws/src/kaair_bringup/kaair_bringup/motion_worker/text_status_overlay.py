#!/usr/bin/env python3
"""
text_status_overlay.py
────────────────────────────────────────────────────────────────────────
범용 "String → RViz 화면 고정 텍스트(OverlayText)" 브리지 노드.

이 노드 자체는 servo 등 특정 기능을 전혀 알지 못한다. 그냥 std_msgs/String
입력 토픽을 구독해서, 그 텍스트를 rviz_2d_overlay_plugins 의 TextOverlay
디스플레이가 이해하는 rviz_2d_overlay_msgs/OverlayText 로 그대로 옮겨
발행할 뿐이다. 그래서 나중에 servo 상태 말고 다른 문자열(예: 배터리, 작업
진행 상태 등)을 RViz 에 띄우고 싶을 때도, 이 노드를 다른 파라미터(입력
토픽·화면 위치·기본 텍스트)로 한 번 더 띄우기만 하면 재사용할 수 있다.

입력 (std_msgs/String, input_topic 파라미터)
  "<level>:<text>" 형식을 우선 시도한다. level 은 info(초록) | warn(주황)
  | error(빨강) 중 하나. level 이 없거나 인식할 수 없으면 전체 문자열을
  그대로 text 로 쓰고 level=info 로 취급한다.
    예) "warn:ARM: SERVO (안정화 중...)"  → 주황색 텍스트
        "그냥 아무 문자열"                → 초록색 텍스트 그대로

출력 (rviz_2d_overlay_msgs/OverlayText, overlay_topic 파라미터, latched)
  화면 고정 위치(기본: 우측 하단)에 표시된다. latched(Transient Local) QoS
  를 쓰지만, rviz_2d_overlay_plugins 쪽에서 첫 구독 시점에 이 QoS 를 항상
  제대로 반영하지는 않아 — RViz 가 이 노드보다 늦게 뜬 경우 글자가 아예 안
  보이거나, 디스플레이 체크박스를 껐다 켜야만 보이는 문제가 있었다. 그래서
  마지막으로 발행한 내용을 republish_period_sec 마다 그대로 재발행해,
  RViz 가 최초 구독을 놓쳐도 늦어도 그 주기 안에는 반드시 표시되게 한다.

기동 시 default_text 파라미터가 비어있지 않으면 즉시 1회 발행한다 —
아직 아무 입력도 없는 "평시" 상태에서도 화면에 뭔가 표시되도록 하기 위함
(예: servo 상태 용도로 쓸 때는 default_text 를 "ARM: PLANNING (config)"
로 설정해, servo 를 켜는 노드(3d_master_controller 등)가 아예 실행되지
않은 평범한 상태에서도 올바른 기본값이 보이게 한다).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, String

_LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

_LEVEL_COLORS = {
    'info':  (0.2, 0.9, 0.2),   # 초록
    'warn':  (1.0, 0.65, 0.0),  # 주황
    'error': (1.0, 0.15, 0.15), # 빨강
}

_ALIGN_MAP = {
    'left':   OverlayText.LEFT,
    'right':  OverlayText.RIGHT,
    'center': OverlayText.CENTER,
    'top':    OverlayText.TOP,
    'bottom': OverlayText.BOTTOM,
}


class TextStatusOverlay(Node):
    def __init__(self):
        super().__init__('text_status_overlay')

        self.declare_parameter('input_topic', '/servo_mode/state_text')
        self.declare_parameter('overlay_topic', '/servo_mode/status_overlay')
        self.declare_parameter('default_level', 'info')
        self.declare_parameter('default_text', '')
        self.declare_parameter('horizontal_alignment', 'right')
        self.declare_parameter('vertical_alignment', 'bottom')
        self.declare_parameter('horizontal_distance', 10)
        self.declare_parameter('vertical_distance', 10)
        self.declare_parameter('width', 280)
        self.declare_parameter('height', 40)
        self.declare_parameter('text_size', 14.0)
        self.declare_parameter('font', 'DejaVu Sans Mono')
        self.declare_parameter('bg_alpha', 0.6)
        # RViz 가 최초 구독 시점에 latched 메시지를 못 받아 글자가 안 보이는
        # 문제(체크박스를 껐다 켜야만 보임)를 피하기 위해 주기적으로 재발행
        self.declare_parameter('republish_period_sec', 1.0)

        input_topic   = self.get_parameter('input_topic').value
        overlay_topic = self.get_parameter('overlay_topic').value
        self._default_level = self.get_parameter('default_level').value
        default_text  = self.get_parameter('default_text').value
        republish_period_sec = float(self.get_parameter('republish_period_sec').value)

        h_align = str(self.get_parameter('horizontal_alignment').value).lower()
        v_align = str(self.get_parameter('vertical_alignment').value).lower()
        self._h_align = _ALIGN_MAP.get(h_align, OverlayText.RIGHT)
        self._v_align = _ALIGN_MAP.get(v_align, OverlayText.BOTTOM)

        self._h_dist    = int(self.get_parameter('horizontal_distance').value)
        self._v_dist    = int(self.get_parameter('vertical_distance').value)
        self._width     = int(self.get_parameter('width').value)
        self._height    = int(self.get_parameter('height').value)
        self._text_size = float(self.get_parameter('text_size').value)
        self._font      = str(self.get_parameter('font').value)
        self._bg_alpha  = float(self.get_parameter('bg_alpha').value)

        # 마지막으로 발행한 (level, text) — republish 타이머가 이걸 그대로 재발행한다.
        self._last_level = None
        self._last_text = None

        self._overlay_pub = self.create_publisher(OverlayText, overlay_topic, _LATCHED_QOS)
        self.create_subscription(String, input_topic, self._on_input, 10)
        self.create_timer(republish_period_sec, self._on_republish_timer)

        self.get_logger().info(
            f'TextStatusOverlay ready. {input_topic} → {overlay_topic} '
            f'(align={h_align}/{v_align}, republish={republish_period_sec}s)'
        )

        if default_text:
            self._publish(self._default_level, default_text)

    def _parse(self, data: str):
        """"<level>:<text>" 를 (level, text) 로 나눈다. level 이 없거나
        인식할 수 없는 값이면 (default_level, data 전체) 를 반환한다."""
        if ':' in data:
            level, _, text = data.partition(':')
            level = level.strip().lower()
            if level in _LEVEL_COLORS:
                return level, text.strip()
        return self._default_level, data

    def _on_input(self, msg: String):
        level, text = self._parse(msg.data)
        self._publish(level, text)

    def _on_republish_timer(self):
        """RViz 가 최초 구독 시 latched 메시지를 놓쳐 화면에 아무것도 안
        뜨는 경우를 대비해, 마지막으로 발행한 내용을 그대로 다시 보낸다."""
        if self._last_text is None:
            return
        self._publish(self._last_level, self._last_text)

    def _publish(self, level: str, text: str):
        self._last_level = level
        self._last_text = text

        r, g, b = _LEVEL_COLORS.get(level, _LEVEL_COLORS['info'])

        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.width = self._width
        overlay.height = self._height
        overlay.horizontal_distance = self._h_dist
        overlay.vertical_distance = self._v_dist
        overlay.horizontal_alignment = self._h_align
        overlay.vertical_alignment = self._v_align
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=self._bg_alpha)
        overlay.line_width = 2
        overlay.text_size = self._text_size
        overlay.font = self._font
        overlay.fg_color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        overlay.text = text
        self._overlay_pub.publish(overlay)


def main(args=None):
    rclpy.init(args=args)
    node = TextStatusOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
