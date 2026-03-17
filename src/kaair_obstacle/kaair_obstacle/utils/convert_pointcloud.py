import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

"""
=============================================================================
convert_pointcloud : numpy array ↔ ROS2 PointCloud2 변환 유틸
=============================================================================
ROS2 PointCloud2 바이너리 포맷 이해:

  PointCloud2 메시지는 포인트 데이터를 raw bytes 로 저장
  각 포인트는 fields 정의에 따라 연속적으로 packing 됨

  [XYZ only]  한 포인트 = 12 bytes (float32 x3)
  [XYZRGB]    한 포인트 = 16 bytes (float32 x3 + float32 1개에 RGB packing)

  RGB packing 방식:
    - 실제 저장은 float32 이지만 내부 비트는 uint32 [R8 G8 B8 0]
    - struct.pack 으로 RGB → 4bytes → float32 로 재해석
    - RViz2 가 이 방식의 'rgb' 필드를 자동으로 인식함

[성능 최적화]
  - np.tobytes() 로 전체 배열을 한 번에 직렬화 (loop 없음)
  - struct.pack_into 로 color 채널도 배열 단위로 처리
=============================================================================
"""

# ─────────────────────────────────────────────────────────────────────────────
# XYZ 전용 PointCloud2
# ─────────────────────────────────────────────────────────────────────────────

_XYZ_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]
_XYZ_POINT_STEP = 12  # float32 x 3 = 12 bytes


def numpy_to_ros(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """
    (N, 3) float32 numpy → XYZ PointCloud2

    사용 예:
        self.pub.publish(numpy_to_ros(pts, msg.header))
    """
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12  # 3 × float32
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.astype(np.float32).tobytes()
    return msg


# ─────────────────────────────────────────────────────────────────────────────
# XYZRGB PointCloud2
# ─────────────────────────────────────────────────────────────────────────────

_XYZRGB_FIELDS = [
    PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]
_XYZRGB_POINT_STEP = 16  # float32 x 3 + float32(rgb) = 16 bytes


def numpy_to_ros_rgb(
    points: np.ndarray,
    colors: np.ndarray,
    header: Header
) -> PointCloud2:
    """
    (N, 3) float32 + (N, 3) float32 [0~1 RGB] → XYZRGB PointCloud2

    colors 범위: 0.0 ~ 1.0  (RViz2 표준)

    RGB packing 과정:
      [r, g, b] float 0~1
           ↓ x255 → uint8
      [R, G, B] uint8
           ↓ struct.pack '>I' (big-endian uint32)  R<<16 | G<<8 | B
      4 bytes uint32
           ↓ struct.unpack 'f' (float32 재해석)
      float32 → PointCloud2 rgb 필드에 저장

    사용 예:
        self.pub.publish(numpy_to_ros_rgb(pts, colors, msg.header))
    """
    n = len(points)
    assert len(colors) == n, "points와 colors의 길이가 다릅니다"

    pts    = np.ascontiguousarray(points, dtype=np.float32)
    colors = np.clip(colors, 0.0, 1.0)

    # uint8 변환
    r = (colors[:, 0] * 255).astype(np.uint8)
    g = (colors[:, 1] * 255).astype(np.uint8)
    b = (colors[:, 2] * 255).astype(np.uint8)

    # RGB → uint32 packing (벡터화)
    rgb_uint32 = (r.astype(np.uint32) << 16 |
                  g.astype(np.uint32) << 8  |
                  b.astype(np.uint32))

    # uint32 → float32 비트 재해석 (RViz2 rgb 필드 규약)
    rgb_float32 = rgb_uint32.view(np.float32)

    # (N, 4) 배열 생성 [x, y, z, rgb]
    xyzrgb = np.column_stack([pts, rgb_float32]).astype(np.float32)
    xyzrgb = np.ascontiguousarray(xyzrgb)

    msg = PointCloud2()
    msg.header       = header
    msg.height       = 1
    msg.width        = n
    msg.fields       = _XYZRGB_FIELDS
    msg.is_bigendian  = False
    msg.point_step   = _XYZRGB_POINT_STEP
    msg.row_step     = _XYZRGB_POINT_STEP * n
    msg.is_dense     = True
    msg.data         = xyzrgb.tobytes()

    return msg


# ─────────────────────────────────────────────────────────────────────────────
# 역방향: ROS2 PointCloud2 → numpy
# ─────────────────────────────────────────────────────────────────────────────

def ros_to_numpy(msg: PointCloud2) -> np.ndarray:
    """
    XYZ PointCloud2 → (N, 3) float32 numpy

    sensor_msgs_py 의존 없이 직접 파싱
    (일부 환경에서 sensor_msgs_py 가 없을 경우 대비)
    """
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n   = msg.width * msg.height

    # float32 3개 연속 → stride 방식으로 추출
    step = msg.point_step

    # x, y, z 필드 offset 탐색
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = f.offset

    if not all(k in offsets for k in ('x', 'y', 'z')):
        raise ValueError("PointCloud2에 x, y, z 필드가 없습니다")

    pts = np.zeros((n, 3), dtype=np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        off = offsets[ax]
        # 각 포인트에서 해당 offset 위치의 4 bytes 를 float32 로 해석
        pts[:, i] = np.frombuffer(
            bytes(b''.join(
                raw[j * step + off: j * step + off + 4].tobytes()
                for j in range(n)
            )),
            dtype=np.float32
        )

    return pts