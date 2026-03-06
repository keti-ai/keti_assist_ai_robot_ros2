from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import sensor_msgs_py.point_cloud2 as pc2


# ================================================================
# 유틸: PointCloud 변환
# ================================================================
def numpy_to_ros(points: np.ndarray, header) -> PointCloud2:
    """(N,3) float32 → PointCloud2 (XYZ only)"""
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
    return pc2.create_cloud_xyz32(header, pts)

def numpy_to_ros_rgb(points: np.ndarray, colors: np.ndarray, header: Header) -> PointCloud2:
    """(N,3) points + (N,3) RGB float → XYZRGB PointCloud2"""
    fields = [
        PointField(name='x', offset=0,
                    datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,
                    datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,
                    datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12,
                    datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 16
    data = []
    for pt, col in zip(points, colors):
        r = int(col[0] * 255)
        g = int(col[1] * 255)
        b = int(col[2] * 255)
        rgb_int = (r << 16) | (g << 8) | b
        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
        data.append(struct.pack('ffff',
                        float(pt[0]), float(pt[1]),
                        float(pt[2]), rgb_float))

    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = len(points)
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = point_step
    cloud_msg.row_step = point_step * len(points)
    cloud_msg.data = b''.join(data)
    cloud_msg.is_dense = True
    return cloud_msg