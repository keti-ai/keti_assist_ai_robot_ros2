import numpy as np
from dataclasses import dataclass, field
from rclpy.node import Node

"""
=============================================================================
PipelineConfig : 파이프라인 전체 파라미터 데이터클래스
load_config    : ROS2 Node 파라미터에서 PipelineConfig 로드
=============================================================================
ROS2 파라미터 시스템 활용:
  - 런타임에 ros2 param set 으로 실시간 변경 가능
  - launch 파일의 params.yaml 로 일괄 설정 가능
  - 각 파라미터에 설명(description)과 기본값이 정의됨

[파라미터 그룹]
  1. 토픽          : 입력 PointCloud2 토픽명
  2. ROI           : 관심 영역 박스 범위 (m)
  3. Voxel         : voxel 해상도
  4. 바닥 제거      : 히스토그램 기반 바닥 판정 기준
  5. 클러스터링     : BFS 클러스터 크기 필터 + 연결성
  6. 로봇 안전 구역 : collision check 기준 거리
=============================================================================
"""


@dataclass
class PipelineConfig:
    # ── 1. 토픽 ────────────────────────────────────────────────────
    input_topic: str = '/camera/depth/color/points'

    # ── 2. ROI (Region of Interest) ────────────────────────────────
    roi_x_min: float = -3.0
    roi_x_max: float =  3.0
    roi_y_min: float = -3.0
    roi_y_max: float =  3.0
    roi_z_min: float =  0.0    # 0 이하는 카메라 뒤 or 지하 → 제거
    roi_z_max: float =  2.5    # 천장 높이

    # ── 3. Voxel Grid ──────────────────────────────────────────────
    voxel_size: float = 0.05   # 5cm (실내 장애물 검출에 적합)

    # ── 4. 바닥 제거 ────────────────────────────────────────────────
    # Z 히스토그램의 하위 X% → 바닥 기준 Z
    ground_z_percentile: float   = 10.0
    # 바닥 기준 Z 위로 이 높이까지 바닥으로 간주 (m)
    ground_height_threshold: float = 0.15

    # ── 5. BFS 클러스터링 ───────────────────────────────────────────
    min_cluster_voxels: int = 5      # 이보다 작으면 노이즈로 제거
    max_cluster_voxels: int = 5000   # 이보다 크면 벽/천장으로 제거
    cluster_connectivity: int = 26   # 26-연결 or 6-연결

    # ── 6. 로봇 안전 구역 ───────────────────────────────────────────
    robot_safety_radius: float = 0.5          # 충돌 판정 반경 (m)
    robot_pos: np.ndarray = field(            # 로봇 위치 (카메라 좌표계)
        default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float32)
    )

    def __repr__(self):
        return (
            f"PipelineConfig("
            f"topic={self.input_topic}, "
            f"voxel={self.voxel_size}m, "
            f"roi_z=[{self.roi_z_min},{self.roi_z_max}], "
            f"ground_thr={self.ground_height_threshold}m, "
            f"cluster=[{self.min_cluster_voxels}~{self.max_cluster_voxels}]vox, "
            f"safety_r={self.robot_safety_radius}m)"
        )


# ============================================================================
def load_config(node: Node) -> PipelineConfig:
    """
    ROS2 Node 의 파라미터를 선언하고 PipelineConfig 로 반환

    사용법:
        self.cfg = load_config(self)   # __init__ 에서 호출
    """

    def dp(name, default, desc=""):
        """declare_parameter 단축 함수"""
        from rcl_interfaces.msg import ParameterDescriptor
        descriptor = ParameterDescriptor(description=desc)
        node.declare_parameter(name, default, descriptor)
        return node.get_parameter(name).value

    cfg = PipelineConfig()

    # ── 1. 토픽 ──────────────────────────────────────────────────────
    cfg.input_topic = dp(
        'input_topic', cfg.input_topic,
        'Orbbec PointCloud2 입력 토픽명'
    )

    # ── 2. ROI ────────────────────────────────────────────────────────
    cfg.roi_x_min = dp('roi_x_min', cfg.roi_x_min, 'ROI x 최솟값 (m)')
    cfg.roi_x_max = dp('roi_x_max', cfg.roi_x_max, 'ROI x 최댓값 (m)')
    cfg.roi_y_min = dp('roi_y_min', cfg.roi_y_min, 'ROI y 최솟값 (m)')
    cfg.roi_y_max = dp('roi_y_max', cfg.roi_y_max, 'ROI y 최댓값 (m)')
    cfg.roi_z_min = dp('roi_z_min', cfg.roi_z_min, 'ROI z 최솟값 (m)')
    cfg.roi_z_max = dp('roi_z_max', cfg.roi_z_max, 'ROI z 최댓값 (m)')

    # ── 3. Voxel ──────────────────────────────────────────────────────
    cfg.voxel_size = dp(
        'voxel_size', cfg.voxel_size,
        'Voxel 한 변 크기 (m). 작을수록 정밀하지만 느려짐'
    )

    # ── 4. 바닥 제거 ──────────────────────────────────────────────────
    cfg.ground_z_percentile    = dp(
        'ground_z_percentile', cfg.ground_z_percentile,
        'Z 히스토그램 하위 N% → 바닥 기준 Z 결정'
    )
    cfg.ground_height_threshold = dp(
        'ground_height_threshold', cfg.ground_height_threshold,
        '바닥 기준 Z 위로 이 높이까지 바닥으로 간주 (m)'
    )

    # ── 5. 클러스터링 ─────────────────────────────────────────────────
    cfg.min_cluster_voxels  = dp(
        'min_cluster_voxels', cfg.min_cluster_voxels,
        '클러스터 최소 voxel 수 (이하 → 노이즈 제거)'
    )
    cfg.max_cluster_voxels  = dp(
        'max_cluster_voxels', cfg.max_cluster_voxels,
        '클러스터 최대 voxel 수 (이상 → 배경 제거)'
    )
    cfg.cluster_connectivity = dp(
        'cluster_connectivity', cfg.cluster_connectivity,
        '26-연결 or 6-연결 BFS'
    )

    # ── 6. 로봇 안전 구역 ─────────────────────────────────────────────
    cfg.robot_safety_radius = dp(
        'robot_safety_radius', cfg.robot_safety_radius,
        'AABB와 로봇 사이 충돌 판정 반경 (m)'
    )
    robot_pos_list = dp(
        'robot_pos', [0.0, 0.0, 0.0],
        '로봇 위치 [x, y, z] (카메라 좌표계 기준)'
    )
    cfg.robot_pos = np.array(robot_pos_list, dtype=np.float32)

    return cfg