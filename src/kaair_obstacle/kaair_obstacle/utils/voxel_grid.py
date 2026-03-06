import numpy as np

# ============================================================================
# VoxelGrid : 핵심 자료구조
# ============================================================================
class VoxelGrid:
    """
    Sparse 3D Voxel Grid
    - key   : (ix, iy, iz) 정수 인덱스 tuple
    - value : 해당 voxel에 속한 포인트들의 centroid (numpy array)

    장점: 빈 공간을 메모리에 저장하지 않아 대규모 공간에도 효율적
    """

    def __init__(self, voxel_size: float):
        self.voxel_size = voxel_size
        self.voxels: dict = {}          # {(ix,iy,iz): np.array([x,y,z])}
        self.point_counts: dict = {}    # {(ix,iy,iz): int}  - 평균 계산용

    def clear(self):
        self.voxels.clear()
        self.point_counts.clear()

    def world_to_index(self, x, y, z):
        """실수 좌표 → voxel 정수 인덱스"""
        ix = int(np.floor(x / self.voxel_size))
        iy = int(np.floor(y / self.voxel_size))
        iz = int(np.floor(z / self.voxel_size))
        return (ix, iy, iz)

    def index_to_center(self, ix, iy, iz):
        """voxel 인덱스 → 중심 좌표"""
        x = (ix + 0.5) * self.voxel_size
        y = (iy + 0.5) * self.voxel_size
        z = (iz + 0.5) * self.voxel_size
        return np.array([x, y, z])

    def insert_points(self, points: np.ndarray):
        """
        포인트 배열을 voxel grid에 삽입
        각 voxel의 centroid를 누적 평균으로 업데이트
        """
        self.clear()
        # 벡터화 연산으로 모든 포인트를 한번에 인덱싱
        indices = np.floor(points / self.voxel_size).astype(np.int32)

        for i, (pt, idx) in enumerate(zip(points, indices)):
            key = (idx[0], idx[1], idx[2])
            if key not in self.voxels:
                self.voxels[key] = pt.copy()
                self.point_counts[key] = 1
            else:
                # 누적 평균 업데이트 (Running Mean)
                n = self.point_counts[key]
                self.voxels[key] = (self.voxels[key] * n + pt) / (n + 1)
                self.point_counts[key] = n + 1

    def get_occupied_keys(self):
        return list(self.voxels.keys())

    def get_centers(self):
        """모든 occupied voxel의 중심좌표 배열 반환"""
        if not self.voxels:
            return np.empty((0, 3))
        return np.array(list(self.voxels.values()))

    def remove_keys(self, keys_to_remove: set):
        for key in keys_to_remove:
            self.voxels.pop(key, None)
            self.point_counts.pop(key, None)
