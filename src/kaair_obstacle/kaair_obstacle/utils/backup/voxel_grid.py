import numpy as np
from typing import Dict, List, Set, Tuple

"""
=============================================================================
VoxelGrid : Sparse 3D Occupancy Grid
=============================================================================
핵심 자료구조:
  voxels : dict  { (ix, iy, iz) → centroid(np.array [x,y,z]) }
    - key   : 정수 voxel 인덱스 튜플
    - value : 해당 voxel에 속한 포인트들의 running mean centroid

[인덱싱 방식]
  voxel 인덱스 = floor(coord / voxel_size)
  → 좌표 → 정수 인덱스로 양방향 변환 가능
  → 두 voxel의 인접 여부 = 인덱스 차이가 각 축 ±1 이내

[Running Mean Centroid]
  새 포인트가 들어올 때마다 평균을 점진적으로 갱신
  → 메모리: voxel당 (centroid xyz + count) 만 저장
  → 전체 포인트 배열을 보관하지 않아도 됨
=============================================================================
"""


class VoxelGrid:

    def __init__(self, voxel_size: float = 0.05):
        """
        Args:
            voxel_size: 각 voxel의 한 변 길이 (m 단위)
                        작을수록 정밀하지만 메모리·연산 증가
        """
        assert voxel_size > 0, "voxel_size must be positive"
        self.voxel_size: float = voxel_size

        # 핵심 저장소
        self.voxels:  Dict[Tuple, np.ndarray] = {}   # key → centroid
        self._counts: Dict[Tuple, int]         = {}   # key → 포인트 수 (running mean용)

    # ----------------------------------------------------------------
    def clear(self):
        """모든 voxel 초기화 (프레임 시작 시 호출)"""
        self.voxels.clear()
        self._counts.clear()

    # ----------------------------------------------------------------
    def _coord_to_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """실수 좌표 → 정수 voxel 인덱스"""
        vs = self.voxel_size
        return (int(np.floor(x / vs)),
                int(np.floor(y / vs)),
                int(np.floor(z / vs)))

    def key_to_center(self, key: Tuple[int, int, int]) -> np.ndarray:
        """voxel 인덱스 → 해당 voxel 중심 좌표"""
        vs = self.voxel_size
        return np.array([(key[0] + 0.5) * vs,
                         (key[1] + 0.5) * vs,
                         (key[2] + 0.5) * vs], dtype=np.float32)

    # ----------------------------------------------------------------
    def insert_points(self, points: np.ndarray):
        """
        (N, 3) float32 배열 → Sparse Voxel Grid 에 삽입

        알고리즘:
          1. 전체 포인트를 voxel 인덱스로 벡터 변환 (numpy 연산)
          2. 동일 voxel에 속하는 포인트끼리 그룹화
          3. 각 voxel의 running mean centroid 갱신

        시간복잡도: O(N log N)  (numpy 내부 정렬 기반)
        """
        vs = self.voxel_size

        # (1) 좌표 → 인덱스 변환 (벡터화)
        indices = np.floor(points / vs).astype(np.int32)  # (N, 3)

        # (2) 고유 voxel 키 추출 + 역인덱스
        # np.unique 는 axis=0 으로 행 단위 unique 지원
        unique_keys, inverse = np.unique(indices, axis=0, return_inverse=True)

        # (3) 각 voxel 별 centroid 계산 (numpy.bincount 활용)
        n_unique = len(unique_keys)

        for dim in range(3):
            # 각 차원별 합산
            pass

        # 벡터화된 per-voxel 평균 계산
        sum_xyz = np.zeros((n_unique, 3), dtype=np.float64)
        count_v = np.zeros(n_unique, dtype=np.int32)

        np.add.at(sum_xyz, inverse, points)
        np.add.at(count_v, inverse, 1)

        centroids = (sum_xyz / count_v[:, np.newaxis]).astype(np.float32)

        # (4) dict 에 저장 (기존 voxel 이면 running mean 갱신)
        for i in range(n_unique):
            key = tuple(unique_keys[i])
            new_c   = centroids[i]
            new_cnt = int(count_v[i])

            if key in self.voxels:
                # Running mean 갱신
                old_c   = self.voxels[key]
                old_cnt = self._counts[key]
                total   = old_cnt + new_cnt
                self.voxels[key]  = (old_c * old_cnt + new_c * new_cnt) / total
                self._counts[key] = total
            else:
                self.voxels[key]  = new_c
                self._counts[key] = new_cnt

    # ----------------------------------------------------------------
    def get_occupied_keys(self) -> List[Tuple[int, int, int]]:
        """점령된 voxel 키 목록 반환"""
        return list(self.voxels.keys())

    def get_centers(self) -> np.ndarray:
        """
        모든 occupied voxel의 centroid 반환
        반환: (N, 3) float32 numpy array
        """
        if not self.voxels:
            return np.empty((0, 3), dtype=np.float32)
        return np.array(list(self.voxels.values()), dtype=np.float32)

    def remove_keys(self, keys: Set[Tuple]):
        """지정 키 집합 삭제 (바닥 제거 등에 사용)"""
        for k in keys:
            self.voxels.pop(k, None)
            self._counts.pop(k, None)

    # ----------------------------------------------------------------
    def __len__(self) -> int:
        return len(self.voxels)

    def __repr__(self) -> str:
        return (f"VoxelGrid(voxel_size={self.voxel_size}m, "
                f"occupied={len(self.voxels)} voxels)")