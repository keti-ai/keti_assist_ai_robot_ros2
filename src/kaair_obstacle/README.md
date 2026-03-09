# collision_avoidance

Robotic Collision Avoidance via 3D Object Simplification  
Orbbec 카메라 PointCloud2 → 단순화된 3D 표현 → 충돌 회피

---

## 파이프라인 구조

```
Orbbec PointCloud2
        │  /camera/depth/color/points
        ▼
┌─────────────────────┐
│  1. Preprocessing   │  Voxel Downsampling + PassThrough + SOR
└──────────┬──────────┘
           │  /pointcloud/preprocessed
           ▼
┌─────────────────────┐
│  2. Ground Removal  │  RANSAC Plane Segmentation
└──────────┬──────────┘
           │  /pointcloud/objects
           ▼
┌─────────────────────┐
│  3. Clustering      │  DBSCAN Euclidean Clustering
└──────────┬──────────┘
           │  /pointcloud/clusters  (XYZRGB)
           ▼
┌─────────────────────┐
│  4. Representation  │  AABB / OBB / Convex Hull / Cylinder
└──────────┬──────────┘
           │  /collision/markers  (MarkerArray → RViz2)
           ▼
        RViz2
```

---

## 의존성 설치

```bash
pip install open3d scipy
```

또는 시스템 패키지:
```bash
sudo apt install python3-open3d python3-scipy
```

---

## 빌드 & 실행

```bash
# 워크스페이스에 패키지 복사 후
cd ~/ros2_ws
colcon build --packages-select collision_avoidance
source install/setup.bash

# 전체 파이프라인 실행
ros2 launch collision_avoidance pipeline.launch.py

# Orbbec 토픽명이 다른 경우
ros2 launch collision_avoidance pipeline.launch.py \
    input_topic:=/orbbec/depth/points

# 표현 방식 변경
ros2 launch collision_avoidance pipeline.launch.py \
    representation_mode:=obb
```

---

## RViz2 설정

1. Fixed Frame → 카메라 frame_id (보통 `camera_depth_optical_frame`)
2. Add → By topic → `/pointcloud/preprocessed` (PointCloud2)
3. Add → By topic → `/pointcloud/objects` (PointCloud2)
4. Add → By topic → `/pointcloud/clusters` (PointCloud2) — RGB 컬러
5. Add → By topic → `/collision/markers` (MarkerArray) — 바운딩 박스

---

## 파라미터 튜닝 가이드

### Stage 1: Preprocessing

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `voxel_size` | 0.05 | 격자 크기(m). 작을수록 정밀, 클수록 빠름 |
| `z_min` | 0.05 | 바닥 바로 위. 카메라 높이에 따라 조정 |
| `z_max` | 2.5 | 관심 높이 상한 |
| `sor_std_ratio` | 2.0 | 작을수록 노이즈 제거 강도 ↑ (포인트 손실 주의) |

### Stage 2: Ground Removal

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `distance_threshold` | 0.02 | 평면 인라이어 허용 거리(m). 울퉁불퉁한 바닥이면 높여야 함 |
| `num_iterations` | 1000 | 많을수록 정확, 느림. 500도 충분 |
| `max_tilt_angle_deg` | 15 | 카메라 기울기 허용 각도. 마운트에 따라 조정 |

### Stage 3: Clustering

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `cluster_tolerance` | 0.05 | 같은 클러스터로 묶을 최대 거리(m) |
| `min_cluster_size` | 50 | 작은 노이즈 클러스터 제거 |
| `max_cluster_size` | 25000 | 너무 큰 덩어리(벽 등) 제거 |

### Stage 4: Representation

| 모드 | 특징 | 권장 사용 |
|------|------|-----------|
| `aabb` | 가장 빠름. 축 정렬 직육면체 | 기본값, 실시간 요구 시 |
| `obb` | PCA 기반 회전 직육면체 | 기울어진 물체 정밀 처리 |
| `convex_hull` | 볼록 다면체. 가장 정밀 | 느린 대신 정확한 형상 |
| `cylinder` | 수직 원기둥 | 사람/기둥 형태 물체 |

---

## 노드별 개별 실행 (디버그용)

```bash
# Stage 1만 실행
ros2 run collision_avoidance preprocessing_node \
    --ros-args -p input_topic:=/camera/depth/color/points

# Stage 2만 실행
ros2 run collision_avoidance ground_removal_node

# params.yaml 로드하면서 실행
ros2 run collision_avoidance preprocessing_node \
    --ros-args --params-file config/params.yaml
```

---

## 토픽 목록

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/depth/color/points` | PointCloud2 | Orbbec 원본 입력 |
| `/pointcloud/preprocessed` | PointCloud2 | 전처리 완료 |
| `/pointcloud/ground` | PointCloud2 | 검출된 바닥 (시각화) |
| `/pointcloud/objects` | PointCloud2 | 바닥 제거된 물체 |
| `/pointcloud/clusters` | PointCloud2 | 색상으로 구분된 클러스터 |
| `/collision/markers` | MarkerArray | RViz2 바운딩 박스 |