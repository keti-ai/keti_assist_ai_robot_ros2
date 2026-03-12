# kaair_obstacle — Obstacle Representation Node

Orbbec 깊이 카메라의 PointCloud2를 수신하여 ROI 필터링 및 Voxel 다운샘플링을 거친 뒤,  
RViz2 시각화용 **MarkerArray**와 MoveIt2 플래닝 씬용 **CollisionObject**를 동시에 퍼블리시합니다.

---

## 파이프라인

```
Orbbec PointCloud2  (BEST_EFFORT QoS)
        │  input_topic  (기본: /femto/depth_registered/points)
        ▼
┌──────────────────────────┐
│  1. 전처리               │  ROI PassThrough + NaN/Inf 제거
│     np.frombuffer 직접   │  Python 루프 없음 → 고속
└────────────┬─────────────┘
             │  /pointcloud/preprocessed  (PointCloud2)
             ▼
┌──────────────────────────┐
│  2. Voxel 다운샘플링      │  Open3D voxel_down_sample (C++)
└────────────┬─────────────┘
             │  /pointcloud/downsampled   (PointCloud2)
             ├──────────────────────────────────────────────────────┐
             ▼                                                      ▼
┌──────────────────────────┐                        ┌──────────────────────────┐
│  3a. Shape Markers        │  SPHERE_LIST / CUBE_LIST│  3b. CollisionObject     │  SPHERE / BOX
│      RViz2 시각화         │                        │      MoveIt2 플래닝 씬   │
└────────────┬─────────────┘                        └────────────┬─────────────┘
             │  /obstacle_spheres (MarkerArray)                  │  /collision_field (CollisionObject)
             ▼                                                    ▼
           RViz2                                              MoveIt2
```

---

## 퍼블리시 / 서브스크라이브 토픽

| 방향 | 토픽 | 메시지 타입 | 설명 |
|------|------|------------|------|
| Subscribe | `input_topic` | `sensor_msgs/PointCloud2` | Orbbec 원본 포인트클라우드 (BEST_EFFORT QoS) |
| Publish | `/pointcloud/preprocessed` | `sensor_msgs/PointCloud2` | ROI 필터 + NaN 제거 후 |
| Publish | `/pointcloud/downsampled` | `sensor_msgs/PointCloud2` | Voxel 다운샘플 후 |
| Publish | `/obstacle_spheres` | `visualization_msgs/MarkerArray` | RViz2 시각화용 마커 (SPHERE_LIST 또는 CUBE_LIST) |
| Publish | `/collision_field` | `moveit_msgs/CollisionObject` | MoveIt2 플래닝 씬 장애물 |

---

## 파라미터

`config/params.yaml` 또는 런타임 `ros2 param set`으로 변경 가능합니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `input_topic` | `/camera/depth/color/points` | 구독할 PointCloud2 토픽 |
| `roi_x_min` / `roi_x_max` | `-2.0` / `2.0` | ROI X 범위 (m, 카메라 기준 좌우) |
| `roi_y_min` / `roi_y_max` | `-1.5` / `1.5` | ROI Y 범위 (m, 카메라 기준 상하) |
| `roi_z_min` / `roi_z_max` | `0.1` / `4.0` | ROI Z 범위 (m, 카메라 기준 전방 깊이) |
| `voxel_size` | `0.05` | Voxel 한 변 크기 (m). 작을수록 정밀하지만 느림 |
| `marker_shape` | `cube` | 마커·CollisionObject 형태: `cube` 또는 `sphere` |

### `voxel_size` 조절 가이드

| 값 | 특징 |
|----|------|
| `0.02` (2 cm) | 세밀, 소형 물체 검출. 포인트 수가 많아 느릴 수 있음 |
| `0.05` (5 cm) | 실내 일반 장애물 기본값 |
| `0.10` (10 cm) | 원거리·실시간 우선 시 |

### `marker_shape` 비교

| 값 | RViz2 Marker 타입 | CollisionObject 타입 | 비고 |
|----|------------------|----------------------|------|
| `cube` | `CUBE_LIST` | `BOX` (한 변 = voxel_size) | 기본값, 플래닝 씬 보수적 표현 |
| `sphere` | `SPHERE_LIST` | `SPHERE` (반지름 = voxel_size / 2) | 유연한 곡면 환경 |

---

## 의존성

### Python
```bash
pip install open3d numpy
```

### ROS2 패키지 (`package.xml`에 이미 선언됨)
```
rclpy  sensor_msgs  visualization_msgs  geometry_msgs  std_msgs
moveit_msgs  shape_msgs  tf2_ros  tf2_geometry_msgs
```

---

## 빌드 & 실행

```bash
# 빌드
cd /ros_ws
colcon build --packages-select kaair_obstacle
source install/setup.bash

# Launch 파일로 실행 (params.yaml 자동 로드)
ros2 launch kaair_obstacle obstacle_representation.launch.py

# 직접 실행 + 파라미터 오버라이드
ros2 run kaair_obstacle obstacle_representation \
    --ros-args \
    --params-file src/kaair_obstacle/config/params.yaml \
    -p voxel_size:=0.05 \
    -p marker_shape:=sphere
```

### 입력 토픽 확인 (Orbbec)
```bash
ros2 topic list | grep point
# 출력 예: /femto/depth_registered/points
```

`params.yaml`의 `input_topic`을 실제 토픽명으로 수정하거나 리매핑을 사용합니다:
```bash
ros2 launch kaair_obstacle obstacle_representation.launch.py \
    # launch 파일 내 remappings 수정 또는
ros2 run kaair_obstacle obstacle_representation \
    --ros-args -r /femto/depth_registered/points:=/your/actual/topic
```

---

## MoveIt2 연동

노드가 퍼블리시하는 `/collision_field` 토픽을 MoveIt2의 `PlanningSceneMonitor`가 구독하면  
장애물이 플래닝 씬에 자동으로 반영됩니다.

`move_group` 노드 파라미터에서 `CollisionObject` 구독이 활성화되어 있는지 확인하세요:

```yaml
# move_group 파라미터 (기본값으로 이미 활성화되어 있음)
planning_scene_monitor_options:
  publish_planning_scene: true
  publish_geometry_updates: true
  publish_state_updates: true
```

> **TF 주의**: `collision_field`의 `header.frame_id`는 카메라 프레임(예: `camera_depth_optical_frame`)입니다.  
> MoveIt2 플래닝 씬과 카메라 프레임 사이의 TF 변환이 브로드캐스트되어야 장애물 위치가 정확하게 반영됩니다.

---

## RViz2 시각화

| 추가 항목 | 타입 | 설명 |
|-----------|------|------|
| `/pointcloud/preprocessed` | PointCloud2 | ROI 필터 적용 원본 |
| `/pointcloud/downsampled` | PointCloud2 | Voxel 격자 결과 |
| `/obstacle_spheres` | MarkerArray | 장애물 마커 (구 또는 정육면체) |

1. RViz2 → **Fixed Frame**을 카메라 `frame_id`로 설정
2. **Add → By Topic**으로 위 토픽들 추가

---

## 파일 구조

```
kaair_obstacle/
├── kaair_obstacle/
│   ├── obstacle_representation.py   # 메인 ROS2 노드
│   └── utils/
│       ├── process.py               # 파이프라인 처리 (전처리·다운샘플·마커·CollisionObject)
│       ├── load_config.py           # ROS2 파라미터 → PipelineConfig 로드
│       └── convert_pointcloud.py   # numpy ↔ PointCloud2 변환
├── config/
│   └── params.yaml                  # 런타임 파라미터 설정
└── launch/
    └── obstacle_representation.launch.py
```
