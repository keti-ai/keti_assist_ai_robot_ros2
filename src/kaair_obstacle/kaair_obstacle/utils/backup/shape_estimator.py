"""
shape_estimator.py — 깊이 카메라 기반 형태 추정

깊이 카메라는 물체의 앞면만 캡처하므로 3D PCA/단면원형도 기반 분류는
편향이 심함. 아래 세 가지 '깊이 카메라에 실제로 discriminative한' 특징을 사용:

  1. depth_variation  : 깊이 프로파일의 상대적 분산 (낮으면 flat face → cube)
  2. spherical_corr   : 중심에서 반경 r²과 깊이 z의 상관관계 (높으면 dome → sphere)
  3. circ_2d / ar_2d  : 2D 마스크의 원형도 / 종횡비 (image space)
                        원형 → sphere/cylinder-top, 직사각형 → cube/cylinder-side

[형태별 특징 프로파일]
  SPHERE   : spherical_corr 높음, circ_2d 높음, depth_variation 중간
  CYLINDER : circ_2d 높음 or ar_2d 높음(옆면), depth_variation 낮음
  CUBE     : depth_variation 낮음, circ_2d 낮음, ar_2d ≈ 1
"""

import cv2
import numpy as np
import open3d as o3d
from dataclasses import dataclass
from enum import Enum


class ShapeType(Enum):
    CUBE     = "cube"
    SPHERE   = "sphere"
    CYLINDER = "cylinder"
    UNKNOWN  = "unknown"


@dataclass
class ShapeResult:
    shape:      ShapeType
    confidence: float       # 0~1
    center:     np.ndarray  # (3,) m 단위
    dimensions: dict
    # cube:     {"width", "depth", "height"}
    # sphere:   {"radius"}
    # cylinder: {"radius", "height"}
    scores:     dict        # 디버그용 점수
    obb_R:      np.ndarray  = None  # (3,3) OBB rotation matrix (camera frame)
    main_axis:  np.ndarray  = None  # (3,)  가장 긴 축 방향 (cylinder axis)


# ──────────────────────────────────────────────
# 특징 추출
# ──────────────────────────────────────────────

def extract_features(
    points:     np.ndarray,   # (K, 3) float, camera coords (x, y, z=depth)
    seg:        np.ndarray,   # (H, W) bool  — 2D 마스크
) -> dict:
    center = points.mean(axis=0)

    # ── 1. OBB 치수 (Open3D) ──────────────────────────────────────
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    obb      = pcd.get_oriented_bounding_box()
    obb_R    = np.asarray(obb.R).copy()            # copy: Open3D 행렬은 read-only
    raw_ext  = np.asarray(obb.extent).copy()     # copy: Open3D 배열은 read-only

    # 가장 긴 축 인덱스 추적 (정렬 후에도 원래 열 매핑 유지)
    sort_idx  = np.argsort(raw_ext)[::-1]        # 큰 순 인덱스
    extents   = raw_ext[sort_idx]                # [긴, 중간, 짧은]
    main_axis = obb_R[:, sort_idx[0]]            # 가장 긴 축 방향 (카메라 프레임)

    # ── 2. 깊이 프로파일 (depth = z 축) ──────────────────────────
    z = points[:, 2]
    z_std   = float(z.std())
    z_range = float(z.max() - z.min())

    # 상대적 깊이 분산: 0=완전히 flat, 높으면 곡면
    depth_variation = z_std / (z_range + 1e-4)

    # ── 3. 구형 깊이 상관관계 ────────────────────────────────────
    # 구라면 xy 반경 r²이 클수록 z가 작음 (dome 형태)
    # Cylinder/Cube는 상관관계가 낮음
    xy  = points[:, :2] - center[:2]
    r2  = (xy ** 2).sum(axis=1)
    r2n = r2 / (r2.max() + 1e-6)
    zn  = (z - z.min()) / (z_range + 1e-6)
    # 1-r2n 과 zn의 피어슨 상관 (dome → 양의 상관)
    corr = float(np.corrcoef(1.0 - r2n, zn)[0, 1])
    spherical_corr = max(0.0, corr)              # 음수는 0 처리

    # ── 4. 2D 마스크 형태 (image space) ─────────────────────────
    vs, us = np.where(seg)
    h_2d = int(vs.max() - vs.min()) + 1
    w_2d = int(us.max() - us.min()) + 1
    ar_2d = max(h_2d, w_2d) / (min(h_2d, w_2d) + 1e-6)   # ≥1, 클수록 elongated

    # 2D 원형도 (cv2 contour 기반)
    contours, _ = cv2.findContours(
        seg.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    area_2d = float(seg.sum())
    peri_2d = float(sum(cv2.arcLength(c, True) for c in contours)) if contours else 1.0
    circ_2d = min((4.0 * np.pi * area_2d) / (peri_2d ** 2 + 1e-6), 1.0)

    return {
        "center":          center,
        "extents":         extents,
        "obb_R":           obb_R,
        "main_axis":       main_axis,
        "depth_variation": depth_variation,
        "spherical_corr":  spherical_corr,
        "ar_2d":           float(ar_2d),
        "circ_2d":         float(circ_2d),
    }


# ──────────────────────────────────────────────
# 형태 분류
# ──────────────────────────────────────────────

def classify_shape(feat: dict) -> tuple[ShapeType, float, dict]:
    dv = feat["depth_variation"]    # 0~1: 높으면 곡면
    sc = feat["spherical_corr"]     # 0~1: 높으면 dome(구)
    c2 = feat["circ_2d"]           # 0~1: 높으면 원형 마스크
    ar = feat["ar_2d"]             # ≥1: 높으면 가늘고 긴 마스크

    # elongation: 1.5 이상부터 cylinder-side로 가중
    elongation = float(np.clip((ar - 1.2) / 2.0, 0.0, 1.0))

    scores = {
        ShapeType.SPHERE: (
            sc  * 0.55   # dome 깊이 상관관계 (핵심)
            + c2 * 0.30  # 원형 마스크
            + dv * 0.15  # 어느 정도 곡면
        ),
        ShapeType.CYLINDER: (
            c2          * 0.20   # 원형 or 직사각형 마스크 모두 가능
            + elongation * 0.50  # 세워진 원통 → 길쭉한 마스크
            + dv         * 0.30  # 측면의 곡면
        ),
        ShapeType.CUBE: (
            (1.0 - dv) * 0.50    # flat한 깊이 프로파일 (핵심)
            + (1.0 - c2) * 0.30  # 비원형 마스크
            + (1.0 - elongation) * 0.20  # 길쭉하지 않음
        ),
    }

    total = sum(scores.values()) + 1e-6
    scores_norm = {k: v / total for k, v in scores.items()}
    best  = max(scores_norm, key=scores_norm.get)
    conf  = float(scores_norm[best])

    return best, conf, {k.value: round(v, 3) for k, v in scores_norm.items()}


# ──────────────────────────────────────────────
# 파라미터 피팅
# ──────────────────────────────────────────────

def fit_parameters(shape: ShapeType, feat: dict) -> dict:
    ext = feat["extents"]   # [긴, 중간, 짧은]

    if shape == ShapeType.SPHERE:
        return {"radius": float(ext.mean() / 2.0)}

    if shape == ShapeType.CYLINDER:
        return {
            "radius": float((ext[1] + ext[2]) / 4.0),
            "height": float(ext[0]),
        }

    if shape == ShapeType.CUBE:
        return {
            "width":  float(ext[2]),
            "depth":  float(ext[1]),
            "height": float(ext[0]),
        }

    return {}


# ──────────────────────────────────────────────
# 메인 진입점
# ──────────────────────────────────────────────

def estimate_shape(
    points: np.ndarray,      # (K, 3) float32/64, camera frame
    seg:    np.ndarray,      # (H, W) bool, 2D 마스크 (image space)
) -> ShapeResult:
    """
    포인트 배열 + 2D 마스크 → 형태 추정

    Args:
        points : (K, 3) float, camera frame  (z = depth in meters)
        seg    : (H, W) bool, 2D 마스크 (이미지 공간)
    """
    if len(points) < 50:
        return ShapeResult(
            shape=ShapeType.UNKNOWN, confidence=0.0,
            center=np.zeros(3), dimensions={}, scores={}
        )

    feat        = extract_features(points, seg)
    shape, conf, score_dict = classify_shape(feat)
    dims        = fit_parameters(shape, feat)

    return ShapeResult(
        shape=shape,
        confidence=conf,
        center=feat["center"],
        dimensions=dims,
        scores=score_dict,
        obb_R=feat["obb_R"],
        main_axis=feat["main_axis"],
    )
