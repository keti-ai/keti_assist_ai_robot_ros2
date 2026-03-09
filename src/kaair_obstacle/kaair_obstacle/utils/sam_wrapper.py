import torch
import numpy as np
import cv2
from pathlib import Path

import sam2 as _sam2_pkg
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

# SAM2 소스 루트: .../third_party/sam2/  (checkpoints/ 디렉토리 포함)
_SAM2_SRC_ROOT = Path(_sam2_pkg.__file__).resolve().parent.parent


class Sam2MaskWrapper:
    def __init__(self, node):
        self._node = node
        self.device = self._set_device()
        self.mask_generator = self._set_model()

        self._node.get_logger().info(f'SAM2 uploaded successfully')

    @staticmethod
    def _set_device():
        if torch.cuda.is_available():
            device = torch.device("cuda")
        elif torch.backends.mps.is_available():
            device = torch.device("mps")
        else:
            device = torch.device("cpu")

        if device.type == "cuda":
            # use bfloat16 for the entire notebook
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True

        return device

    def _set_model(self):
        # 체크포인트: SAM2 소스 루트의 checkpoints/ 디렉토리
        ckpt = str(_SAM2_SRC_ROOT / 'checkpoints' / self._node.cfg.sam2_model)

        # config: initialize_config_module("sam2") 기준 상대 경로
        # sam2/__init__.py 가 "sam2" 모듈을 Hydra 루트로 등록하므로
        # sam2/configs/sam2.1/xxx.yaml -> 'configs/sam2.1/xxx.yaml'
        cfg = f'configs/sam2.1/{self._node.cfg.sam2_cfg}'

        # self._node.get_logger().info(f'SAM2 ckpt: {ckpt}')
        # self._node.get_logger().info(f'SAM2 cfg:  {cfg}')

        sam2 = build_sam2(cfg, ckpt, self.device, apply_postprocessing=False)
        
        mask_generator = SAM2AutomaticMaskGenerator(
            model=sam2,
            points_per_side=32,          # 유지 (충분)
            points_per_batch=64,         # 유지
            pred_iou_thresh=0.7,        # ↑ 0.7→0.88: 품질 낮은 작은 마스크 제거
            stability_score_thresh=0.95, # ↑ 0.92→0.95: 경계 불안정한 마스크 제거
            stability_score_offset=0.7,  # 유지
            crop_n_layers=0,             # ↓ 4→0: crop 자체를 끔 (crop이 작은 물체 검출의 주범)
            box_nms_thresh=0.2,          # ↓ 0.6→0.4: 겹치는 마스크 더 적극 제거
            crop_n_points_downscale_factor=0,  # crop=0이므로 무의미
            min_mask_region_area=50000.0, # ↑ 25→2000: 작은 마스크 후처리에서 제거
            use_m2m=True,                # 유지
        )
        return mask_generator

    def filter_contained_masks(self, masks, containment_thresh=0.6):
        """
        다른 마스크 안에 포함된 마스크를 제거
        
        containment_thresh: 한 마스크가 다른 마스크 안에 
                            이 비율 이상 포함되면 제거 (0.8 = 80%)
        """
        if len(masks) == 0:
            return masks

        # 면적 큰 순으로 정렬
        masks = sorted(masks, key=lambda x: x["area"], reverse=True)
        
        seg_array = np.array([m["segmentation"] for m in masks])  # (N, H, W)
        keep = np.ones(len(masks), dtype=bool)

        for i in range(len(masks)):
            if not keep[i]:
                continue
            for j in range(i + 1, len(masks)):
                if not keep[j]:
                    continue
                
                # j가 i 안에 얼마나 포함되는지
                intersection = np.logical_and(seg_array[i], seg_array[j]).sum()
                j_area = seg_array[j].sum()
                
                containment = intersection / j_area  # j 중 i와 겹치는 비율
                
                if containment >= containment_thresh:
                    keep[j] = False  # j는 i 안에 포함됨 → 제거

        return [m for m, k in zip(masks, keep) if k]

    @staticmethod
    def dilate_masks(masks, dilate_px=5):
        """
        모든 마스크를 (H, W, N) uint8로 쌓아 cv2.dilate 1번 호출로 처리.

        [기존 루프 방식 vs 벡터화]
          루프  : N × (astype + dilate + astype + sum) → N=20 기준 ~40-80ms
          벡터화: stack + dilate(1회) + astype + sum(axis) → ~5-15ms (4-8×)

        cv2.dilate는 멀티채널 이미지 지원 (최대 512채널).
        N > 512 이면 자동으로 512채널씩 청크 처리.
        """
        if not masks:
            return masks

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * dilate_px + 1, 2 * dilate_px + 1)
        )
        N = len(masks)
        CHUNK = 512   # cv2 최대 채널 수

        # (H, W, N) uint8 - bool 배열을 C-contiguous로 쌓기
        segs = np.stack(
            [m['segmentation'] for m in masks], axis=-1
        ).astype(np.uint8, copy=False)          # (H, W, N)

        # 512채널 초과 시 청크 분할
        if N <= CHUNK:
            dilated = cv2.dilate(segs, kernel)  # (H, W, N)
        else:
            dilated = np.empty_like(segs)
            for s in range(0, N, CHUNK):
                dilated[:, :, s:s+CHUNK] = cv2.dilate(segs[:, :, s:s+CHUNK], kernel)

        dilated_bool = dilated.astype(bool)     # cv2 출력이 read-only일 수 있어 copy
        areas = dilated_bool.sum(axis=(0, 1))   # (N,) — 한 번에 모든 area 계산

        for i, mask in enumerate(masks):
            mask['segmentation'] = dilated_bool[:, :, i]
            mask['area'] = int(areas[i])

        return masks

    def get_masks(self, image, dilate_masks=False, dilate_px=3, show_result_img=False):
        try:
            masks = self.mask_generator.generate(image)
            self._node.get_logger().info(f'Masks: {len(masks)}')
            masks = self.filter_contained_masks(masks)

            if dilate_masks:
                masks = self.dilate_masks(masks, dilate_px)

            if show_result_img:
                result_img = self.show_anns(masks, image, borders=False)
            else:
                result_img = None
            return masks, result_img
        except Exception as e:
            self._node.get_logger().error(f'Error in get_masks: {e}')
            return None, None

    @staticmethod
    def show_anns(anns, base_image=None, mask_alpha=0.5, borders=True):
        """
        마스크를 원본 이미지에 alpha 블렌딩으로 오버레이.

        base_image : (H, W, 3) uint8 BGR  — None이면 흰 배경 사용
        mask_alpha : 마스크 색상 불투명도 (0.0~1.0)
        반환       : (H, W, 3) uint8 BGR
        """
        if not anns:
            return base_image

        sorted_anns = sorted(anns, key=lambda x: x['area'], reverse=True)
        H, W = sorted_anns[0]['segmentation'].shape

        # 원본을 float32 [0,1]로 정규화 (없으면 흰 배경)
        if base_image is not None:
            canvas = base_image[:H, :W].astype(np.float32) / 255.0
        else:
            canvas = np.ones((H, W, 3), dtype=np.float32)

        # 마스크 색상 레이어: (H, W, 3) float32
        color_layer = np.zeros((H, W, 3), dtype=np.float32)
        alpha_layer = np.zeros((H, W),    dtype=np.float32)   # 픽셀별 alpha

        for ann in sorted_anns:
            m = ann['segmentation']                            # (H, W) bool
            color = np.random.random(3).astype(np.float32)    # RGB 랜덤 색
            color_layer[m] = color                            # BGR 순서 상관없음 (시각화용)
            alpha_layer[m] = mask_alpha

            if borders:
                contours, _ = cv2.findContours(
                    m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                )
                contours = [
                    cv2.approxPolyDP(c, epsilon=0.5, closed=True) for c in contours
                ]
                # 테두리: 흰색 선, 완전 불투명
                cv2.drawContours(color_layer, contours, -1, (1.0, 1.0, 1.0), thickness=1)
                cv2.drawContours(alpha_layer,  contours, -1, 1.0,             thickness=1)

        # Alpha compositing: out = canvas * (1 - alpha) + color * alpha
        a = alpha_layer[:, :, np.newaxis]                     # (H, W, 1) 브로드캐스트용
        composited = canvas * (1.0 - a) + color_layer * a

        return (np.clip(composited, 0.0, 1.0) * 255).astype(np.uint8)