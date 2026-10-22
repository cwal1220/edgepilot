"""모델 입력 워프의 Numpy 구현. 행렬은 src/model/model_input_transform.cc와 같다.

샘플링은 고정소수점 bilinear(가중치 12비트)다. 보드의 GDC 워프와 바이트 단위로 같지는 않지만 Y 1 LSB, U/V 평균
0.4 LSB 안이라(docs/verification.md) axmodel 보정·평가 데이터(axmodel/make_m2_data.py)가 런타임과 같은 픽셀
분포를 갖는다. 출력은 supercombo 모델이 받는 YUV6
텐서 배치다: 128x256에서 채널 [Y(2x,2y), Y(2x,2y+1), Y(2x+1,2y), Y(2x+1,2y+1), U, V].
"""

from __future__ import annotations

import numpy as np

MODEL_W, MODEL_H = 512, 256
HALF_W, HALF_H = MODEL_W // 2, MODEL_H // 2
WEIGHT_BITS = 12
WEIGHT_SCALE = 1 << WEIGHT_BITS

CAMERA_HEIGHT_M = 1.22  # src/common/model_output.h kModelHeight: 장착 높이를 따로 주지 않을 때

# 1920x1080에서의 MaixCAM2 카메라: src/common/app_config.h kCamera*(check_recording_reader.py가 같은지 본다).
MAIXCAM2_FX_1080 = 1131.24
MAIXCAM2_FY_1080 = 1130.85
MAIXCAM2_CX_1080 = 940.13
MAIXCAM2_CY_1080 = 552.60

# view_from_device: 행이 (0,1,0), (0,0,1), (1,0,0). 역행렬은 전치다.
VIEW_FROM_DEVICE = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])


def maixcam2_intrinsics(width: int, height: int) -> tuple[float, float, float, float]:
    """1920x1080에서 width x height로 스케일한 MaixCAM2 런타임 내부 파라미터."""
    return (MAIXCAM2_FX_1080 * width / 1920.0, MAIXCAM2_FY_1080 * height / 1080.0,
            MAIXCAM2_CX_1080 * width / 1920.0, MAIXCAM2_CY_1080 * height / 1080.0)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def projection_matrix(fx: float, fy: float, cx: float, cy: float,
                      rpy: np.ndarray, sbig: bool,
                      camera_offset_m: float = 0.0,
                      camera_height_m: float = CAMERA_HEIGHT_M) -> np.ndarray:
    """모델 화소 -> 원본 화소. ModelInputTransform::projection_matrix와 같은 openpilot get_warp_matrix다:
    K · view_from_device · device_from_calib(rpy) · shift · inv(K_model · view_from_device). 카메라 중심에서의
    순수 회전에, 가상 카메라를 오른쪽으로 camera_offset_m 옮긴 높이 camera_height_m의 도로면 이동(shift)을
    더한다. C++는 단정밀도로 곱하므로 상대 1e-5 안에서 같다(tests/gtest_model_input_transform.cc의 기준값)."""
    if sbig:
        model_f, model_cy = 455.0, 0.5 * (256.0 + 47.6)  # sbigmodel_fl, 가운데
    else:
        model_f, model_cy = 910.0, 47.6                  # medmodel_fl, MEDMODEL_CY
    model_cx = 256.0
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    model_k_inv = np.array([[1.0 / model_f, 0.0, -model_cx / model_f],
                            [0.0, 1.0 / model_f, -model_cy / model_f],
                            [0.0, 0.0, 1.0]])
    rot = rotation_from_rpy(*np.asarray(rpy, dtype=np.float64))
    shift = np.eye(3)
    shift[1, 2] = camera_offset_m / camera_height_m
    projection = k @ VIEW_FROM_DEVICE @ rot @ shift @ VIEW_FROM_DEVICE.T @ model_k_inv
    return projection.astype(np.float32)


def _scale_transform(projection: np.ndarray, scale: float) -> np.ndarray:
    t_out = np.array([[1.0 / scale, 0, 0.5], [0, 1.0 / scale, 0.5], [0, 0, 1]],
                     dtype=np.float32)
    t_in = np.array([[scale, 0, -0.5 * scale], [0, scale, -0.5 * scale], [0, 0, 1]],
                    dtype=np.float32)
    return t_in @ projection @ t_out


class _PlaneMap:
    """평면 하나의 미리 계산한 gather 인덱스 + 고정소수점 bilinear 가중치."""

    def __init__(self, projection: np.ndarray, src_w: int, src_h: int,
                 dst_scale: int, dst_x_offset: int, dst_y_offset: int):
        xs = np.arange(HALF_W, dtype=np.float32) * dst_scale + dst_x_offset
        ys = np.arange(HALF_H, dtype=np.float32) * dst_scale + dst_y_offset
        gx, gy = np.meshgrid(xs, ys)
        x0 = projection[0, 0] * gx + projection[0, 1] * gy + projection[0, 2]
        y0 = projection[1, 0] * gx + projection[1, 1] * gy + projection[1, 2]
        w0 = projection[2, 0] * gx + projection[2, 1] * gy + projection[2, 2]
        valid = np.abs(w0) > 1e-6
        with np.errstate(divide="ignore", invalid="ignore"):
            sx = np.where(valid, x0 / w0, 0.0)
            sy = np.where(valid, y0 / w0, 0.0)
        ix = np.floor(sx).astype(np.int64)
        iy = np.floor(sy).astype(np.int64)
        ix[~valid] = -2
        iy[~valid] = -2
        ax = (sx - ix).astype(np.float32)
        ay = (sy - iy).astype(np.float32)
        weights = np.stack([(1 - ax) * (1 - ay), ax * (1 - ay),
                            (1 - ax) * ay, ax * ay])
        corner_x = np.stack([ix, ix + 1, ix, ix + 1])
        corner_y = np.stack([iy, iy, iy + 1, iy + 1])
        in_bounds = ((corner_x >= 0) & (corner_x < src_w) &
                     (corner_y >= 0) & (corner_y < src_h))
        weights = np.clip(np.round(weights * WEIGHT_SCALE), 0, WEIGHT_SCALE)
        weights[~in_bounds | ~valid[None]] = 0
        self.weights = weights.astype(np.uint32)

        base_x = np.clip(ix, 0, src_w - 1)
        base_y = np.clip(iy, 0, src_h - 1)
        x_step = np.where((ix >= 0) & (ix + 1 < src_w), 1, 0)
        y_step = np.where((iy >= 0) & (iy + 1 < src_h), src_w, 0)
        base = base_y * src_w + base_x
        self.indices = np.stack([base, base + x_step, base + y_step,
                                 base + y_step + x_step]).astype(np.int64)

    def sample(self, plane_flat: np.ndarray) -> np.ndarray:
        pixels = plane_flat[self.indices].astype(np.uint32)
        acc = (pixels * self.weights).sum(axis=0, dtype=np.uint32)
        return np.minimum((acc + WEIGHT_SCALE // 2) >> WEIGHT_BITS, 255).astype(np.uint8)


class ModelInputWarp:
    """보정을 고정한 모델 프레임 워프 하나(medmodel 또는 sbigmodel)."""

    def __init__(self, src_w: int, src_h: int, rpy: np.ndarray, sbig: bool,
                 camera_offset_m: float = 0.0, camera_height_m: float = CAMERA_HEIGHT_M):
        fx, fy, cx, cy = maixcam2_intrinsics(src_w, src_h)
        proj_y = projection_matrix(fx, fy, cx, cy, rpy, sbig, camera_offset_m, camera_height_m)
        proj_uv = _scale_transform(proj_y, 0.5)
        self.src_w, self.src_h = src_w, src_h
        self.rpy = np.asarray(rpy, dtype=np.float32).copy()
        self.mount = (float(camera_offset_m), float(camera_height_m))
        offsets = [(0, 0), (0, 1), (1, 0), (1, 1)]
        self.y_maps = [_PlaneMap(proj_y, src_w, src_h, 2, ox, oy)
                       for ox, oy in offsets]
        self.uv_map = _PlaneMap(proj_uv, src_w // 2, src_h // 2, 1, 0, 0)

    def warp(self, y: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """(720p 평면 YUV420) -> uint8 [6, 128, 256]."""
        out = np.empty((6, HALF_H, HALF_W), dtype=np.uint8)
        y_flat = np.ascontiguousarray(y).reshape(-1)
        for ch, plane_map in enumerate(self.y_maps):
            out[ch] = plane_map.sample(y_flat)
        out[4] = self.uv_map.sample(np.ascontiguousarray(u).reshape(-1))
        out[5] = self.uv_map.sample(np.ascontiguousarray(v).reshape(-1))
        return out


class WarpPair:
    """보정이 바뀔 때까지 캐시해 두는 med + sbig 워프."""

    def __init__(self, src_w: int, src_h: int):
        self.src_w, self.src_h = src_w, src_h
        self._med: ModelInputWarp | None = None
        self._sbig: ModelInputWarp | None = None

    def set_calibration(self, rpy: np.ndarray, camera_offset_m: float = 0.0,
                        camera_height_m: float = CAMERA_HEIGHT_M):
        """보정(rpy)과 카메라 장착(modeld set_camera_mount: 오른쪽 이동, 높이). 녹화된 ModelState에 둘 다 있다."""
        rpy = np.asarray(rpy, dtype=np.float32)
        mount = (float(camera_offset_m), float(camera_height_m))
        if (self._med is None or not np.allclose(self._med.rpy, rpy, atol=1e-7)
                or not np.allclose(self._med.mount, mount, atol=1e-5)):
            self._med = ModelInputWarp(self.src_w, self.src_h, rpy, sbig=False,
                                       camera_offset_m=mount[0], camera_height_m=mount[1])
            self._sbig = ModelInputWarp(self.src_w, self.src_h, rpy, sbig=True,
                                        camera_offset_m=mount[0], camera_height_m=mount[1])

    def warp(self, y, u, v) -> tuple[np.ndarray, np.ndarray]:
        if self._med is None:
            raise RuntimeError("set_calibration() before warp()")
        return self._med.warp(y, u, v), self._sbig.warp(y, u, v)
