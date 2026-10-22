#!/usr/bin/env python3
"""HUD 스냅샷 도우미.

  hud_tools.py inputs <route_dir> <out_dir> [--time SECONDS]
      recordd route에서 hud_snapshot 입력을 뽑는다: model.bin(ModelState 원본), control.bin(꼬리 패딩까지
      240바이트로 채운 ControlState), camera.png(맞는 도로 프레임).
      --time이 없으면 순간을 자동으로 고른다: 컨트롤러 활성, 40~95 km/h, 차선이 모두 확실하고, 있으면
      앞차도 있는 때.

  hud_tools.py compose <prefix> [camera.png]
      hud_snapshot의 EDGEARGB 프레임(<prefix>_<scenario>.argb, 640x480)을 PNG로 바꾼다:
      <name>_overlay.png(투명)와, 카메라 프레임이 있으면 화면처럼 프레임 가운데 4:3 위에 겹친
      <name>_composite.png.
"""
from __future__ import annotations

import argparse
import glob
import struct
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from recording_reader import (CONTROL_STATE_SIZE, RECORD_CONTROL_STATE, RECORD_MODEL_STATE, RouteVideo,  # noqa: E402
                              chunk_states, concat_chunks, route_event_files)

FRAME_MAGIC = b"EDGEARGB"


# ---- 입력 ----

def scan(route: Path) -> tuple[list[Path], dict[str, np.ndarray], dict[str, np.ndarray]]:
    """이벤트 청크들, 제어 상태(머리 시각과 레코드), 모델 순간들(머리 시각, 청크 번호와 그 안의 줄, frame_id, 차선 확률,
    앞차). 고른 순간의 ModelState 원본은 그 청크에서 다시 읽는다: route 전체의 ModelState는 메모리에 담기에 크다."""
    paths = route_event_files(route)
    controls, models = [], []
    for n, path in enumerate(paths):
        chunk = chunk_states(path, RECORD_CONTROL_STATE, RECORD_MODEL_STATE)
        control_ts, control = chunk[RECORD_CONTROL_STATE]
        model_ts, model = chunk[RECORD_MODEL_STATE]
        controls.append({"t": control_ts, "state": control})
        models.append({"t": model_ts, "chunk": np.full(len(model), n), "row": np.arange(len(model)),
                       "frame_id": model["frame_id"].astype(np.int64),
                       "lanes": model["lane_probabilities"].astype(float),
                       "lead_valid": model["lead"]["valid"].astype(np.int64)})
    return paths, concat_chunks(controls), concat_chunks(models)


def score(control: np.ndarray, lanes: np.ndarray, lead_valid: np.ndarray) -> np.ndarray:
    """순간마다의 점수: 컨트롤러 활성, 40~95 km/h가 아니면 -1, 그 밖에는 차선 확률과 앞차·TPMS가 있으면 더한다."""
    value = 1.0 + 2.0 * np.minimum(lanes[:, 1], lanes[:, 2]) + 0.5 * (lanes[:, 0] + lanes[:, 3])
    value += np.where(control["radar_lead_valid"] != 0, 1.5, 0.0)
    value += np.where(lead_valid != 0, 1.0, 0.0)
    value += np.where(control["tpms_valid"] != 0, 0.5, 0.0)
    speed = control["cluster_speed_kph"].astype(float)
    return np.where((control["active"] == 1) & (speed >= 40.0) & (speed <= 95.0), value, -1.0)


def cmd_inputs(args: argparse.Namespace) -> int:
    paths, controls, models = scan(args.route)
    if not len(controls["t"]) or not len(models["t"]):
        sys.exit("route has no control/model records")
    control_ts, model_ts = controls["t"], models["t"]

    if args.time is not None:
        target = int(model_ts[0]) + int(args.time * 1e9)
        mi = int(np.clip(np.searchsorted(model_ts, target), 0, len(model_ts) - 1))
    else:
        candidates = np.arange(0, len(model_ts), 3)
        j = np.searchsorted(control_ts, model_ts[candidates])
        candidates, j = candidates[j < len(control_ts)], j[j < len(control_ts)]
        values = score(controls["state"][j], models["lanes"][candidates], models["lead_valid"][candidates])
        mi = int(candidates[np.argmax(values)])
    cj = min(int(np.searchsorted(control_ts, model_ts[mi])), len(control_ts) - 1)
    control = controls["state"][cj]
    frame_id, lanes, lead_valid = int(models["frame_id"][mi]), models["lanes"][mi], int(models["lead_valid"][mi])

    args.out.mkdir(parents=True, exist_ok=True)
    _, chunk_models = chunk_states(paths[models["chunk"][mi]], RECORD_MODEL_STATE)[RECORD_MODEL_STATE]
    (args.out / "model.bin").write_bytes(chunk_models[models["row"][mi]].tobytes())
    raw = control.tobytes()
    (args.out / "control.bin").write_bytes(raw + b"\0" * (CONTROL_STATE_SIZE - len(raw)))
    seconds = (int(model_ts[mi]) - int(model_ts[0])) / 1e9
    print(f"moment t={seconds:.1f}s frame_id={frame_id} speed={control['cluster_speed_kph']:.1f} "
          f"active={control['active']} lanes={np.round(lanes, 2)} lead={lead_valid}")

    video = RouteVideo(args.route)
    rgb = video.frame_rgb(frame_id)
    if rgb is None:
        print("camera frame not found in frames.bin", file=sys.stderr)
        return 0
    camera = args.out / "camera.png"
    Image.fromarray(rgb).save(camera)
    segment, index = video.where[frame_id]
    print(f"camera frame: {video.segments[segment].path.name} index {index} -> {camera}")
    return 0


# ---- 합성 ----

def load_frame(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:8] != FRAME_MAGIC:
        raise ValueError(f"{path}: not an EDGEARGB file")
    width, height = struct.unpack_from("<II", data, 8)
    pixels = np.frombuffer(data, np.uint8, count=width * height * 4, offset=16)
    return pixels.reshape(height, width, 4)[..., [2, 1, 0, 3]].copy()  # BGRA를 RGBA로


def screen_camera(path: Path, size: tuple[int, int]) -> Image.Image:
    """overlayd가 보여 주는 카메라 프레임: 가운데 4:3을 화면 크기로 늘린다."""
    image = Image.open(path).convert("RGBA")
    crop_w = image.height * 4 // 3
    left = (image.width - crop_w) // 2
    return image.crop((left, 0, left + crop_w, image.height)).resize(size, Image.BILINEAR)


def cmd_compose(args: argparse.Namespace) -> int:
    frames = sorted(glob.glob(f"{args.prefix}_*.argb"))
    if not frames:
        print(f"no frames matching {args.prefix}_*.argb", file=sys.stderr)
        return 1
    camera = None
    for frame in frames:
        path = Path(frame)
        rgba = load_frame(path)
        overlay = Image.fromarray(rgba, "RGBA")
        overlay.save(path.with_name(path.stem + "_overlay.png"))
        coverage = (rgba[..., 3] > 0).mean() * 100.0
        line = f"{path.stem}: overlay coverage {coverage:.1f}%"
        if args.camera is not None:
            camera = camera or screen_camera(args.camera, overlay.size)
            composite = camera.copy()
            composite.alpha_composite(overlay)
            composite.convert("RGB").save(path.with_name(path.stem + "_composite.png"))
            line += " (+composite)"
        print(line)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    inputs = commands.add_parser("inputs")
    inputs.add_argument("route", type=Path)
    inputs.add_argument("out", type=Path)
    inputs.add_argument("--time", type=float, help="seconds from route start instead of auto-pick")
    inputs.set_defaults(run=cmd_inputs)
    compose = commands.add_parser("compose")
    compose.add_argument("prefix")
    compose.add_argument("camera", nargs="?", type=Path)
    compose.set_defaults(run=cmd_compose)
    args = parser.parse_args()
    return args.run(args)


if __name__ == "__main__":
    sys.exit(main())
