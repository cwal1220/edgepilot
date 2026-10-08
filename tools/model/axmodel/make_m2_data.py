"""Calibration and evaluation data for the master core from MaixCAM2 recordings.

Every sample is exactly what the runtime feeds the core on the board: the recorded H.264 frame
(1280x720) through the device warp (tools/model/model_warp.py, MaixCAM2 intrinsics from
src/common/app_config.h, the calibration the recording carries), the 5-frame image history per tower
(t-4 and t), the desire pulses controlsd sent (src/model/model_temporal.h: rising edge, 100 ticks
max-pooled to 25), and the core's own 96-tick hidden-state history from running the fp32 core in order.

  make_m2_data.py calib <spec.json> <out_dir>
      Pulsar2 calibration set (<out_dir>/<tensor>.tar of NNNN.npy, float32 with a batch axis).
      spec: {"core": core_fp32.onnx, "routes_dir": dir with <route>/events,
             "video_dir": dir with <route>/<seg>/road.h264 + frames.bin,
             "groups": [[route, [seg, ...]], ...]   contiguous segments, decoded as one stream,
             "exclude": {route: [t, ...]}             leave out t-12..t+5 s (evaluation moments),
             "dense": {route: [[a, b], ...]},         sample densely here (light changes),
             "stride": 80, "dense_stride": 5}         frames between samples
      Every 4th sample also gets one synthetic desire pulse so the desire range stays [0, 1].

  make_m2_data.py eval <spec.json> <route> <seg>[,<seg>...] <from> <to> <out_dir>
      Exact inputs of every frame in [from, to] (route seconds from the first ControlState) after
      an 8 s warm-up, as frame-major .npy for run_axmodel_assembled.py on the board, plus
      the fp32 outputs for the same inputs (ref.npy) and frame times (t.npy).
"""
import io
import json
import sys
import tarfile
from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recording_reader as rr  # noqa: E402
from model_warp import WarpPair, maixcam2_intrinsics  # noqa: E402

# src/common/app_config.h: MaixCAM2 1920x1080 intrinsics scaled to the 1280x720 AI stream
INTRINSICS = maixcam2_intrinsics(1280, 720)
FEAT = 1064
TENSORS = ("input_imgs", "big_input_imgs", "desire", "features_buffer", "traffic_convention")


def recorded(route_dir: Path):
    """ControlState (time, desire) and the calibration of every ModelState."""
    cs_t, cs_des, m_t, rpy = [], [], [], []
    for path in rr.route_event_files(route_dir):
        for rec in rr.iter_event_records(path):
            if rec.type == rr.RECORD_CONTROL_STATE:
                c = rec.control_state()
                cs_t.append(c["timestamp_ns"] * 1e-9)
                cs_des.append(int(c["desire"]))
            elif rec.type == rr.RECORD_MODEL_STATE:
                lay = rec.model_layout()
                m_t.append(np.frombuffer(rec.payload, "<u8", 1, lay["capture_timestamp_ns"])[0] * 1e-9)
                rpy.append(np.frombuffer(rec.payload, "<f4", 3, lay["calibration"] + 8).copy())
    return np.array(cs_t), np.array(cs_des), np.array(m_t), np.array(rpy)


def run_stream(sess, route_dir: Path, seg_dirs: list[Path]):
    """Yield (route time, feed, raw output) for every frame of a contiguous run of segments."""
    cs_t, cs_des, m_t, rpy_all = recorded(route_dir)
    t_first = cs_t[0]
    segments = [rr.read_segment_index(d) for d in seg_dirs]
    t0 = segments[0].frames["capture_timestamp_ns"][0] * 1e-9 - t_first
    t1 = segments[-1].frames["capture_timestamp_ns"][-1] * 1e-9 - t_first
    sel = (m_t - t_first >= t0 - 30) & (m_t - t_first <= t1)
    warp = WarpPair(1280, 720, INTRINSICS)
    warp.set_calibration(np.median(rpy_all[sel], axis=0))
    zero = np.zeros((6, 128, 256), np.uint8)
    road, wide = deque([zero] * 5, maxlen=5), deque([zero] * 5, maxlen=5)
    feat = np.zeros((96, 512), np.float32)
    desire = np.zeros((100, 8), np.float32)
    prev = np.zeros(8, np.float32)
    for rec, y, u, v in rr.decode_route_yuv(segments):
        t = rec["capture_timestamp_ns"] * 1e-9 - t_first
        med, sbig = warp.warp(y, u, v)
        road.append(med)
        wide.append(sbig)
        j = min(np.searchsorted(cs_t - t_first, t), len(cs_t) - 1)
        cur = np.zeros(8, np.float32)
        if cs_des[j] > 0:
            cur[cs_des[j]] = 1.0
        pulse = np.where(cur - prev > 0.99, cur, 0.0).astype(np.float32)
        prev = cur
        desire = np.roll(desire, -1, 0)
        desire[-1] = pulse
        feed = {
            "input_imgs": np.concatenate([road[0], road[-1]])[None].astype(np.float32),
            "big_input_imgs": np.concatenate([wide[0], wide[-1]])[None].astype(np.float32),
            "desire": desire.reshape(25, 4, 8).max(1)[None],
            "features_buffer": feat[0::4][None].copy(),
            "traffic_convention": np.array([[1.0, 0.0]], np.float32),
        }
        out = sess.run(None, feed)[0][0]
        yield t, feed, out
        feat = np.roll(feat, -1, 0)
        feat[-1] = out[FEAT:FEAT + 512]


def session(core: str):
    opts = ort.SessionOptions()
    opts.log_severity_level = 3
    return ort.InferenceSession(str(Path(core).expanduser()), opts)


def cmd_calib(spec: dict, out_dir: Path):
    sess = session(spec["core"])
    out_dir.mkdir(parents=True, exist_ok=True)
    tars = {k: tarfile.open(out_dir / f"{k}.tar", "w") for k in TENSORS}
    rng = np.random.default_rng(0)
    stride, dense_stride, count = spec.get("stride", 80), spec.get("dense_stride", 5), 0
    for route, segs in spec["groups"]:
        seg_dirs = [Path(spec["video_dir"]).expanduser() / route / s for s in segs]
        n, kept, start = 0, 0, None
        for t, feed, _ in run_stream(sess, Path(spec["routes_dir"]).expanduser() / route, seg_dirs):
            start = t if start is None else start
            n += 1
            dense = any(a <= t <= b for a, b in spec.get("dense", {}).get(route, []))
            excluded = any(g - 12.0 <= t <= g + 5.0 for g in spec.get("exclude", {}).get(route, []))
            if t - start < 5.0 or excluded or n % (dense_stride if dense else stride):
                continue
            if count % 4 == 1:  # 합성 desire 펄스: desire 범위를 [0, 1]로 유지한다
                d = rng.choice([1, 2, 3, 4, 5, 6], p=[0.1, 0.1, 0.3, 0.3, 0.1, 0.1])
                feed["desire"] = feed["desire"].copy()
                feed["desire"][0, rng.integers(0, 25), d] = 1.0
            for k in TENSORS:
                buf = io.BytesIO()
                np.save(buf, feed[k].astype(np.float32))
                info = tarfile.TarInfo(f"{count:04d}.npy")
                info.size = buf.tell()
                buf.seek(0)
                tars[k].addfile(info, buf)
            count += 1
            kept += 1
        print(f"{route} {','.join(segs)}: {n} frames, kept {kept}, total {count}", flush=True)
    for t in tars.values():
        t.close()


def cmd_eval(spec: dict, route: str, segs: str, t_from: float, t_to: float, out_dir: Path):
    sess = session(spec["core"])
    seg_dirs = [Path(spec["video_dir"]).expanduser() / route / s for s in segs.split(",")]
    keep = {k: [] for k in TENSORS}
    ts, refs = [], []
    for t, feed, out in run_stream(sess, Path(spec["routes_dir"]).expanduser() / route, seg_dirs):
        if t > t_to:
            break
        if t < t_from:
            continue
        for k in TENSORS:
            keep[k].append(feed[k][0].astype(np.uint8) if k.endswith("imgs") else feed[k][0])
        ts.append(t)
        refs.append(out)
    out_dir.mkdir(parents=True, exist_ok=True)
    for k, v in keep.items():
        np.save(out_dir / f"{k}.npy", np.stack(v))
    np.save(out_dir / "t.npy", np.array(ts))
    np.save(out_dir / "ref.npy", np.stack(refs))
    print(out_dir, len(ts), "frames")


def main():
    spec = json.loads(Path(sys.argv[2]).read_text())
    if sys.argv[1] == "calib":
        cmd_calib(spec, Path(sys.argv[3]))
    elif sys.argv[1] == "eval":
        cmd_eval(spec, sys.argv[3], sys.argv[4], float(sys.argv[5]), float(sys.argv[6]), Path(sys.argv[7]))
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
