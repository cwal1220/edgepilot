"""MaixCAM2 녹화로 만드는 master 코어용 캘리브레이션·평가 데이터.

샘플은 보드에서 런타임이 코어에 넣는 것을 가깝게 따른다: 녹화된 H.264 프레임(1280x720)을 기기 워프
(tools/model/model_warp.py, src/common/app_config.h의 MaixCAM2 내부 파라미터)에 통과시킨 것, 타워별 5프레임
이미지 이력(t-4와 t), controlsd가 보낸 desire 펄스(src/model/model_temporal.h와 같은 desire_step: 상승 에지, 회전이
끝나면 회전 펄스 지우기, 100틱을 25로 max-pool), fp32 코어를 순서대로 돌려 얻은 코어 자신의 96틱 hidden-state 이력.
런타임과 다른 점: 보정과 카메라 장착은 그 구간 녹화의 중앙값 하나를 쓴다(보드는 프레임마다 그때 값). 워프는 예전
빌드의 CPU 워프를 옮긴 것이고 보드는 GDC로 워프한다(docs/verification.md가 둘의 차이를 잰다). H.264를 다시 풀어 얻은
프레임이라 보드가 본 원본 프레임과도 압축만큼 다르다.

  make_m2_data.py calib <spec.json> <out_dir>
      Pulsar2 캘리브레이션 세트(<out_dir>/<tensor>.tar 안의 NNNN.npy, batch 축이 있는 float32).
      spec: {"core": core_fp32.onnx, "routes_dir": 녹화 route들이 있는 디렉터리(<route>/events, <route>/segments),
             "groups": [[route, [seg, ...]], ...]   이어진 세그먼트, 스트림 하나로 디코드한다,
             "exclude": {route: [t, ...]}             t-12..t+5 s를 뺀다(평가 순간),
             "dense": {route: [[a, b], ...]},         여기서는 촘촘히 뽑는다(빛 변화),
             "stride": 80, "dense_stride": 5}         샘플 사이 프레임 수
      4번째 샘플마다 합성 desire 펄스 하나를 더 넣어 desire 범위를 [0, 1]로 유지한다.

  make_m2_data.py eval <spec.json> <route> <seg>[,<seg>...] <from> <to> <out_dir>
      8 s 워밍업 뒤 [from, to](첫 ControlState부터 잰 route 초) 안 모든 프레임의 정확한 입력.
      보드의 run_axmodel_assembled.py가 읽는 frame-major .npy이고, 같은 입력의 fp32 출력(ref.npy)과
      프레임 시각(t.npy)도 함께 낸다.
"""
import argparse
import io
import json
import sys
import tarfile
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import recording_reader as rr  # noqa: E402
from model_warp import WarpPair  # noqa: E402

FEAT = 1064
TENSORS = ("input_imgs", "big_input_imgs", "desire", "features_buffer", "traffic_convention")


def recorded(route_dir: Path):
    """ControlState(시각, desire)와 모든 ModelState의 시각, 보정, 카메라 장착."""
    parts = []
    for chunk in rr.iter_states(route_dir, rr.RECORD_CONTROL_STATE, rr.RECORD_MODEL_STATE):
        _, c = chunk[rr.RECORD_CONTROL_STATE]
        _, model = chunk[rr.RECORD_MODEL_STATE]
        cal = model["calibration"]
        parts.append({"cs_t": c["timestamp_ns"] * 1e-9, "cs_des": c["desire"].astype(np.int64),
                      "m_t": model["capture_timestamp_ns"] * 1e-9,
                      "rpy": np.stack([cal["roll"], cal["pitch"], cal["yaw"]], 1),
                      "mount": np.stack([model["camera_offset_m"], model["camera_height_m"]], 1).astype(float)})
    v = rr.concat_chunks(parts)
    return v["cs_t"], v["cs_des"], v["m_t"], v["rpy"], v["mount"]


def route_segments(spec: dict, route: str, segs: list[str]) -> tuple[Path, list[Path]]:
    """route 디렉터리와 그 안의 세그먼트 디렉터리들(<route>/segments/<seg>)."""
    route_dir = Path(spec["routes_dir"]).expanduser() / route
    return route_dir, [route_dir / "segments" / s for s in segs]


def run_stream(sess, route_dir: Path, seg_dirs: list[Path]):
    """이어진 세그먼트 구간의 프레임마다 (route 시각, feed, 원시 출력)을 낸다."""
    cs_t, cs_des, m_t, rpy_all, mount_all = recorded(route_dir)
    t_first = cs_t[0]
    segments = [rr.read_segment_index(d) for d in seg_dirs]
    t0 = segments[0].frames["capture_timestamp_ns"][0] * 1e-9 - t_first
    t1 = segments[-1].frames["capture_timestamp_ns"][-1] * 1e-9 - t_first
    sel = (m_t - t_first >= t0 - 30) & (m_t - t_first <= t1)
    warp = WarpPair(1280, 720)
    offset, height = np.median(mount_all[sel], axis=0)
    warp.set_calibration(np.median(rpy_all[sel], axis=0), offset, height)
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
        desire, prev = desire_step(desire, prev, int(cs_des[j]))
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


def desire_step(history: np.ndarray, prev: np.ndarray, desire: int) -> tuple[np.ndarray, np.ndarray]:
    """desire 이력 [100, 8] 한 틱: 상승 에지 펄스, 회전 desire(1, 2)가 아니면 이력의 회전 펄스 지우기, 밀고 넣기.
    src/model/model_temporal.h의 push_desire_pulse와 같은 규칙이다(check_recording_reader가 대조한다)."""
    cur = np.zeros(8, np.float32)
    if desire > 0:
        cur[desire] = 1.0
    pulse = np.where(cur - prev > 0.99, cur, 0.0).astype(np.float32)
    history = history.copy()
    if desire not in (1, 2):
        history[:, 1:3] = 0.0
    history = np.roll(history, -1, 0)
    history[-1] = pulse
    return history, cur


def session(core: str):
    import onnxruntime as ort  # 시험(check_recording_reader)이 desire_step만 읽을 수 있게 여기서 읽는다

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
        n, kept, start = 0, 0, None
        for t, feed, _ in run_stream(sess, *route_segments(spec, route, segs)):
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
    keep = {k: [] for k in TENSORS}
    ts, refs = [], []
    for t, feed, out in run_stream(sess, *route_segments(spec, route, segs.split(","))):
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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    calib = commands.add_parser("calib")
    calib.add_argument("spec", type=Path)
    calib.add_argument("out_dir", type=Path)
    evaluate = commands.add_parser("eval")
    evaluate.add_argument("spec", type=Path)
    evaluate.add_argument("route")
    evaluate.add_argument("segs", help="<seg>[,<seg>...]")
    evaluate.add_argument("t_from", type=float, metavar="from")
    evaluate.add_argument("t_to", type=float, metavar="to")
    evaluate.add_argument("out_dir", type=Path)
    args = parser.parse_args()
    spec = json.loads(args.spec.read_text())
    if args.command == "calib":
        cmd_calib(spec, args.out_dir)
    else:
        cmd_eval(spec, args.route, args.segs, args.t_from, args.t_to, args.out_dir)


if __name__ == "__main__":
    main()
