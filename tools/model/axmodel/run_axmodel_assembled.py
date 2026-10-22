"""준비된 입력으로 axmodel을 보드에서 개루프로 돌리는 러너(pyaxengine).

사용: run_axmodel_assembled.py <model.axmodel> <input_dir> <out.bin>
<input_dir>에는 모델 입력마다 frame-major <input>.npy가 하나씩 있다(make_m2_data.py eval이 쓴다).
프레임마다 출력을 원래 출력 배치의 float32 2576개로 <out.bin>에 쓴다. 헤드가 나뉜 axmodel
(split_outputs.py)의 출력을 src/model/model_output_assembly.h와 같은 방식으로 다시 합친다. modeld가 NPU 코어를
쥐고 있으므로 edgepilot.service를 멈춘 채 돌린다.
"""
import argparse
import time
import numpy as np
import axengine as axe

PARTS = [("out_meta", 0, 55), ("out_desire_pred", 55, 32), ("out_pose", 87, 12), ("out_wide_from_device", 99, 6),
         ("out_road_transform", 105, 12), ("out_lanes", 117, 528), ("out_lane_prob", 645, 8),
         ("out_road_edges", 653, 264), ("out_lead", 917, 144), ("out_lead_prob", 1061, 3),
         ("out_hidden", 1064, 512), ("out_plan_std", 1576 + 495, 495), ("out_desire_state", 2566, 8)]
MEAN = np.arange(495).reshape(33, 15)

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("model", help="model.axmodel")
parser.add_argument("input_dir")
parser.add_argument("out", help="out.bin")
args = parser.parse_args()
model, in_dir, out_path = args.model, args.input_dir, args.out
sess = axe.InferenceSession(model)
ins = sess.get_inputs(); outs = [o.name for o in sess.get_outputs()]
data = {i.name: np.load(f"{in_dir}/{i.name}.npy", mmap_mode="r") for i in ins}
dtypes = {i.name: (np.uint8 if "uint8" in str(i.dtype) else np.float32) for i in ins}
n = len(next(iter(data.values())))
print("outputs:", len(outs), "frames:", n, flush=True)
times = []
with open(out_path, "wb") as f:
    for k in range(n):
        feed = {i.name: np.ascontiguousarray(data[i.name][k][None], dtype=dtypes[i.name]) for i in ins}
        t0 = time.perf_counter(); res = sess.run(None, feed); times.append(time.perf_counter() - t0)
        r = {name: np.asarray(v, np.float32).reshape(-1) for name, v in zip(outs, res)}
        y = np.zeros(2576, np.float32)
        for name, off, cnt in PARTS: y[off:off + cnt] = r[name]
        y[1576 + MEAN[:, :9].reshape(-1)] = r["out_plan_motion"]
        y[1576 + MEAN[:, 9:].reshape(-1)] = r["out_plan_orient"]
        f.write(y.tobytes())
t = np.array(times) * 1e3
print(f"done {n} frames, run() mean {t.mean():.2f} ms median {np.median(t):.2f} ms", flush=True)
