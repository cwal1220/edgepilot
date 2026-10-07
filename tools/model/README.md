# Model tools

The MaixCAM2 model is built by [`axmodel/`](axmodel/README.md): it cuts the NPU
core out of the openpilot master `driving_supercombo.onnx`, builds calibration
and evaluation data, and compiles it with Pulsar2 6.0 into the axmodel that
`modeld` loads. See [`../../models/README.md`](../../models/README.md) for
the contract and the board numbers.

The other scripts here read recorded routes and reproduce the device's input
pipeline.

## axmodel pipeline

- `axmodel/extract_core.py`
  - cuts the history queues out of the released graph and rewrites the ops
    Pulsar2 does not accept (opset-20 Cast, GatherND, the `Where(-inf)` mask, 2D
    LpNorm) into equivalent ones; fp16 is promoted to fp32.
- `axmodel/split_outputs.py`
  - exports every head as its own output so each gets its own U16 scale. The
    runtime puts them back together (`src/model/model_output_assembly.h`).
- `axmodel/make_m2_data.py`
  - builds the calibration and evaluation sets from MaixCAM2 recordings with the
    exact runtime inputs (device warp, t-4/t pairs, recorded desire, the fp32
    core's own feature history). `m2_calib_20261004.json` is the spec used on
    2026-10-04.
- `axmodel/run_axmodel_assembled.py`
  - runs an axmodel on the board over a `make_m2_data.py eval` set and writes
    the outputs in the original 2576-float layout, for comparison with the fp32
    reference.
- `axmodel/pulsar2_u16_u8in.json`
  - the Pulsar2 build config: AX620E / NPU1 (one core; the AI-ISP denoiser
    uses the other), U16 everywhere, SmoothQuant, uint8 image inputs.

## Host environment

```sh
python3 -m venv .model-venv
.model-venv/bin/pip install -r tools/model/requirements.txt
```

`axmodel/` additionally needs `onnx` and `onnxruntime` (both in the
requirements) and the Pulsar2 6.0 Docker image for the compile step.

## Recording-driven helpers

`recording_reader.py` decodes a route and `model_warp.py` is a numpy port of
the CPU warp in `src/model/model_input_transform.cc`.

- `recording_reader.py`
  - the Python mirror of `src/recording/recording_format.h` and `src/common/ipc_messages.h` for
    the analysis tools, checked against the C++ asserts by
    `tests/check_recording_reader.py` (`scripts/web_console/` keeps its
    own standard-library copy of the layouts it shows, checked by
    `tests/check_web_console.py`). ControlState's field at offset 56 is
    `cluster_speed_kph`, the cluster's display speed; `ego_speed_kph` is the
    wheel speed the controller uses.
- `make_replay.py`
  - writes an `SCNV12R1` replay for `modeld` replay mode on the board.
- `lane_bias.py`
  - measures the lateral bias of a drive and splits it into a translation term
    and a rotation term, which is what tells you whether a lane-hugging
    complaint is a camera-calibration problem or not. See
    `../../docs/diagnostics.md`.
- `model_warp.py`
  - also used by `tools/camera/warp_preview.py` to show the MaixCAM2 model views.
    It holds the MaixCAM2 intrinsics once (`maixcam2_intrinsics`, the
    `src/common/app_config.h` values) for the preview, the calibration report and the
    axmodel calibration data; the warp uses them unless given others.
