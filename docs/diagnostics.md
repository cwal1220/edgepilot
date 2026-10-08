# Diagnostics

[← Documentation index](../README.md)

Procedures and measured results for the diagnostic tools. The tools themselves,
their options, and how to build them are listed in
[diagnostics/README.md](../diagnostics/README.md); the host unit tests are in
[tests/README.md](../tests/README.md).

## HUD snapshots

`hud_snapshot` renders the HUD renderer off-line for 27 scenarios (driving,
alerts, cards, warnings and the less common branches: soft disable, panda fault,
radar-only lead, bar TPMS, engaged but blocked, ...) at the HUD's 640x480
landscape size, writes each frame as an `EDGEARGB` file and prints draw timings.
`--portrait` draws the way `overlayd` does, transposed into the portrait panel's
480x640 buffer (`--flip-x`/`--flip-y` add the board's axis flips), and writes
that buffer. The renderer needs no OpenCV, so the tool builds on the host as
well as in the container build with `-DEDGEPILOT_BUILD_DIAGNOSTICS=ON`; time it
on the board, where the numbers matter (the host is 20–40x faster). Frames from
two builds of the same compiler are byte-identical when the drawing did not
change, which is the check used for HUD refactors (compare the host and the
container build separately: GCC fuses multiply-adds that clang leaves apart).

```sh
./hud_snapshot --out /tmp/hud
```

`hud_snapshot --model model.bin --control control.bin` replays a recorded
`ModelState` / `ControlState` pair instead of the synthetic scene;
`python3 tools/ui/hud_tools.py inputs <route_dir> <out_dir>` extracts such a
pair (the current layout, unchanged in v9; v7 records are zero-filled), plus the matching camera frame when the route copy has
its `segments/`. `python3 tools/ui/hud_tools.py compose /tmp/hud [camera.png]`
turns the frames into PNGs and composites them over the centre 4:3 of the
camera frame, as the screen shows it.

`alert_sound_preview --out /tmp/alert` writes each alert sound `overlayd`
plays (`/tmp/alert_engage.wav`, ...) as a 48 kHz mono WAV at 100% volume, from
the same synthesis code, so a sound change can be heard on the host. On the
board, `pkill -USR1 -f "[.]/overlayd"` plays them one by one through the
speaker.

## NV12 replay

`modeld` can run headless from a recorded route: replay mode reads an
`SCNV12R1` file instead of the camera ring and feeds the same GDC warp as live
capture, so it validates model execution and online calibration from stored
segments. It needs the NPU, so it runs on the board.

```sh
# host: cut 120 frames of a route into an SCNV12R1 replay
python3 tools/model/make_replay.py --route recordings/<route> --out /tmp/replay_nv12 --frames 120
scp /tmp/replay_nv12/replay.scnv12 root@192.168.219.117:/root/edgepilot/

# board (stop the manager first, or at least modeld)
cd /root/edgepilot
EDGEPILOT_REPLAY_NV12=/root/edgepilot/replay.scnv12 \
  ./modeld models/supercombo.axmodel
```

## Model swap verification

A model swap changes three things at once: the warp input, the temporal
plumbing, and the network. The board side is verified by running a replay
with the raw outputs dumped:

```sh
# board: same frames through the runtime, dumping raw outputs
EDGEPILOT_REPLAY_NV12=/root/verify/replay.scnv12 \
EDGEPILOT_RAW_DUMP=/root/verify/board_raw.bin \
EDGEPILOT_CALIB_AUTO=0 \
  ./modeld models/<candidate>.axmodel
```

`board_raw.bin` is an `SCODMP1` file of 2576-float frames. Compare it against a
host reference on the slices that drive control (plan lateral offset, lane
positions) rather than on the raw vector, and check that the hidden-state slice
evolves smoothly — a dead temporal buffer still produces plausible single-frame
output. With `EDGEPILOT_CALIB_AUTO=0`, the host reference must use the rpy the
board restored from `params/calibration.json`, because the calibration service
feeds the input warp on every frame.

For the master core, the fp32 reference with the runtime's queue semantics is
built by `tools/model/axmodel/make_m2_data.py eval` from a MaixCAM2 recording, and
`tools/model/axmodel/run_axmodel_assembled.py` runs an axmodel on the same
inputs on the board. A host runner that takes an `SCNV12R1` replay is not in
the repository yet.

Measured when the master axmodel was brought up (200-frame replay): the board
output against the host axengine runner gave a plan lateral difference of
0.0008 m at 2 s and a hidden-state cosine similarity of 0.9994.

## Lateral bias

The tools below run on the host over `recordd` routes.

If the car holds one side of the lane, `tools/model/lane_bias.py` says whether
the camera calibration is responsible:

```sh
python tools/model/lane_bias.py <route_dir> [<route_dir> ...]
```

It reads the recorded `modelState` and `controlState`, keeps straight engaged
stretches, and fits the perceived lane-centre offset against distance:

| term | meaning |
| --- | --- |
| translation (m) | camera off the vehicle centreline, or the car genuinely off-centre. **Camera intrinsics cannot produce this** -- a principal-point or focal-length error acts about the camera, so its lateral effect is exactly zero at `x=0`. |
| rotation (rad/m) | the calibration-shaped term. A wrong `cx` of `dcx` pixels appears here as roughly `dcx/fx`. |

It also prints the tuning that was active on that drive from the route's own
`params/` snapshot, the mean steering angle needed to hold a straight line, and
the mean curvature command, so a control bias can be told apart from a
perception bias.

Straightness is judged from the model's own 48 m path, never from the steering
angle: when the car needs a non-zero angle to go straight, an `|angle| < k`
filter keeps one side of the curve distribution and manufactures a rotation
term that is not there. On the 2026-08-19 route that mistake reported
-4.22 mrad where the honest figure is -0.74 mrad.

Measured on the two logged drives (no lane-line offset was applied on either):

| drive | `path_offset_m` | translation | rotation |
| --- | ---: | ---: | ---: |
| 2026-08-16 | 0.00 | +19.9 cm | +0.18 mrad |
| 2026-08-19 | 0.07 | +7.2 cm | -0.74 mrad |

The rotation term is under 1 mrad on both, so the left-hugging on those drives
was a lateral offset, not a camera-matrix error.

## Lateral dataset extraction

`extract_lateral_dataset` replays a recording and writes one CSV row per
`ControlState` record (~62 Hz), joining the CAN state decoded by the runtime's
own `vehicle_can` so signs and scaling match the board exactly:

```sh
cmake --build build-host --target extract_lateral_dataset -j2
./build-host/bin/extract_lateral_dataset out.csv <route>/events/*.bin
```

Version 2 route-level `events.bin` files work as well; a file truncated by the
tmpfs fill stops at the zero-filled gap with the byte offset on stderr.

Columns: time, wheel speed, `active`/`desire`/`block`, steering angle, driver
and applied torque (`tq_norm` is sign-corrected and divided by `steer_max`),
ESP12 lateral/longitudinal acceleration and yaw rate, the live bank estimate
and its roll equivalent, requested and measured curvature, and the NNFF-style
future values `la_p03..p15` / `roll_p03..p15` taken from the recorded model
plan.

Two deviations from the runtime are deliberate. The bank filter runs at row
rate with a `dt`-derived alpha rather than the controller's fixed 100 Hz step,
and future lateral acceleration is `v * dpsi/dt` off the plan's yaw angles
because the recorded `ModelState` has no plan acceleration. Both were checked
against known results: straight-line bank reproduces -0.178 on the 8-28 route
(logged: -0.176), and the total-least-squares torque fit over the 8-19 route
returns `latAccelFactor` 4.00, the value that route was fit to.

## Lateral planner replay

`replay_planner` re-runs `LateralPlanner` over a recording and writes
what the planner asked for, one row per `ModelState`. The MPC has no
board-specific dependency, so a recorded route can be re-planned without the
board:

```sh
cmake --build build-host --target replay_planner -j2
./build-host/bin/replay_planner out.csv <route>/events/*.bin
```

Columns include the recorded and re-planned desired curvature, the MPC's own
`target_curv`/`heading0`, the lane observations behind the plan, and the
`laneless`/`mpc_valid` flags. Options go before the output file:

- `--laneless` or `--lane` forces that mode, so the same route can be re-planned
  both ways without editing `params/steering.json`.
- `--steering` loads a parameter file, such as the route's
  `params/steering.json` snapshot.
- `--vehicle` feeds the recorded blinkers, driver torque and blind spot (from
  the CAN, without the Panda's echoes, as controlsd does) and the recorded
  `active`. Without it the planner sees no blinker and no driver, so lane
  changes never run.
- `--exact` writes every `LateralTarget` field and the lag-adjusted curvature
  as `%a` hex floats. Two builds that plan identically give byte-identical
  files, which is the check used for planner refactors:

```sh
R=<route>; ./build-host/bin/replay_planner --exact --vehicle --lane \
  --steering $R/params/steering.json out.csv $R/events/*.bin
```

## Control tick replay

`replay_controls` runs a recording through `ControlsTick`, the logic of one
controlsd tick, at 10 ms ticks: the recorded CAN, `ModelState`, `PandaState` and
`Localization` records go in, the planner is computed in place and the clock is
the tick time, so identical code gives a bit-identical `--dump` and digest. It
also reports how often engaged/active/desire agree with the recorded
`ControlState`. A route engaged before recording started needs
`--force-engaged`.

```sh
R=<route>; ./build-host/bin/replay_controls --steering $R/params/steering.json \
  --cruise $R/params/adaptive_cruise.json --dump out.txt $R/events/*.bin
```

## Related documents

- [Departure alerts](departure-alerts.md)
- [K7 Panda port](k7-panda-port.md)
