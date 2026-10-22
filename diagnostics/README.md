# Diagnostic tools

[← Documentation](../docs/README.md)

Analysis, replay and benchmark tools kept outside the runtime path. They are not
in the default build; [`CMakeLists.txt`](CMakeLists.txt) builds them when
`EDGEPILOT_BUILD_DIAGNOSTICS=ON`. The unit tests and the Python checks
(`check_*.py`) are in [`../tests/`](../tests/README.md).

Most tools read a `recordd` route. This map shows what each tool reads and what
it writes:

```mermaid
flowchart LR
    route["recorded route"] --> events["events/*.bin"]
    route --> inputs["hud_tools.py inputs<br>model.bin, control.bin"]
    events --> closed["replay_closed_loop<br>score, CSV"]
    events --> planner["replay_planner<br>CSV"]
    events --> controls["replay_controls<br>digest, dump"]
    events --> learners["replay_lateral_learners<br>learned values"]
    events --> loc["replay_localization<br>CSV"]
    events --> dataset["extract_lateral_dataset<br>CSV"]
    inputs --> hud["hud_snapshot<br>EDGEARGB frames"]
    classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
    class route,events,inputs,closed,planner,controls,learners,loc,dataset,hud proc
```

`hud_snapshot` also draws synthetic scenes without any input, and
`alert_sound_preview` needs no input at all.

## Build

The host tools build without the AX runtime or the MaixCAM2 SDK:

```sh
cmake -S . -B build-host \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDGEPILOT_BUILD_RUNTIME=OFF \
  -DEDGEPILOT_BUILD_DIAGNOSTICS=ON
cmake --build build-host --target replay_closed_loop -j2
```

For the board, add `-DEDGEPILOT_BUILD_DIAGNOSTICS=ON` when configuring
`build-ax630/` in the container from `tools/docker_ax630/`, and the same tools
appear in `build-ax630/bin`. `build.sh` takes no options, so configure inside
the container by hand.

## Tools

The replay tools read `recordd` recordings (`events/*.bin`). They all read the
records in time order as they go (`route_events.h` over
`recording/time_ordered_events.h`), so a route is never held in memory, and
they exit with code 1 when a path is not an event file.

| Tool | Usage | What it does |
| --- | --- | --- |
| `replay_closed_loop` | `[options] <out.csv\|-> <events.bin...>` | Replays the lateral control loop in closed loop, with the car's response simulated. The recorded ESP12 (corrected by the simulated car's drift) and lagd delay go in as controlsd gets them. `--lane-filter W` is the lane-line time filter's weight for the new measurement (1 turns it off, [Lane-line filter](../docs/lane-line-filter.md)). `--lateral-nn m.json` loads the learned torque model for a `--steering` file with `nn_feedforward` on. |
| `replay_planner` | `[--laneless\|--lane] [--exact] [--vehicle] [--steering s.json] [--lane-filter W] <out.csv> <events.bin...>` | Re-runs `LateralPlanner` on the recorded `ModelState`/`ControlState` and writes the plan and the lag-adjusted desired curvature for each frame ([Lateral planner replay](../docs/diagnostics.md#lateral-planner-replay)). `--laneless`/`--lane` force a path mode, `--steering` loads a parameter file (the code defaults without it), `--vehicle` feeds the recorded blinkers, driver torque and active flag (lane changes need it), `--exact` writes every `LateralTarget` field bit for bit (`%a`), and `--lane-filter` is the lane-line filter's weight (1 turns it off) |
| `replay_controls` | `[--steering s.json] [--cruise c.json] [--vehicle-json j] [--torque-cache c.bin] [--force-engaged] [--lane-filter W] [--lateral-nn m.json] [--nn-inputs out.csv [--nn-inputs-every N]] [--dump out.txt] <events.bin...>` | Feeds a recording through the logic of one controlsd tick (`ControlsTick`) at 10 ms ticks, writes a digest of what it would send and compares it with the recorded `ControlState` ([Control tick replay](../docs/diagnostics.md#control-tick-replay)). `--vehicle-json` and `--torque-cache` restore the learners' saved state from the route's `params/`, `--force-engaged` engages without the button, `--lateral-nn` loads the learned torque model (with `nn_feedforward` on in the `--steering` file), `--nn-inputs` writes the model's inputs for edgepilot-nnlc ([Lateral NN](../docs/lateral-nn.md)), and `--dump` writes a line per tick |
| `replay_lateral_learners` | `[options] <events.bin...>` | Runs the paramsd and torqued learners on a recording and prints the learned values |
| `replay_localization` | `[--steering s.json] [--lag-cache live_delay.json] <out.csv> <events.bin...>` | Feeds a recording, in time order, through the same code as the board's `locationd` (the pose Kalman filter and lagd), writes one CSV row per IMU batch, and at the end prints a yaw-rate comparison against the CAN, the lagd result and the board's recorded lagd state. Give the route's `params/steering.json` and `params/live_delay.json` to start lagd as `locationd` did; without them lagd starts from scratch. It needs a recording with IMU records |
| `extract_lateral_dataset` | `<out.csv> <events.bin...>` | One CSV row per `ControlState`, records read in time order. The CAN is decoded by the same `vehicle_can` as the runtime |
| `hud_snapshot` | `[--model m.bin] [--control c.bin] [--iterations N] [--out PREFIX] [--portrait]` | Draws 27 HUD scenarios at the same 640x480 as `overlayd` into `EDGEARGB` frames (`PREFIX_<scenario>.argb`), and prints the drawing time over N iterations (50 by default). `--model` and `--control` draw a recorded `ModelState`/`ControlState` pair (from `tools/ui/hud_tools.py inputs`) instead. `--portrait` draws into the portrait 480x640 buffer with a transpose, as the board does. Time it on the board |
| `alert_sound_preview` | `[--out PREFIX]` | Writes the `overlayd` alert sounds with the same synthesis code to `PREFIX_<name>.wav` (48 kHz mono, 100% volume) |

## `replay_closed_loop`

The tool turns the lane geometry the recording saw back into car coordinates by
how far the simulated car has drifted from it (`dy`, `dpsi`), so a control
change really makes the car move differently. Ticks with driver torque, or
inactive ticks, resynchronise to the recorded state, so every free-running
stretch starts from the real pose.

- Plant: `--wn`, `--zeta`, `--delay`, `--gain G` (all speed nodes) or
  `--gain-pts a,b,c,d`. `--path-lag S` is the steering-angle-to-path lag in
  seconds (0.11 by default; 0 makes the car travel along the steering angle's
  curvature, as before 2026-10-17).
- Controller: `--sad` (steer_actuator_delay), `--kp`, `--ki`, `--laf`. With
  `use_live_delay` on, the recording's lagd estimate is the path delay
  while it is valid, as on the board.
- Driver intervention hysteresis: `--driver-high`, `--driver-low`,
  `--driver-release`.
- `--open-loop`: stops the pose correction, to see only how well the plant
  reproduces the recorded drive.
- `--steering route/params/steering.json`: the tuning at the time of the
  recording (including the path mode). The recording's LearnerState learned
  values (paramsd, torqued) go into the controller as in controlsd, and the
  inverse steering-angle model uses the same learned values.
- `--camera-shift D`: shifts the model output as if the camera mount offset
  differed from the recording by D m.
- `--camera-lever a,b`: the camera lever `a − b·v²` m that tyre slip adds to
  the camera's sideways travel (2.33, 0.0029 by default, measured 2026-10-22;
  `0,0` reproduces the replay before then). See
  [Closed-loop replay](../docs/closed-loop-replay.md).
- `--torque F,O,R`: substitutes torqued values (factor, offset, friction) to see
  the behaviour once they are valid.
- In laneless mode no lane holds the position, so the closed loop diverges
  (2 m clamp). Use it only in Lane mode.

It prints the reproduction score (overall and per speed band) on standard
output. Give `-` as the output path when no CSV is needed. Besides the earlier
columns, the CSV has `k_path` (the simulated path curvature), `delay` (the
path delay the controller used), `nn` and `ff` (whether the learned torque
model was blended in, and the feed-forward in lateral-acceleration units) and
`bank` (the controller's road-bank estimate). Plant identification
and how to read the score are in
[Closed-loop replay](../docs/closed-loop-replay.md).

## `replay_lateral_learners`

The tool calls the same `LateralLearners` as controlsd. The active flag and the
torque sent come from `ControlState`; driver intervention comes from the
recorded driver torque, debounced as the controller does it.

- `--steering route/params/steering.json`: uses the tuning at the time of the
  recording (priors, delay, input source). Without it the code defaults apply,
  which may differ from the board.
- `--vehicle-json route/params/live_parameters.json`: starts from the paramsd
  saved values.
- `--source steering|locationd|esp`: the yaw-rate and roll source for paramsd
  and torqued. The default (`steering`) follows
  `use_locationd_learner_inputs` and, as controlsd does, falls back to ESP12
  when locationd is more than 0.5 s stale or its filter is invalid.
- `--metric-from N`: counts the curvature comparison printed at the end from
  file N (counted from 0). The earlier files only serve to converge the
  learning.
- `--upstream-schedule`: observes the steering angle and speed only at 20 Hz,
  as upstream does (the default is every tick).
- `--torque-cache c.bin`: restores it when it exists and overwrites it at each
  save tick, to chain several drives like the runtime cache.
- `--outputs`/`--torque-outputs` write the published paramsd and torqued
  values as CSV.

## Writing a tool

- The Korean `//` comment at the top of the file says what the tool is for and
  gives its usage on a `사용:` line.
- Input files are positional arguments; options are `--name value` or `--name`
  without a value, read with `ArgReader` (`cli_args.h`). An unknown option or
  one missing its value prints the usage.
- Numbers are read with `ArgReader::read`: the whole value must be a finite
  number (or a list of exactly that many) within the option's range. `std::atof`
  and `sscanf` read `abc`, `16x` or a short list as 0 or a prefix, so a typo ran
  with a wrong value instead of failing.
- A usage error exits with code 2; a run error, where an input cannot be read or
  a result does not match, exits with 1.

## Related

- [Diagnostics](../docs/diagnostics.md): procedures and results for HUD
  snapshots, dataset extraction, and the planner and control-tick replays
- [Closed-loop replay](../docs/closed-loop-replay.md): plant identification and
  the score of `replay_closed_loop`
- [Lane-line filter](../docs/lane-line-filter.md): the filter that
  `--lane-filter` weighs
- [Lateral NN](../docs/lateral-nn.md): the torque model that `--lateral-nn`
  loads
- [Recording](../docs/recording.md): the route layout the replay tools read
- [Host tests](../tests/README.md): the unit tests and Python checks
- [Build and deploy](../docs/build-and-deploy.md): the host and board builds
