# Runtime options

[← Documentation index](../README.md)

Boolean environment variables follow one convention: unset or empty uses the
default, `0`/`false`/`no`/`off`/`n` (case-insensitive) is false, anything else is
true.

## Manager

- `EDGEPILOT_MODEL=/path/to/supercombo.axmodel`
  - overrides the model selected by `manager.py` (default
    `models/supercombo.axmodel` in the install directory). A command-line
    argument wins over it.
- `EDGEPILOT_STOP_LAUNCHER=0`
  - leaves the stock `launcher.service` and `/maixapp/apps/` running. The
    default stops them, since they hold the camera and NPU.

## Model and calibration

- `modeld <supercombo.axmodel>`
  - the only argument. The runtime targets the openpilot master supercombo core:
    5 inputs (`input_imgs` and `big_input_imgs` uint8 `[1,12,128,256]`,
    `desire` `[1,25,8]`, `features_buffer` `[1,24,512]`, `traffic_convention`
    `[1,2]`) and 2576 output floats (15 head outputs reassembled, or one
    `[1,2576]` output). `modeld` checks names, shapes, dtypes,
    and buffer sizes at startup and refuses any other axmodel rather than
    misreading it.
- `EDGEPILOT_PROFILE=1`
  - `modeld` prints warp/inputs/NPU/outputs averages every 30 frames;
    `overlayd` adds HUD draw and video push timing to its status line.
- `EDGEPILOT_CAMERA_INTRINSICS=fx,fy,cx,cy`
  - camera intrinsics at 1920x1080, scaled to the capture size. The default is
    the calibrated MaixCAM2 camera (`1131.24,1130.85,940.13,552.60`); set it to
    replay footage from another camera.
- `EDGEPILOT_CALIB_ROLL_DEG`, `EDGEPILOT_CALIB_PITCH_DEG`,
  `EDGEPILOT_CALIB_YAW_DEG`
  - manual calibration in degrees for both the HUD projection and the model
    input warp. If any value is set, it wins and online calibration is applied to
    neither. Otherwise the saved calibration is restored and pose-based online
    calibration feeds the next frame's input warp, matching openpilot's
    `cameraOdometry -> liveCalibration -> modeld` loop.
- `EDGEPILOT_CALIB_AUTO=0`
  - disables pose-based online calibration and keeps the restored or
    manually supplied projection.
- `EDGEPILOT_LOG_CALIB=1`
  - prints the online calibrator status, accepted/rejected sample counts, valid
    block count, rpy, and spread.

## Camera and input warp

- `EDGEPILOT_MAX_SHUTTER_US=N`
  - caps the auto-exposure shutter in `camerad` (default `33333`, the
    30 fps frame time, so the 20 fps sensor does not add motion blur). `0`
    leaves the sensor default.
- `EDGEPILOT_WARP_CPU=1`
  - disables the GDC warp and returns the whole input warp to the CPU; the
    wide tower is warped on the second core. The GDC path also falls back to the
    CPU when it cannot be set up or the frame is NV21.

## Storage and replay

- `EDGEPILOT_PARAMS_DIR=/path/to/params`
  - overrides the shared runtime parameter directory, and is the only way to
    relocate parameter files; there are no per-file overrides. The default is
    `params/` relative to the runtime working directory. The web console reads
    the factory defaults from the directory beside it with `.defaults` added
    (`params.defaults/`, filled by the upload script). Stable online
    calibration is stored atomically in `params/calibration.json` and restored
    before the first model frame. Manual `EDGEPILOT_CALIB_*` values take
    precedence and seed this file.
- `EDGEPILOT_REPLAY_NV12=/path/to/replay.scnv12`
  - when launching `modeld` directly, runs headless from an `SCNV12R1` NV12
    replay file instead of the camera ring. Width, height, and frame count are
    read from the replay header; the warp uses the same GDC path as live. This is
    for validating inference and online calibration from collected logs. It
    still runs on the board, since it needs the NPU.
- `EDGEPILOT_CAMERA_OFFSET_M=m`, `EDGEPILOT_CAMERA_HEIGHT_M=m`
  - the camera mount for replay mode, which does not read
    `params/display.json` (defaults `0` and the model height, 1.22 m). Live
    capture uses `camera_offset_m` and `camera_height_m` from the device
    settings.
- `EDGEPILOT_MAX_FRAMES=N`
  - stops after `N` frames (`modeld` inferred frames, `camerad`
    captured frames). This is mainly useful with replay mode.
- `EDGEPILOT_RAW_DUMP=/path/to/dump.bin`
  - during replay, writes every raw model output to an `SCODMP1` file that
    `gtest_model_output` reads; see
    [diagnostics](diagnostics.md#model-swap-verification).

`recordd` reads `EDGEPILOT_RECORD_ROOT` (default `recordings` under the install
directory) and `EDGEPILOT_RECORD_STAGING`. `params/recording.json` `enabled`
toggles recording.

## IMU

- `EDGEPILOT_IMU_DEV=/dev/i2c-1`, `EDGEPILOT_IMU_ADDR=0x6B`
  - where `imud` finds the LSM6DSOW. The defaults are the MaixCAM2's.

## Panda

- `EDGEPILOT_ENABLE_PANDA=1`
  - manager also starts `pandad` and switches the USB-C port to host mode
    (restored on exit). The manager default is off; `edgepilot.service` sets
    it. The binary must have been built with `-DEDGEPILOT_BUILD_PANDA=ON`.
- `EDGEPILOT_USB_ROLE=host|device`
  - fixes the USB-C role at manager start and leaves it on exit.
    `edgepilot.service` sets `host` (the Panda plugs into the USB-C port);
    set `device` to reach the board from a computer over USB. Unset, the role
    only changes with `EDGEPILOT_ENABLE_PANDA=1` as above.
- `EDGEPILOT_PANDA_SAFETY=nooutput|silent|elm327|hyundai|hyundaiCommunity|allOutput`
  - panda safety mode for `pandad`. Its standalone default is `nooutput`;
    the manager defaults to `hyundaiCommunity`. `hyundai` defaults its parameter
    to `2` (the Hyundai/Kia hybrid path), every other mode to `0`; an unknown
    name falls back to `nooutput`.
  - collected KIA K7 YG HEV logs from the current openpilot fork report
    `safety=hyundaiCommunity:0`, `sccBus=-1`, `mdpsBus=1`, and `sasBus=1`. Use
    `EDGEPILOT_PANDA_SAFETY=hyundaiCommunity` for shadow/TX experiments unless a newer
    fingerprint proves otherwise.
- `EDGEPILOT_PANDA_SAFETY_PARAM=N`
  - numeric safety parameter passed with the safety mode. Unset takes the mode's
    default; the manager sets `0`.
- `EDGEPILOT_PANDA_TX=1`
  - allows `pandad` to relay ordered `/dev/shm/edgepilot_sendcan` batches to
    panda. Its standalone default is `0`; the manager defaults to `1`.
- `EDGEPILOT_PANDA_LOG_CAN=1`
  - prints every received CAN frame from `pandad`. This is a bus-bringup
    aid only; at full bus load it is far too noisy to leave on.
- `EDGEPILOT_PANDA_ENGAGED=1`
  - sends panda heartbeat as engaged, only meaningful with `EDGEPILOT_PANDA_TX=1`. Its
    standalone default is disengaged; the manager defaults to engaged.
- `EDGEPILOT_PANDA_FIRMWARE=path`
  - the Panda application image the web console flashes, relative to the
    install directory. Default `firmware/panda.bin.signed` (built by
    `make -C firmware/panda`).
- `EDGEPILOT_PANDA_IDLE_US=5000`
  - sleep time used by `pandad` when panda returns no CAN frames and no
    pending `sendcan` batch exists. This keeps USB-only or parked shadow runs from
    stealing scheduler time from `modeld`.

## K7 control

- `EDGEPILOT_ENABLE_CONTROL=1`
  - manager starts `controlsd`. This is the manager default. It does not
    start `pandad` on its own; that needs `EDGEPILOT_ENABLE_PANDA=1`. No
    openpilot checkout or Python native extension is required.
- `EDGEPILOT_FORCE_ENGAGED=0|1`
  - bypasses the SET/CANCEL engage latch for offline replay only. Default is `0`
    and must remain `0` in a vehicle.

## Web console and display

- `EDGEPILOT_ENABLE_WEB_CONSOLE=0|1`
  - starts the web console (FastAPI, `web_console/`) with the manager. It
    defaults to the value of `EDGEPILOT_ENABLE_CONTROL`.
- `EDGEPILOT_WEB_CONSOLE_HOST=address`, `EDGEPILOT_WEB_CONSOLE_PORT=port`
  - select the web console's listen address and port. Defaults are
    `0.0.0.0:8080`.
- `EDGEPILOT_CALIBRATION_RESET_PATH`
  - the request file behind the web console's calibration reset (default
    `/dev/shm/edgepilot_calibration_reset`). `modeld` checks it every second;
    when it exists, `modeld` deletes it and restarts calibration from the
    beginning. Both sides read the same variable.

`overlayd` turns the backlight on at start (`pwmchip0/pwm3`, level from
`maix_backlight_value` in `/boot/configs` and `disp_max_backlight` in
`/boot/board`). The web console then applies `params/display.json` through
`scripts/web_console/backlight.py` on the same PWM: duty = 100 µs period x
brightness % x `disp_max_backlight` %, and `enabled: false` sets the duty to 0.

## Alerts

`overlayd` plays the alert sounds (`src/hud/alert_tones.cc`) on the board speaker. One
`aplay` starts with overlayd and stays open. A sound thread feeds it silence,
or the alert's samples when one fires. The HUD loop never spawns a process at
alert time. The amplifier stays on, so the first note is not cut. Latency is
under 0.1 s. A new alert interrupts the one playing.

- `EDGEPILOT_ALERT_SOUND=0` turns the sounds off (default on)
- `EDGEPILOT_ALERT_VOLUME` 0–100, default 70; `alert_volume_percent` in
  `params/display.json` (web 기기 설정) overrides it at runtime
- `EDGEPILOT_ALERT_PCM` ALSA device, default `plughw:0,1`

To check the speaker at the desk, run `pkill -USR1 -x overlayd`. Each
signal plays the next sound in the order engage, disengage, unable,
signal_changed, unavailable.

Departure alerts and engage refusals are also shown on the HUD, and every alert
is written as a `overlayd: alert=...` log line. Which events alert
is described in [Departure alerts](departure-alerts.md).

## Parameter files

The tracked JSON files in `params/` are the source of truth for K7 steering,
driving, vision-cruise, recording, and display configuration. Changes are written
atomically. Control changes are signaled to `controlsd` and also detected by
its 100 ms fallback poll.

`params/calibration.json` is also tracked as the initial calibration seed. The
runtime replaces it atomically when a stable calibration is learned, while
`scripts/upload_to_board.sh` preserves an existing runtime copy and installs the
repository copy under `params.defaults/` as a fallback. After a mount change,
let online calibration relearn it.

## Web console

Open the web console at `http://<board-ip>:8080`. It can also be started
directly, from the install directory:

```sh
cd /root/edgepilot && python3 -m web_console --host 0.0.0.0 --port 8080
```

The header shows the processes the manager runs (its `ManagerState`, a red or
amber pill when one is down or the manager stopped publishing) and the
backlight. There is one tab per parameter file, then 실시간 학습 (the learners,
calibration and steering lag) and BEV. The address keeps the tab (`#steering`,
`#bev`), so a reload stays on it, and only the tab on screen polls the board.
Parameter tabs search, show only the changed items, mark each item's default
and put it back with one press. A value is written as soon as it is changed.

> [!WARNING]
> The web console has no authentication and writes steering parameters that
> `controlsd` hot-reloads while driving. Expose it only on a trusted vehicle
> or development network.

The BEV tab shows what the model sees from above: lane lines, road edges, the
planned path and the lead, round the ego car, with the HUD's colours and limits
(`hud_renderer.cc`). The browser does all the work. The board copies the
`ModelState` and `ControlState` payloads, unparsed, into one streamed response
(`/api/bev/stream`, a frame per new model frame, 3.8 kB at up to 20 Hz). It
sleeps until the next model frame is due, so a viewer costs about 3% of one
core at nice 10, and nothing once the tab is closed or hidden. `/api/bev` says
where the fields sit. `ipc_messages.h` pins those offsets, and
`check_web_console.py` checks the two match. The page refuses to draw, and says
so, when the server does not list a field it reads. The page loads `bev.js` and three.js
from `web_console/static/`. The server gzips each file once in memory (about 250 kB in
all) and then answers with 304 while it is unchanged.

The ego car is the car's own model (`web_console/static/bev_ego.js`); the BEV knows
only its size and the function that builds it, so another car brings its own. Today
it is a black 2017 Kia K7, built in code from the published dimensions and from
measurements of Kia's studio photographs (side, front, back): its proportions,
grille, lamps, chrome, plates and wheels. Its tail lamps are lit. It costs the
browser 26k triangles in 22 meshes, and is made once when the tab opens. The lead
is the generic car (`web_console/static/bev_car.js`).

Beyond the HUD, the BEV shows the following:

- **Planned slowdown.** The inside of the path turns amber, then red, where the
  model plans to slow down. The `PLAN` chip gives the slowest planned speed and
  where it is, or `STOP`, or the speed the plan reaches when moving off. The
  speed comes from the plan's positions over openpilot's T_IDXS. Against the
  model's own velocity output it is 0.04 m/s off at 2 s and 0.07 m/s at 4 s
  (median, 2026-10-04 drives). The vision cruise does not use it.
- **Curvature arcs.** A white arc shows the curvature `controlsd` asks for and a
  cyan arc the curvature it measures. Each runs as far as the car goes in 2.5 s.
  The gap between their end ticks shows where the error would take the car. The
  `LAT` chip gives both as lateral acceleration.
- **Steering strain.** The ego halo turns amber as the steering output nears its
  limit. The HUD shows this on the path instead.
- **Lead brake lights.** They light when the model's lead decelerates at
  1 m/s² or more, and go out above −0.6 m/s².
- **Ego turn signals.** The K7's mirror repeater, the block at its headlamp's
  inner end and its tail lamp's amber lens flash, with a glow on the road, while
  `controlsd` reports a blinker, every 0.7 s like the K7's own lamps. Hazards
  light both sides.
- **Ego brake lights.** The K7's tail lamps brighten, its high brake light comes
  on and a red glow lights the road behind it, while the pedal is pressed or
  AUTO HOLD holds the car
  (`kHudFlagBrakeLights`; see [the K7 brake signals](k7-panda-port.md#brake-signals)).
- **Departure card.** While the car stands in D, a card shows the departure
  alert's inputs (`departure_alert.cc`):
  - the 2 s gas press probability against the 0.3 that fires it, with its last
    10 s;
  - the plan's x at 10 s against the ±5 m that arms the alert and the 10 m that
    fires it;
  - `ARMED`, or `GREEN` / `LEAD GO` while an alert shows.

## Production defaults

- Capture is `NV12 1280x720`, the full sensor field of view scaled (no crop),
  sensor at 20 fps.
- The frame ring has 8 CMM slots.
- `overlayd` and `recordd` start once `camerad` has run for 1.5 s, because
  opening VI resets the AX pools.
- Child process nice levels are fixed as `camerad=0`, `overlayd=10`,
  `recordd=15`, `modeld=-15`, `imud=10`, `locationd=5`, optional `pandad=-10`,
  `controlsd=-8`, and `web_console=10`.
- The front-vehicle marker is always enabled with probability threshold `0.5`.
- Desired curvature is clamped to openpilot's `0.2 1/m`; it is intentionally not
  a runtime tuning option.
