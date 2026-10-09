# Source layout

[← Documentation index](../README.md)

Runtime code lives in `src/<subsystem>/`, one folder per subsystem. Each
folder's `CMakeLists.txt` defines its libraries (built on the host too, so the
tests and replay tools use the same code) and, in the board build, the process
it runs. Headers are included by their path from `src/`
(`#include "controls/adaptive_cruise.h"`) and the board wrappers by their path
from `platform/` (`#include "maixcam2/maix_display.h"`); those are the only two
include roots.

```
src/
  common/        configuration, shared memory, message and model-output formats, helpers
  car/           K7 CAN: address table, received-frame decoding, sent-frame encoding
  panda/         the panda USB bridge (pandad)
  controls/      controlsd: lateral controller, vision cruise, departure alerts, the tick
  planning/      lateral planner, desire helper, lateral MPC
  learners/      paramsd and torqued (run inside controlsd)
  localization/  locationd and lagd, the board IMU reader (imud)
  model/         modeld: NPU session, input warp, online calibration, ModelState packing
  camera/        camerad and the camcal still capture
  hud/           the HUD (overlayd): state mapping, drawing, alert policy and sounds
  recording/     recordd and replayd, the recording format and its readers
platform/maixcam2/   AX and MaixCDK wrappers (board build only)
```

Libraries link downward only: `common` ← `car` ← `control_core` ←
`planning`, `learners` ← `controls`. `localization`, `model`, `hud_state` /
`hud`, `recording` and `panda` build on `common`. `controls/` holds two
libraries because the planner and the learners sit between the controller
(`control_core`) and the tick that drives them (`controls`). A few headers are
read across that order for their enums, constants and default values only:
`panda_firmware.cc` reads `car/can_frame.h`, the HUD reads
`controls/departure_alert.h`, `controls/control_block.h`,
`controls/control_params.h`, `car/can_frame.h`, `model/calibration_online.h`
and `localization/lateral_lag.h`, and `controls_tick.cc` reads
`localization/lateral_lag.h`. Those uses stay header-only; calling a function
defined in one of those libraries means linking it.

## Naming

- Code, folders and scripts are snake_case; documents are kebab-case. C++
  headers are `.h`, sources `.cc`, generated data `.inc`.
- File names read `<subject>_<role>` (`recording_writer`, `lateral_planner`)
  and are unique across the repository, so comments and documents can name a
  file alone. Folders do not strip the prefix. Prefix families (`utils_*`,
  `maix_*`) and one-word programs (`projection`, `manager.py`) are fine.
- A C++ process's main is `<name>d.cc`, named like its executable; the deploy
  scripts and `manager.py` start it by that name. `camcal`, the intrinsics
  capture that `camcal.service` runs while the runtime is stopped, keeps its
  tool name.
- Tests are `tests/gtest_<module>.cc`, or `gtest_<folder>.cc` when they cover a
  folder. A module can be a family of files on one subject: `gtest_calibration`
  covers `calibration_online`, `calibration_service` and the model input warp.
- The HUD module is `hud` in files, types and libraries. Only `overlayd`, the
  process, and the display's hardware overlay layer keep "overlay".
- Names something else fixes keep their form: files tools look for
  (`CMakeLists.txt`, `README.md`, `Dockerfile`), systemd units and the scripts
  installed under their names (`edgepilot-drivers.service`,
  `wifi-dhcp-renew.sh`), `requirements-web-console.txt`, and vendored
  third-party files (`scripts/web_console/static/three/`, the panda tree in `firmware/panda/`).

## src/common

Libraries `common` and `utils_json`.

- `app_config.*`
  - parses the small runtime option set once at startup, and holds the
    MaixCAM2 camera intrinsics, the capture size, and the 4:3 preview crop
    shared by `overlayd` and `projection`.
- `ipc_messages.h`
  - every message that crosses `/dev/shm`: topic names, magics, channel headers,
    the state snapshots (`ModelState`, `ControlState`, `PandaState`, …) with
    their `static_assert`s. Recording v9 stores the state snapshots as-is, so
    their layouts are pinned here and tied to `kRecordingVersion` (that assert
    sits in `state_recorder.cc`). Code that
    only reads or fills a message includes this and nothing else.
- `ipc_channels.*`
  - the `/dev/shm` channel implementations: latest-message channel, CAN queue,
    and the camera frame ring, all on one `ShmRegion` (open, size, map, close).
    `Subscription<T>` wraps a latest-message channel with the last snapshot
    and its sequence number, replacing it only after a complete read.
    The frame ring (version 5) keeps only its header in shm; the slots are
    camerad's CMM blocks, listed by physical address, each with a seqlock that
    hardware readers check before and after reading.
- `model_output.*`
  - owns the openpilot master supercombo raw-output layout
    (`model_output_layout`, every block offset with a `static_assert`) and
    exposes parsed plan, lanes, road edges, leads, pose, and the meta pedal
    predictions (the chance the driver presses gas or brake 0, 2, …, 10 s
    ahead; the departure alert uses gas at 2 s). Also owns the shared
    `T_IDXS`/`X_IDXS` trajectory grids and `kLeadProbabilityThreshold`, the
    one lead probability cut (0.5, as openpilot's radard) for the vision
    cruise, the departure alert, the HUD and the web BEV.
- `projection.*`
  - converts model road coordinates through the openpilot-style `view_from_calib`
    matrix onto the display, using the target's real width and the same 4:3
    preview crop as the video layer.
- `device_settings.h`
  - the web device settings (`params/display.json`) the runtime reads: modeld
    (camera mount) and overlayd (alert volume) poll it from their main loops.
- `utils_process.h`
  - what a process gets from the OS: environment variables (`env_flag` is the
    one boolean convention), the `params/` directory path, and the
    SIGINT/SIGTERM → stop-flag hookup used by every `*d` main.
- `utils_math.h`
  - clamping, openpilot `interp`, degree/radian conversion.
- `utils_time.h`
  - `monotonic_now_ns` (`CLOCK_BOOTTIME`), the clock behind every timestamp that
    crosses a process boundary, and the freshness predicates for ns and
    CAN-seconds timestamps. Per-process scheduling may still use
    `std::chrono::steady_clock`.
- `utils_json.*`
  - minimal JSON value readers, the clamped `parse_json_optional_*` helpers, and
    the `Json*Field` tables that `control_params` and `adaptive_cruise` fill
    their structs from: one `{key, min, max, member}` row per parameter.
- `utils_file.h`, `background_writer.h`
  - `file_stamp` (the stat fingerprint the processes poll parameter files
    with), `read_text_file`, and `write_file_atomic` (temporary file + rename,
    so a reader never sees a half-written file). `BackgroundWriter` is the
    thread controlsd (learner files) and locationd (lag cache) hand those
    writes to: the newest content per path wins, failures are counted, and
    the queue drains on shutdown.

## src/car

Library `car`.

- `can_frame.h`, `vehicle_can.*`, `hyundai_can.*`
  - `can_frame.h` holds the transport type and the K7 YG HEV address/bus table;
    `vehicle_can` decodes received frames into vehicle state, `hyundai_can`
    encodes LKAS11/CLU11/MDPS12 commands. The CAN layer does not see the
    controller's parameters; the controller converts its torque limits.
- `speed_filter.*`
  - openpilot's `vEgo`: the wheel-speed average through the speed Kalman
    filter of opendbc `CarStateBase.update_speed_kf`, run once per control
    tick.

## src/controls

Libraries `control_core` and `controls`, process `controlsd`.

- `controlsd.cc`
  - the controlsd process: opens the channels, reads them each tick in the
    order `ControlsTick` documents, publishes, sends, writes the learner files
    on a background thread, and logs the one-second stats line.
- `controls_tick.*`
  - one 100 Hz controlsd tick without shared memory or files: CAN, model, Panda
    and locationd inputs in; the lateral controller, departure alerts, vision
    cruise and learners in between; the frames to send, `ControlState` and
    `LearnerState` out. The 20 Hz planner sits behind `PlannerPort`: a worker
    thread on the board (`LateralPlannerWorker`), computed in place in tools
    and tests (`SyncPlanner`). It also holds the parameter-file watcher.
- `lateral_controller.*`, `lateral_torque.*`, `control_params.*`
  - apply the planner's lag-adjusted curvature through the validated K7
    torque/CAN path (`car/hyundai_can`).
- `lateral_target.h`
  - declares `LateralTarget`, the planner-to-controller interface.
- `lateral_path.*`
  - reduces `modelState` to the steering-usability gate (reach and point
    count). It computes no path geometry; curvature comes from the MPC.
- `adaptive_cruise.*`, `departure_alert.*`
  - vision cruise setpoint control and departure alerting. The cruise
    controller dead-reckons the car's set speed per session (this car does not
    report it), re-anchors to the cluster speed once the driver's buttons
    settle and after a long mismatch, and paces SET-/RES+ pulses from a
    filtered vision lead and the learned cluster/wheel speed ratio. The
    departure detector runs three trackers per stop: close lead, lead
    departure and green light.
- `control_block.h`
  - the engage/steer block reasons as one table: enum, wire name, HUD label,
    and kind (reject / hard disengage / transient Panda handshake /
    availability). The controller decides in `BlockReason`, `ControlState`
    carries the wire name so recordings and the Python readers stay text, and
    `hud_state` labels it from the same rows. `gtest_hud_state` proves
    every reason has a label.
- `control_holds.*`
  - the two control safety holds, below.

### Control safety holds

`control_holds.*` implements both holds as `PandaHealthGate` and
`PathHoldGate`; `gtest_control_holds` exercises their boundaries.
`controlsd` tolerates a single malformed plan frame by holding the last
usable path for at most 150 ms; the normal 250 ms model freshness timeout remains
a hard safety gate, so a stale or invalid model still removes control. A
transient Panda health-snapshot gap is similarly limited to 100 ms; a fresh,
transport-ready `controls_allowed=0` is never held. Other health faults are
released after that short hold if they persist.

## src/planning

Library `planning`.

- `lateral_planner.*`
  - applies openpilot lane probability/width logic and the lateral MPC
    (Lane mode), or the plan yaw (Laneless mode and Lane mode's model-path
    stretches), to produce curvature targets. This is the only producer of
    `LateralTarget`.
- `desire_helper.*`
  - the openpilot desire_helper port the planner runs each model frame: lane
    change states (`LaneChangeState`), the blinker/torque/blind-spot/road-edge
    gates, the lane-line fade, and the `Desire` the model gets.
- `lateral_mpc.*`
  - the lateral MPC itself: one Gauss-Newton SQP iteration per call over the
    openpilot 0.8.16 OCP, solved by a backward Riccati recursion. No external
    solver. See [Verification](verification.md#lateral-mpc-solver).

## src/learners

Library `learners`.

- `vehicle_params_learner.*`, `torque_estimator.*`, `lateral_learners.*`,
  `localizer_inputs.h`
  - the paramsd/torqued ports that estimate steer ratio and torque response
    while driving (`use_live_vehicle_params`, `use_live_torque_params`, both on
    by default). `vehicle_params_learner` is paramsd with its
    car_kf EKF and saved-value restore; `torque_estimator` is torqued with its
    cache; `lateral_learners` is the controlsd glue that turns vehicle CAN and
    locationd samples into learner inputs and collects the outputs as
    `LiveLateralParams`. `localizer_inputs.h` converts the IPC
    `LocalizationState` into a learner sample, so the learner library does not
    depend on the IPC layout. "Localizer" is the learners' word for locationd
    input throughout, as in upstream paramsd.

## src/localization

Library `localization`, processes `imud` and `locationd`.

- `location_estimator.*`, `lateral_lag.*`, `localization_pipeline.*`
  - ports of openpilot locationd (the 18-state pose EKF over the board IMU and
    the model's camera odometry) and lagd (the steering delay from desired vs
    actual lateral acceleration), and the pipeline that feeds them in time
    order. `locationd` and `replay_localization` share it.
- `imud.cc`, `locationd.cc`
  - the board IMU reader (LSM6DSOW over `i2c-dev`) and the process that runs
    `localization_pipeline` on its samples and publishes `LocalizationState`.
    It hands the lag cache to `BackgroundWriter`, so an SD stall never holds up
    the IMU loop.

## src/model

Libraries `model` and (board) `replay`, process `modeld`.

- `modeld.cc`
  - the model process: takes the newest ring frame, runs one frame, and
    publishes `ModelState`.
- `supercombo_model.*`
  - loads the axmodel, enforces the input/output contract, runs the GDC (or
    CPU) warp into the image histories, fills the temporal inputs, and runs one
    frame. `run_frame_phys` reads a ring slot by physical address and drops the
    frame if it was overwritten during the warp.
- `ax_engine_session.*`, `ax_engine_api.h`
  - a minimal `libax_engine` session with a cached CMM buffer per tensor. The
    board image ships no engine headers, so `ax_engine_api.h` declares the API.
    The only files under `src/` that touch the NPU.
- `model_output_assembly.h`
  - reassembles the split-head axmodel outputs into the openpilot 2576-float
    layout (`model_output.h`); each head is quantised over its own range.
- `model_temporal.h`
  - the history queues the NPU core does not carry: 100-tick desire pulses
    pooled to 25x8, 96 ticks of hidden state strided to 24x512, and the
    5-frame image history per tower. No engine dependency, so
    `gtest_model_output` pins the convention on the host.
- `model_input_transform.*`
  - the CPU input warp: direct `NV12 -> calibrated warped YUV6`, fusing
    homography sampling and YUV6 packing through a compact fixed-point LUT.
    Also produces the projection matrices the GDC warp uses.
- `calibration_service.*`, `calibration_online.*`
  - wrap pose-based online calibration, manual override, projection policy, and
    the model-input calibration feedback loop.
- `model_state_fill.*`
  - `fill_model_state`, which modeld calls to pack a frame's outputs into
    `ModelState`, and `compute_lane_t` (the openpilot plan→lane time mapping).
    It needs the online calibrator's snapshot, so it lives in the `model`
    library and message consumers do not pull in the calibrator.
- `replay_source.*`
  - reads `SCNV12R1` replay files into `Nv12Frame` for `modeld` replay
    mode. POSIX only.

## src/camera

Processes `camerad` and `camcal` (board build only).

- `camerad.cc`
  - captures into the frame ring for `modeld`, `overlayd` and `recordd`.
- `camcal.cc`
  - still capture through the runtime's camera path for the intrinsics
    measurement ([Camera calibration](camcal.md)).

## src/hud

Libraries `hud_state`, `hud` and `alert_tones`, and on the board
`alert_sound`; process `overlayd`.

- `overlayd.cc`
  - the two-layer LCD HUD: the camera preview on the video layer, the HUD
    drawn onto the graphic (overlay) layer, alert sounds and the touchscreen.
- `hud_renderer.*`, `hud_scene.cc`, `hud_cards.cc`, `hud_draw.h`,
  `hud_canvas.*`, `hud_font.*`
  - draw the 640x480 HUD into a straight-alpha BGRA buffer (landscape
    coordinates; `HudOrientation` maps them onto the portrait panel buffer):
    state border, speed with yellow turn-signal/hazard chevrons, set speed
    (with the vision cruise `SET` speed) and gear cards, steering mode and the
    lane change in progress, an `AUTO HOLD` badge
    under the speed, plan/lane/road-edge ribbons faded with distance, the
    car's position in its lane marked on the road, lead chevron, torque bar
    with the driver's torque, alerts (including the lane-change nudge and the
    large-angle pause), TPMS and camera calibration cards with the board
    state card above TPMS and the learned values card above calibration
    (white while control uses them), a
    recording/Wi-Fi status pill (and the network card a tap opens), and chips
    that appear only when something needs attention (panda, storage). The
    numbers no other card shows sit in a card behind the `hud_debug` device
    setting. `hud_renderer` lays out the frame and the top and bottom-edge
    widgets, `hud_scene` draws the road scene (ribbons, lead chevron, lane
    position), `hud_cards` the cards, and `hud_draw.h` holds what they
    share (design tokens, text placement, state colours). `hud_canvas`
    fills polygons with 4-subrow anti-aliasing that touches only covered spans,
    fills the straight rows of integer rounded rectangles directly, blends
    without divisions, and records the 64 px tiles each row touched so the
    next use of the same buffer clears only those; `hud_font` holds the
    glyphs that `tools/ui/make_hud_font.py` bakes from Pillow's Aileron (CC0).
    No OpenCV, so `gtest_hud_canvas` and `hud_snapshot` run on the host.
    The renderer keeps only coverage scratch and the per-buffer tiles; the
    turn-signal phase and the card toggles come from `hud_policy`.
- `hud_state.*`
  - `HudState`, the IPC state (`ControlState`, `ModelState`, …) →
    `HudState` mapping shared by
    `overlayd` and `hud_snapshot`, the `ModelState` →
    `ParsedModelOutput`/`ProjectionState` unpacking, the engage-block label
    table, and `hud_select_alert`, the one alert card (`HudAlertCard`) a HUD
    state shows (its priority, text and severity; the renderer only colours
    it). `gtest_hud_state` pins it on the host.
- `hud_policy.*`
  - what `overlayd` decides besides drawing, without the screen, speaker or
    touch device: `HudAlertEvents` turns the controlsd event counters into
    one alert a frame (baseline on first sight, rebaseline on a controlsd
    restart, reject > engage > disengage > departure), `HudAlertPolicy`
    picks the one sound a frame plays (a take-control edge waits for a quiet
    frame, an available → unavailable transition is skipped in a frame that
    already sounded) and holds the engage-reject toast for 3 s,
    `TurnSignalClock` steps the blinker animation from the time the blinkers
    changed, `HudTouch` opens and closes the network and debug cards, and
    `smooth_lane_center_offset` filters the lane-position readout.
    `gtest_hud_policy` pins it on the host.
- `alert_tones.*`, `alert_sound.*`
  - the alert sounds: `alert_tones` synthesises them (overlapping bell-like
    notes with soft attacks and decaying overtones; portable, so
    `alert_sound_preview` writes them as WAV and `gtest_alert_tones` checks
    them on the host), and `alert_sound` streams them to one long-lived
    `aplay` on the board speaker (`overlayd` and `camcal`).
- `system_monitor.*`
  - `/proc`, thermal-zone, and network sampling (the Wi-Fi SSID through the
    `SIOCGIWESSID` ioctl) into `HudState`, called at 1 Hz by `overlayd`.
    Only a `wlan` link with an address and an associated SSID counts as
    connected; another link (the USB virtual Ethernet, which always has an
    address) is kept apart for the network card.

## src/recording

Library `recording`, processes `recordd` and `replayd`.

- `recording_format.h`, `recording_writer.*`, `state_recorder.*`
  - the event-log writer and on-disk contract that `recordd` writes.
    `StateRecorder` copies each new state snapshot (model, control, Panda,
    learner, IMU, locationd) into the log, attaching to a channel once its
    producer has created it. `gtest_recording` pins the layout and the
    state recording; `recording_format.h`
    (`kRecordingVersion`, the `EDGELOG1` / `EDGEIDX1` headers and the earlier
    magics that recordings up to v8 carry, record types) is mirrored by
    `tools/model/recording_reader.py`.
- `event_log_reader.h`
  - the one C++ reader of `events/NNN.bin`, shared by `replayd` and the replay
    and dataset tools. It checks the magic, skips `header_size`, and stops
    without resyncing at a truncated tail (record type out of range, a payload
    over 1 MiB, or a short header or payload), which `truncated()` reports.
- `recorded_can.h`, `recorded_model_state.h`
  - the recorded CAN payload (written by `recordd`, read back by `replayd` and
    the tools, whole frames only) and the reader that turns a v3 to v9
    `ModelState` payload into the current struct.
- `recordd.cc`
  - the drive recorder: encodes the frames `modeld` used and writes the CAN
    and state channels through `recording_writer` and `StateRecorder`.
- `replayd.cc`, `replay_route.*`
  - rehearsal: plays a recorded route in place of `camerad` and `pandad`
    ([Rehearsal](rehearsal.md)). `ReplayRoute` reads the route (frame
    indexes, the frames with their parameter sets, and the CAN/Panda events
    in time order) without the decoder, so `gtest_recording` reads back
    a route the writer made.

## src/panda

Library `panda`, process `pandad`, tool `panda_flash`.

- `panda_client.*`, `panda_can_codec.*`, `pandad.cc`
  - optional panda USB bridge. It handles USB, health, heartbeat, receive CAN,
    and the final TX gate, but does not generate vehicle control messages.
    `pandad` also flashes the Panda when the web console asks.
- `panda_protocol.h`
  - what `firmware/panda` speaks over USB: ids, request codes, packet versions,
    the health packet and the application flash layout. `gtest_panda_firmware`
    checks it against the firmware sources.
- `panda_firmware.*`
  - application image checks, the parked-car condition for flashing (it reads
    `car/can_frame.h` for `kGearPark`), and the status JSON. No USB, so the host
    tests build it.
- `panda_flasher.*`, `panda_flash.cc`
  - the bootstub flasher (board only) and the command-line tool that runs it
    with the runtime stopped.

## firmware/panda

The Panda's firmware for the STM32F413 boards, imported from openpilot_c2 with
upstream's file names (`board/`, `crypto/`, `certs/`). `make -C firmware/panda`
builds and signs it; `panda_version.py` names a build by its sources. See
[Panda firmware](../firmware/panda/README.md).

## platform/maixcam2

`platform/maixcam2/` keeps the AX and MaixCDK headers out of `src/`. It builds
only in the board build, against `deps/ax630` from
`scripts/fetch_maixcam2_sdk.sh`, as the `maixcam2_media`, `maixcam2_gdc`,
`maixcam2_venc` and `maixcam2_vdec` libraries.

- `maix_camera.*`
  - opens the camera (VI) through `libmaixcam_lib`'s `ax_middleware` classes,
    AI-ISP always on, sensor at the requested fps with auto exposure and a capped
    shutter, and copies each frame into a CMM block by IVPS TDP.
- `maix_display.*`
  - the LCD: VO video layer 0 takes a CMM frame, IVPS crops the centre 4:3 and
    scales it to 640x480, and VO rotates it for the 480x640 panel with the
    board's flip/mirror. The HUD goes to the graphic layer: it is drawn in the
    panel's portrait orientation into a cached 480x640 BGRA buffer, and only the
    64 px tiles drawn this frame or last frame are copied into the visible page
    of `/dev/fb0`. MaixCDK's layer-1 push rotates with IVPS TDP, which smears
    G/R/A into the next pixel for 32-bit BGRA (stair-stepped, fringed edges), so
    the HUD does not use it. Also turns the backlight on.
- `maix_touch.*`
  - the touchscreen (`hyn_ts`, multi-touch type B) read without blocking;
    reports short taps in screen coordinates, rotated clockwise 90° as
    MaixCDK's `maix_touchscreen_maixcam2.hpp` does.
- `maix_cmm.*`
  - physically contiguous CMM blocks shared across processes (allocate in one,
    map by physical address in another, uncached).
- `maix_gdc_warp.*`
  - the model input warp on the IVPS GDC (`AX_IVPS_Dewarp`, perspective): two
    512x256 views from a physical source address, unpacked to YUV6. Uses only
    MSP SDK headers, not the middleware, so the NPU process initialises AX SYS
    once.
- `maix_venc.*`, `maix_vdec.*`
  - the hardware H.264 encoder and decoder. `recordd` copies a ring slot into
    the encoder's own pool block with IVPS before encoding; `replayd` decodes a
    recorded frame into a ring slot. Both use only MSP SDK headers.
- `maix_shim.cc`, `stub/`
  - the few MaixCDK runtime pieces (log, err, board config lookup) and Kconfig
    stubs that the inline code in `ax_middleware.hpp` references but
    `libmaixcam_lib` does not export.

## Board scripts

- `scripts/manager.py`
  - minimal supervisor and heartbeat publisher. It is intentionally not a full
    openpilot manager clone. It stops the stock launcher, switches USB-C to host
    for the Panda, and one table in start order decides which processes run
    (`EDGEPILOT_ENABLE_CONTROL`, `EDGEPILOT_ENABLE_PANDA`, `EDGEPILOT_ENABLE_WEB_CONSOLE`) and
    with what nice value.
- `scripts/web_console/`
  - the web console (`EDGEPILOT_ENABLE_WEB_CONSOLE`, `python3 -m web_console`).
    `console_server.py` is the FastAPI app and its routes; each card or tab has
    its own module behind it: `param_store.py` and `param_metadata.py` (the
    parameter files, their labels, sections and ranges), `process_status.py`
    (the manager's process table, and signalling a process to reload),
    `learner_monitor.py` (learner and localization state), `calibration_reset.py`,
    `panda_update.py` (the Panda firmware card), `bev_stream.py`,
    `backlight.py` (the MaixCAM2 backlight, PWM3) and `static_assets.py`.
    `shm_channel.py` reads the shared-memory channels and `state_layout.py` holds
    the ModelState and ControlState offsets they share.
- `scripts/web_console/static/`
  - the page, plain ES modules with no build step: `index.html`, `console.css`,
    `console.js` (header, tabs, routing), `ui.js` (DOM helpers, cards, toasts,
    API calls, polling), `param_state.js`, `param_editor.js` (the controls,
    saving and the item card) and `param_view.js` (the parameter tabs),
    `vehicle_view.js` (차량 특성: learned and manual values side by side),
    `panda_card.js`, and the BEV tab: `bev_view.js`,
    `bev.js` (three.js view, ported from sv_recorder_bev), `bev_ego.js` (the
    ego car; today a black 2017 K7 made in code), `bev_car.js` (the lead's car
    model), `bev_data.js` (reads the ModelState and ControlState bytes the
    console streams), and three.js 0.186.1 in `three/`.

## Tools and tests

- `scripts/fetch_maixcam2_sdk.sh`, `tools/docker_ax630/`,
  `scripts/upload_to_board.sh`, `scripts/run_host_tests.sh`
  - pinned SDK and board-library fetch, the arm64 build container, deploy, and
    host tests. See `scripts/README.md` and
    [Build and deploy](build-and-deploy.md).
- `tools/model/axmodel/`
  - the openpilot master → axmodel pipeline (core extraction, output split,
    calibration and evaluation data, the board evaluation runner, Pulsar2
    config).
- `tools/model/`
  - the recording readers: `recording_reader.py` decodes `recordd` routes
    (frame index, event log, H.264 video) and mirrors
    `recording_format.h` / `ipc_messages.h` for the analysis tools;
    `lane_bias.py` and `make_replay.py` build on it. See
    `tools/model/README.md`.
- `tools/camera/`
  - intrinsics from `camcal` captures (`calibrate_intrinsics.py`), the TV
    checkerboard generator and the measured MaixCAM2 intrinsics (see
    [Camera calibration](camcal.md)), the IMU-to-camera extrinsic estimate,
    the model-view preview and the 12x7 chessboard image.
- `tools/control/`
  - `fit_lateral_params.py` (torque regression and actuator-lag estimate from
    drives, both on the wheel speed as openpilot's torqued and lagd use
    `vEgo`) and `export_can_fixture.py` (recorded CAN → `gtest_lateral_controller`
    fixture).
- `tools/ui/hud_tools.py`
  - extracts `hud_snapshot` inputs from a route and composes its frames.
- `tests/`
  - host unit tests (`gtest_*.cc`, googletest + CTest), one self-contained file
    per target, each registered by one `add_host_test(<name> <libraries>)` line in
    `tests/CMakeLists.txt`, and the Python checks `check_web_console.py` and
    `check_recording_reader.py`, registered beside them;
    `scripts/run_host_tests.sh` runs them all. See `tests/README.md`.
- `diagnostics/`
  - replay and HUD tools (built by `diagnostics/CMakeLists.txt`); see
    `diagnostics/README.md`.
