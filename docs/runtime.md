# Split runtime

[← Documentation index](../README.md)

## Managed startup

```sh
python3 /root/edgepilot/manager.py [supercombo.axmodel]
```

The manager changes to its own directory, so it can be started from anywhere.
The no-argument command selects `models/supercombo.axmodel` (or `EDGEPILOT_MODEL`),
K7 control enabled, Panda TX enabled, and the FastAPI web console enabled.
`pandad` is off unless `EDGEPILOT_ENABLE_PANDA=1`. Every setting can be
overridden with its environment variable; see
[Runtime options](runtime-options.md).

Stop it with Ctrl-C or `SIGTERM`: children get `SIGTERM`, then `SIGKILL` after
3 s.

For boot autostart, `scripts/install_autostart.sh [root@board]` installs
`scripts/edgepilot.service` (after `edgepilot-drivers.service`, which loads the
AX drivers early in boot and is installed by `scripts/install_boot_tuning.sh`,
and after `usb-gadget.service`; `Conflicts=launcher.service`; `Restart=always`)
and disables the stock
launcher at boot. The unit reads `/etc/environment` for `LD_LIBRARY_PATH` and
sets `EDGEPILOT_LOG_DIR=/run/edgepilot`, so each child's output goes to
`/run/edgepilot/<name>.log` (tmpfs, emptied past 1 MiB). `systemctl start
launcher.service` switches back to the stock UI; `install_autostart.sh
--remove` undoes the install.

It also installs `wifi-dhcp-renew.service`. The service waits for
wpa_supplicant's control socket, then renews the wlan0 DHCP lease on every
reconnect; otherwise udhcpc keeps the old network's lease. It also sets the
AIC8800 Wi-Fi driver's debug level to errors only. The stock level 1039 logs
more than 5 lines/s, which the journal writes to the SD card.

Real-time daemons must not write to the SD card from their loop. With the
recording mover busy, a small open/rename can block for seconds. modeld's
calibration save did this, stalled modeld and dropped steering. Use a
background writer, as calibration and controlsd's learners do.

The manager restarts a dead child after 1 s. When `camerad` restarts,
`overlayd` and `recordd` are restarted after it settles (1.5 s), because opening
the VI resets the AX common pools they hold. If `camerad` crashes on a signal or
fails twice within 5 s of starting, the manager restarts every process: a
crashed `camerad` leaves its VI/IVPS group and pools allocated while `modeld`
keeps the AX system open.

## Runtime processes

### `manager.py`

- stops the stock UI (`systemctl stop launcher.service`) and any app under
  `/maixapp/apps/`, which cost about 30% CPU and hold the camera and NPU, unless
  `EDGEPILOT_STOP_LAUNCHER=0`
- with `EDGEPILOT_ENABLE_PANDA=1`, switches the USB-C port to host mode
  (`/sys/class/usb_role/8000000.dwc3-role-switch/role`) and restores the
  previous role on exit; `EDGEPILOT_USB_ROLE=host|device` instead fixes the
  role at start and leaves it (the boot service sets `host`)
- starts, in this order: `camerad`, `modeld`, `imud` and `locationd` (not in
  rehearsal mode), then `pandad`, `controlsd`, and the web console when
  enabled; `overlayd` and `recordd` start once `camerad` has run for 1.5 s
  (opening VI resets the AX pools); binaries that are not installed are
  skipped
- restarts a process 1 s after it exits
- publishes `managerState` to `/dev/shm/edgepilot_manager_state` every second
- nice levels: `camerad=0`, `overlayd=10`, `recordd=15`, `modeld=-15`,
  `imud=10`, `locationd=5`, `pandad=-10`, `controlsd=-8`, `web_console=10`

### `camerad`

- opens the `ov_os04d10` through `libmaixcam_lib` (VI) at `NV12 1280x720` with
  AI-ISP on (one NPU core; `modeld` uses the other). It refuses to start unless
  `/boot/configs` has `maix_npu_ai_isp=1`, which splits the NPU at boot
- runs the sensor itself at 20 fps (`AX_ISP_SetSnsAttr`) with auto exposure on
  and the maximum shutter capped at 33,333 us (`EDGEPILOT_MAX_SHUTTER_US`), so
  frames arrive 50 ms apart as the model expects
- allocates the 8 frame-ring slots as CMM (physically contiguous) blocks and
  copies each camera frame into the next slot with IVPS TDP; the CPU never
  touches the pixels
- the ring header in `/dev/shm/edgepilot_road_ai` (ring version 5) carries each
  slot's physical address and a per-slot seqlock; only frame metadata is
  published as `roadAiFrame`
- lowers the sensor library's log level, which otherwise writes a harmless
  gain-table error on every AE update

### `overlayd`

- drives the LCD: the camera on VO video layer 0 and the HUD on the graphic
  layer (`/dev/fb0`); the hardware composes them for the 480x640 panel
- layer 0 is the camera: IVPS reads the newest ring slot, crops the centre 4:3
  (960x720 of 1280x720), and scales it to 640x480, so the preview is not
  stretched; a frame overwritten during the read is dropped
- the HUD is straight-alpha BGRA drawn by the CPU renderer in the panel's
  portrait orientation into a cached buffer (no rotation pass: IVPS TDP
  rotation smears 32-bit BGRA edges); only the 64 px tiles drawn this frame or
  last frame are cleared and copied into the visible fb0 page
- redraws the HUD when a new model, control, panda, manager, or record snapshot
  arrives and once a second, at most every 45 ms, which gives 20 Hz with the
  model; lanes and path are anti-aliased and fade with distance; the
  turn-signal animation has its own 50 ms clock
- HUD content: a border in the steering state colour (grey while engaged but
  not steering, including the large-angle pause), the speed with yellow
  turn-signal chevrons beside it (both sides for the hazard lights) and a large
  green `AUTO HOLD` badge under it while the car holds the brake, the set speed
  card (with the vision cruise speed as `SET` when it is lower) and gear card
  and steering mode chip at the top left with a manoeuvre chip below it while
  steering (`TURN LEFT/RIGHT` for turn desire, `CHANGING LANES`), a status pill
  at the top right (red `REC`
  while `recordd` writes a route, a Wi-Fi fan lit by signal, or `OFFLINE` unless
  a `wlan` link is associated with an AP; the USB link does not count), TPMS
  and camera calibration cards in the bottom corners with a pair of cards
  above them: board state above TPMS (CPU temperature, CPU, RAM, disk; amber
  from 70 °C / 90%) and the learned values above calibration (steer ratio,
  angle offset, lateral-acceleration torque factor, steering delay; white
  while control uses the learned value, dim while it still uses the
  parameter), a lane-position marker across the
  ego lane about 7 m ahead (ticks at both lane lines, the lane centre above the
  line and the car below it at true scale, the offset in cm), a steering
  torque bar at the bottom
  edge while engaged (openpilot's mici UI torque bar: sent torque / 384 from the
  center toward the turn, white, then orange above 75%; the driver's torque on
  the same scale as a light-blue tick), and an alert card above it, which also
  asks for the nudge that starts a lane change (`LANE CHANGE`) and says when the
  wheel is past 85° or held by the driver (`STEERING PAUSED`). Panda,
  storage, and a radar-only lead appear as chips only when they apply,
  and the green-light wait as a traffic-light icon (red lamp lit); the `hud_debug` device setting
  adds a card with what no other card shows: camera/model/HUD FPS, steering
  torques, paramsd stiffness and average offset, torqued raw estimates and
  progress, and lagd's blocks (amber until valid)
- reads the touchscreen (`hyn_ts`, rotated clockwise 90° like MaixCDK): a tap
  on the status pill opens a network card (SSID, IPv4, interface, signal, or
  `Not connected`, plus the USB link's address) for
  10 s and a second tap closes it, a tap on the left column (set speed and gear
  cards, chips, board state card) shows or hides the diagnostics card until the `hud_debug` setting
  changes or overlayd restarts, and a tap elsewhere closes the network card;
  every tap is logged as `overlayd: tap x=... y=... <action>`
- turns the backlight on (`/sys/class/pwm/pwmchip0/pwm3`, level from
  `/boot/configs`)
- plays the alert sounds (short bell-like tones, `src/hud/alert_tones.cc`) on the
  board speaker, shows departure
  alerts and engage refusals on screen, and writes every alert as a
  `overlayd: alert=...` log line

### `modeld`

- takes one argument, the axmodel path, and refuses any model that does not
  match the openpilot master core contract
- runs on every frame (20 Hz): the GDC (`AX_IVPS_Dewarp`, perspective) warps
  the ring slot directly from its physical address into the two 512x256 model
  views; `EDGEPILOT_WARP_CPU=1` selects the CPU warp instead
- checks after the warp that the slot was not overwritten; a torn frame is
  dropped before it enters the image and feature queues
- runs the NPU, parses the output, updates online calibration, feeds the
  calibration back into the next warp, and publishes compact `modelState`
- see [Model pipeline](model-pipeline.md)

### `pandad` (optional)

- started only with `EDGEPILOT_ENABLE_PANDA=1`, when built with
  `-DEDGEPILOT_BUILD_PANDA=ON` (the default)
- connects to the Panda over `libusb`, publishes compact Panda health and
  ordered CAN receive batches, and can relay ordered `sendcan` batches
- standalone default is shadow mode (`EDGEPILOT_PANDA_TX=0`); the manager's default
  enables TX
- reads the Panda's firmware version when it connects and, once a second, writes
  the link, that version and any flashing progress to
  `/dev/shm/edgepilot_panda_status.json` for the web console
- flashes the Panda when the web console asks: `/dev/shm/edgepilot_panda_flash`
  holds the version to write, and `pandad` writes the installed image
  (`EDGEPILOT_PANDA_FIRMWARE`) only if it is that version and the car is parked.
  See [Panda firmware](../firmware/panda/README.md)

### `controlsd` (K7 controller)

- enabled by default (`EDGEPILOT_ENABLE_CONTROL=1`)
- runs the openpilot-compatible lane planner and lateral MPC in a worker,
  with the KIA K7 YG HEV torque controller and `LKAS11`/`CLU11`/`MDPS12` packer
  at 100 Hz
- consumes model path, lane, road-edge, and vehicle-state IPC
- uses the vision lead distance and relative speed to adjust the stock
  fixed-speed cruise setting with rate-limited `SET-`/`RES+` CLU11 pulses; the
  first driver SET speed remains the maximum. Closing distance is projected
  through the measured 1.5 km/h/s vehicle response, repeated `SET-` pulses wait
  for that response, and `RES+` cannot immediately reverse a recent slowdown
- publishes generated raw `sendcan` batches for `pandad`
- publishes compact `controlState` diagnostics for the display HUD
- does not transmit by itself; actual TX still requires `pandad` with
  `EDGEPILOT_PANDA_TX=1`

### `imud`

- reads the board IMU (ST LSM6DSOW on `i2c-1`, address `0x6B`) through
  `i2c-dev`: accelerometer and gyroscope at 104 Hz, ±4 g and ±250 dps. It does
  not use libmaixcam_lib, which has no IMU class, or MaixPy, whose import remaps
  the UART4 pins
- publishes batches of raw samples to `/dev/shm/edgepilot_imu` every 100 ms.
  The axes are the chip's and the gyro bias is not removed; `recordd` records
  them as `Imu` records (`recording_reader.read_route_imu`)
- feeds `locationd`, whose pose filter gives paramsd and torqued their yaw
  rate and roll (`use_locationd_learner_inputs` in `params/steering.json`) and
  whose lagd estimate can set the steering delay (`use_live_delay`). Without
  the IMU, `locationd` publishes nothing and `controlsd` falls back to ESP12
  values. If the IMU cannot be opened it retries every 10 s instead of
  exiting, so a missing IMU neither loops the manager nor marks the runtime
  unhealthy. It is not started in rehearsal mode

### `locationd`

- runs the openpilot locationd pose filter on the IMU batches, the model's
  camera odometry (`ModelState` pose) and `controlState`, and the lagd
  steering-delay estimate (`src/localization/localization_pipeline.*`)
- publishes `LocalizationState` to `/dev/shm/edgepilot_localization`; with no
  IMU input it publishes nothing and waits
- saves the lagd estimate to `params/live_delay.json` every 60 s and on exit,
  and resumes it at start when `steer_actuator_delay` is unchanged. It is not
  started in rehearsal mode

### `recordd`

- follows the frame `modeld` used (`/dev/shm/edgepilot_record_frame`), copies
  the ring slot into its own pool block with IVPS, checks the slot was not
  overwritten meanwhile, and encodes it with the hardware H.264 encoder (VENC,
  CBR, 1 s GOP) — the CPU never touches pixels (~5% of a core while recording).
  H.264 because the board's decoder only decodes H.264, so a route can be
  replayed as recorded ([Rehearsal](rehearsal.md))
- writes the recording format with the codec in the manifest
  (`segments/NNN/road.h264` + `frames.bin`,
  event log with CAN, model, control, panda, learner, IMU and localization
  state, params snapshot), staged in tmpfs and moved to `recordings/` on the SD card
- `params/recording.json` `enabled` starts/stops a route; `bitrate_bps` is read
  once, when `recordd` starts
- publishes `recordState` (route being written, storage reserve exhausted)
  twice a second; `overlayd` shows `REC` or a `STORAGE FULL` chip from it

### `replayd` (rehearsal)

- with `EDGEPILOT_REPLAY_ROUTE`, takes the place of `camerad` and `pandad`: it
  decodes the route's H.264 into the frame ring with the hardware decoder and
  replays its CAN and panda state in real time, while the other processes run
  as in the car. Generated CAN is logged, never sent. See
  [Rehearsal](rehearsal.md)

## Measured load

CPU per process on the board with the full camera/model/HUD/control pipeline
(one core = 100%): `camerad` ~11% (mostly the ISP 3A thread), `overlayd` ~14%,
`modeld` ~8%, `controlsd` ~1%. Camera, model, and HUD all run at 20 Hz with no
missed model frames.

## Recording format

`recordd` writes the routes the host tools read (`recording_reader.py`, the
replay tools, `lane_bias.py`). `gtest_recording` covers
`src/recording/recording_writer.*`.

The event log is written as 60 s chunks in `events/NNN.bin`, each starting with
an 8-byte `EDGELOG1` magic, a version word, and fixed 16-byte record headers;
each segment's `frames.bin` starts with `EDGEIDX1`. The current version is `9`,
which only changed those two magics: recordings up to version 8 carry the
earlier ones, and every reader still accepts them. Version 8 gave `ModelState`
the model's pedal predictions
(`gas_press_probs`, `brake_press_probs`: the chance the driver presses the
pedal 0, 2, …, 10 s ahead, 3576 B), which the departure alert reads.
Version 7 added the camera mount the warp used for that frame
(`camera_offset_m`, `camera_height_m`, 3528 B), so the HUD and the analysis
tools use exactly what modeld applied. Version 6 added the plan yaw and yaw
rate (`plan_yaw`, `plan_yaw_rate`, 3520 B) that laneless mode steers from.
Readers take version 7 and older payloads as before, with the missing fields
zeroed (`src/recording/recorded_model_state.h`, `recording_reader.model_state_layout`).

| Record type | Payload |
| --- | ---: |
| `CanRx` / `CanTx` | variable CAN batch |
| `ModelState` | 3576 B |
| `ControlState` | 240 B |
| `PandaState` | 96 B |
| `LearnerState` | 128 B |
| `Imu` | 16 B + 40 B per sample (about 10 samples every 100 ms) |
| `Localization` | 128 B |

Older recordings are not `ModelState`-compatible: version 1 carried 4384 B
including unused lateral draft fields, versions 2–3 carried 4080 B including the
stop-line block that openpilot v0.9.4 does not emit, and version 4 carried
4048 B including plan position stds and orientations that nothing read.
Version 2 also kept a single route-level `events.bin`; CAN logging alone
(~0.5 MB/s) filled the 988 MB tmpfs staging in about 30 minutes on long drives
and silently killed the rest of the recording, which is why version 3 rotates
event chunks alongside video segments. `tools/model/recording_reader.py` reads
the v2 to v9 layouts; `lane_bias.py`, `hud_tools.py`,
`fit_lateral_params.py lag`, and `export_can_fixture.py` all walk the event log
through its `iter_event_records`.

## IPC boundaries

Camera frames never go through the small-message IPC. The frame ring's pixels
live in camerad's CMM blocks and are read by physical address: the GDC in
`modeld` and IVPS in `overlayd` read the slot directly, and a CPU
reader (the CPU warp fallback) maps it uncached. Only the ring header and
per-frame metadata are in `/dev/shm`. This keeps the split runtime close to
openpilot's process boundaries without paying the cost of Cap'n Proto/cereal.

The lateral plan has a single producer: the lateral MPC inside `controlsd`.
`modelState` carries perception output only.
