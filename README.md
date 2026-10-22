<h1 align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/images/brand/edgepilot-lockup-dark.svg">
    <img src="docs/images/brand/edgepilot-lockup.svg" alt="edgepilot · MaixCAM2" height="72">
  </picture>
</h1>

<table>
  <tr>
    <td width="50%" align="center" valign="top"><img src="docs/images/drive-day.webp" width="100%" alt="Daytime drive in lane mode at about 60 km/h behind a lead car: the HUD shows the green lane-mode path, the lead's distance and the car a few centimetres from the lane centre as the road turns into a right-hand curve"><br><sub><b>Day</b>: about 60 km/h behind a lead car, from a straight into a 250 m right-hand curve</sub></td>
    <td width="50%" align="center" valign="top"><img src="docs/images/drive-night.webp" width="100%" alt="Night drive in lane mode at about 77 km/h: the HUD shows the green lane-mode path and the car within about 10 cm of the lane centre, under overpasses and into a right-hand curve"><br><sub><b>Night</b>: about 77 km/h under overpasses, from a straight into a 240 m right-hand curve</sub></td>
  </tr>
</table>

<p align="center">
  <strong>openpilot perception and lateral control, native on a Sipeed MaixCAM2 (AX630C)</strong><br>
  KIA K7 YG HEV · supercombo on the AX630C NPU · Panda USB/CAN · 640x480 driving HUD<br>
  <sub>Both clips are recorded drives steering in lane mode, with the HUD redrawn from the logged model,
  control and learner states by the runtime's own renderer, as <code>overlayd</code> shows it on the
  board's screen. The board load card holds typical values; the recording does not log them.</sub>
</p>

| Board | Vehicle | Model | Control | Runtime |
| --- | --- | --- | --- | --- |
| Sipeed MaixCAM2 (AX630C, 1 GB) | KIA K7 YG HEV | openpilot master supercombo (Pulsar2 axmodel) | lateral (LKAS torque) | C++17 split processes |

edgepilot runs openpilot's driving model and lateral control on small embedded
boards. This `main` branch targets the MaixCAM2; the [`k230`](../../tree/k230)
branch keeps the earlier Kendryte K230 port.

> [!WARNING]
> This is experimental vehicle-control software. Keep Panda safety enabled, run
> in shadow mode first, and validate every change in a controlled environment
> before road use.

## Highlights

- **supercombo on the AX630C NPU.** The openpilot master `driving_supercombo`
  core, compiled with Pulsar2 6.0 (U16 activations, uint8 image inputs), runs in
  about 15.5 ms on one NPU core (the other core runs the AI-ISP denoiser); a
  whole `modeld` frame is about 19.5 ms, inside the 20 Hz budget. The history queues the released ONNX keeps in-graph
  (images, desire, features) run on the CPU in `src/model/model_temporal.h`.
- **Hardware all the way to the model.** The camera frame goes into a CMM
  (physically contiguous) frame ring by IVPS copy; the GDC warps it straight
  from there into both model views, and IVPS scales it onto the LCD. No process
  touches camera pixels on the CPU.
- **openpilot's lateral stack in C++.** Lane planner, lateral MPC, torque
  controller, and online camera calibration, with no openpilot checkout, Python
  native extension, or Qt on the board. The in-tree MPC solver replaces acados.
  Ports of paramsd and torqued estimate the steer ratio and torque response
  while driving, and the controller uses them.
- **K7 YG HEV integration.** `LKAS11` and `MDPS12` at 100 Hz, `CLU11` at 50 Hz,
  the 60 kph MDPS speed helper, and a torque ramp that cuts the request before
  the MDPS fault angle.
- **Vision cruise.** The vision lead nudges the stock fixed-speed cruise setpoint
  with `SET-`/`RES+` pulses. There is no longitudinal actuation.
- **Driving HUD.** Plan, lanes, road edges, and lead over the camera preview on
  the 640x480 LCD, drawn at 20 Hz on a hardware overlay layer, with status
  panels and stop-and-go departure alerts (on screen and on the board speaker).
- **Replay and tune.** Host tools replay a recorded drive open- or closed-loop,
  and a web parameter editor pushes changes the controller picks up within
  100 ms.
- **Tested off the board.** The control and perception libraries build on
  macOS or Linux, with a googletest suite (320 tests) that needs neither the
  board nor its SDK.

## How it works

### The pipeline

```mermaid
---
config:
  flowchart:
    wrappingWidth: 400
---
flowchart TB
    cam(["camera<br>OS04D10"]) --> camerad["camerad<br>ISP, 20 Hz"]
    camerad --> ring[("frame ring<br>8 CMM slots")]
    ring -- GDC --> modeld["modeld<br>supercombo on the NPU"]
    ring -- IVPS --> overlayd["overlayd<br>LCD video and HUD"]
    ring -- "IVPS, VENC" --> recordd["recordd<br>H.264, event log"]
    modeld --> controlsd["controlsd, 100 Hz<br>planner, MPC, torque"]
    modeld -- "frame it used" --> recordd
    imu(["board IMU"]) --> imud["imud"]
    imud --> locationd["locationd<br>pose, steering lag"]
    locationd --> controlsd
    controlsd <-- "CAN, sendcan" --> pandad["pandad"]
    pandad <-- USB --> panda(["Panda"])
    panda <-- CAN --> car(["K7 HEV"])
    classDef pixel fill:#e7f5f4,stroke:#1f9e9a,color:#1f2328
    classDef compute fill:#ebf3fe,stroke:#2f81f7,color:#1f2328
    classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
    classDef external fill:#fff3e8,stroke:#f0883e,color:#1f2328
    class cam,camerad,ring,overlayd pixel
    class modeld,controlsd,locationd compute
    class recordd,imud,pandad,imu proc
    class panda,car external
    linkStyle 0,1,2,3,4 stroke:#1f9e9a,stroke-width:2px
    linkStyle 11,12 stroke:#f0883e,stroke-width:2px
```

Teal lines carry camera pixels through hardware buffers, black lines carry
states through `/dev/shm`, and orange lines are the USB and CAN links.

Each process does one job and talks to the others through `/dev/shm`, keeping
openpilot's process boundaries without Cap'n Proto/cereal. Camera pixels never
pass through the CPU: the frame ring lives in physically contiguous memory, the
GDC warps it straight into both model views, and IVPS scales it onto the LCD and
copies it for the recorder. `manager.py` starts and supervises the processes,
the web console (`web_console/`) serves the tuning UI, and `recordd` logs every state next to
the H.264 video. See [Runtime](docs/runtime.md) for each process.
`scripts/install_autostart.sh` installs a systemd unit that starts the runtime at
boot in place of the stock launcher.

### One frame, end to end

Measured on the board. Frames arrive every 50 ms (20 Hz); `controlsd` runs
every 10 ms and uses the newest model output:

```mermaid
---
config:
  gantt:
    useWidth: 820
    fontSize: 13
    sectionFontSize: 13
    barHeight: 24
---
gantt
    dateFormat x
    axisFormat %-L ms
    tickInterval 10millisecond
    todayMarker off
    section One frame
    capture + ISP into the frame ring, 43.1 ms :0, 43ms
    GDC warp, 1 ms :43, 1ms
    NPU (supercombo), 15.5 ms :44, 16ms
    inputs + parse + publish, 3 ms :60, 3ms
    model output, about 63 ms :milestone, 63, 0ms
    wait for the next controlsd tick, up to 10 ms :63, 10ms
    steering command, within 73 ms :milestone, 73, 0ms
```

### Lateral control

`controlsd` runs at 100 Hz; the learners refine the car model while driving:

```mermaid
---
config:
  flowchart:
    wrappingWidth: 400
---
flowchart TB
    model["supercombo<br>plan, lanes, lead"] --> lane["lane mode<br>lane-centre path + MPC"]
    model --> laneless["laneless mode<br>curvature from the plan"]
    lane --> desired["desired curvature<br>jerk-limited, lag-compensated"]
    laneless --> desired
    desired --> torque["torque controller<br>openpilot torque model"]
    learners["learners<br>paramsd, torqued, lagd"] -.-> torque
    torque --> limits["Hyundai limits<br>rate, override, fault angle"]
    limits -- "LKAS11, MDPS12, CLU11" --> safety["Panda safety<br>hyundaiCommunity"]
    safety --> mdps(["MDPS"])
    classDef compute fill:#ebf3fe,stroke:#2f81f7,color:#1f2328
    classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
    classDef external fill:#fff3e8,stroke:#f0883e,color:#1f2328
    classDef safety fill:#eaf7ee,stroke:#2da44e,color:#1f2328
    class model,lane,laneless,desired,torque compute
    class learners proc
    class limits,safety safety
    class mdps external
```

On a recorded minute of engaged lane keeping at 41-66 km/h (2026-10-05), the
car's measured curvature (orange, from the steering angle and yaw rate) follows
what the controller asks for (blue) with a correlation of 0.98; the driver
steered for 1.5% of the time:

```mermaid
---
config:
  xyChart:
    height: 300
  themeVariables:
    xyChart:
      plotColorPalette: "#2f81f7, #f0883e"
---
xychart-beta
    x-axis "time (s)" 0 --> 60
    y-axis "curvature (1/km)" -5 --> 25
    line [-0.4, -0.3, -0.6, -0.8, -0.8, -1.2, -1.5, -1.5, -1.3, -1.6, -1.9, -2.4, -2.3, -2.2, -2.2, -2.5, -2.6, -2.9, -2.8, -2.7, -2.6, -2.5, -2.2, -2.0, -2.7, -2.6, -2.9, -2.9, -3.1, -3.2, -2.9, -2.5, -1.7, -1.8, -1.9, -2.2, -3.0, -3.0, -3.1, -2.3, -2.0, -2.2, -2.6, -2.8, -2.8, -2.8, -3.0, -2.8, -2.4, -2.2, -2.0, -2.4, -2.2, -2.2, -2.6, -2.7, -2.5, -2.3, -2.6, -2.0, -2.4, -2.4, -2.4, -1.4, -0.7, -0.2, -0.0, -0.6, -0.4, -0.6, -1.1, -1.6, -1.5, -2.0, -2.2, -1.9, -1.8, -1.3, -0.7, -0.1, 0.4, 1.0, 2.5, 4.5, 7.6, 11.5, 13.1, 16.1, 15.3, 15.5, 15.8, 15.4, 15.7, 16.1, 15.2, 12.0, 13.8, 17.9, 18.3, 20.4, 18.0, 13.2, 8.1, 5.5, 4.2, 3.1, 2.0, 0.9, 0.1, 0.3, 0.3, 0.4, 0.2, 0.3, 0.1, -0.4, -0.8, -1.6, -2.3, -2.7, -2.2]
    line [-0.3, -0.5, -0.6, -0.6, -0.7, -0.9, -1.3, -1.5, -1.4, -1.5, -1.6, -2.1, -2.3, -2.3, -2.1, -2.3, -2.5, -2.5, -2.6, -2.7, -2.7, -2.7, -2.4, -2.2, -2.2, -2.2, -2.4, -2.7, -2.7, -3.0, -3.3, -2.6, -2.4, -1.6, -2.0, -2.0, -2.4, -2.9, -3.0, -2.9, -2.4, -2.0, -2.3, -2.6, -2.7, -2.6, -2.6, -2.9, -2.6, -2.2, -1.9, -2.1, -2.4, -2.4, -2.3, -2.5, -2.6, -2.5, -2.2, -2.2, -2.0, -2.3, -2.2, -1.9, -1.2, -0.2, 0.2, -0.3, -0.3, -0.3, -0.7, -1.1, -1.2, -1.4, -1.9, -2.0, -1.9, -1.7, -1.2, -0.7, 0.2, 0.6, 1.1, 3.4, 4.5, 8.9, 10.7, 13.7, 14.7, 15.1, 15.1, 15.0, 15.1, 15.1, 15.0, 13.8, 12.4, 12.4, 20.3, 22.3, 19.9, 19.4, 13.4, 5.8, 5.4, 4.3, 3.1, 2.0, 0.6, 0.2, 0.2, 0.4, 0.3, 0.2, 0.5, 0.2, -0.4, -0.9, -1.6, -2.3, -2.4]
```

### What it costs the board

CPU per process with the full pipeline at 20 Hz on the board, over 60 s
windows (2026-10-07): 41% of one core together, about a fifth of the two-core
AX630C. `overlayd` draws the HUD, `camerad` is mostly the ISP's 3A thread, and
the NPU runs the model for `modeld`:

```mermaid
---
config:
  xyChart:
    height: 340
    chartOrientation: horizontal
  themeVariables:
    xyChart:
      plotColorPalette: "#1f9e9a"
---
xychart-beta
    x-axis ["overlayd", "camerad", "modeld", "locationd", "controlsd", "imud", "web_console", "recordd", "pandad", "manager.py"]
    y-axis "CPU, % of one core" 0 --> 16
    bar [14.9, 11, 8.2, 2, 1.6, 1.3, 0.7, 0.5, 0.5, 0.3]
```

## The HUD

`overlayd` draws the HUD at 20 Hz on the display's hardware overlay layer, above
the camera preview. These states are rendered off-line with
[`hud_snapshot`](diagnostics/README.md), the same renderer the board runs:

<table>
  <tr>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/drive.jpg" width="240" alt="Engaged"><br><sub><b>Engaged</b>: path, lead distance and closing speed, lane position</sub></td>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/lane-change.jpg" width="240" alt="Lane change"><br><sub><b>Lane change</b>: blinker on, steer to start once safe</sub></td>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/depart.jpg" width="240" alt="Green light"><br><sub><b>Green light</b>: departure alert while held at a stop</sub></td>
  </tr>
  <tr>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/saturated.jpg" width="240" alt="Take control"><br><sub><b>Take control</b>: the turn needs more torque than allowed</sub></td>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/paused.jpg" width="240" alt="Steering paused"><br><sub><b>Steering paused</b>: the driver turned the wheel past 85°; steering resumes below 15°</sub></td>
    <td width="33%" align="center" valign="top"><img src="docs/images/hud/fault.jpg" width="240" alt="Steering fault"><br><sub><b>Steering fault</b>: the MDPS reports a fault</sub></td>
  </tr>
</table>

Alerts also play on the board speaker. Tapping the status pill opens a network
card, and tapping the left column toggles a diagnostic card.

## Safety model

Every layer must agree before steering torque reaches the car:

```mermaid
---
config:
  flowchart:
    wrappingWidth: 400
---
flowchart TB
    gates["1 · controlsd gates<br>model, MPC, CAN, gear,<br>seatbelt, driver SET"] --> limits["2 · controller limits<br>curvature, jerk, torque rate,<br>fault-angle ramp"]
    limits --> tx["3 · EDGEPILOT_PANDA_TX<br>final transmit switch,<br>0 = shadow mode"]
    tx --> panda["4 · Panda safety<br>hyundaiCommunity,<br>never bypassed"]
    panda --> mdps(["MDPS"])
    classDef safety fill:#eaf7ee,stroke:#2da44e,color:#1f2328
    classDef external fill:#fff3e8,stroke:#f0883e,color:#1f2328
    class gates,limits,tx,panda safety
    class mdps external
```

1. **`controlsd` gates** require a fresh model, a valid MPC solution, fresh
   vehicle state, the right gear, a fastened seatbelt, and an explicit driver
   SET press. A failing gate shows its reason on the HUD.
2. **Controller limits** cap the curvature at openpilot's `0.2 1/m` with a jerk
   limit, rate-limit the torque, and ramp it to zero before the MDPS fault angle.
3. **`EDGEPILOT_PANDA_TX`** is the final transmit switch; the controller never
   transmits on its own. On the MaixCAM2 the manager starts `pandad` only
   with `EDGEPILOT_ENABLE_PANDA=1`.
4. **Panda safety firmware** (`hyundaiCommunity`) enforces the Hyundai torque,
   rate, and driver-override limits and is never bypassed.

## Hardware

The Panda joins the board to three of the car's CAN buses; TX lists the frames
edgepilot sends on each:

```mermaid
---
config:
  flowchart:
    wrappingWidth: 400
---
flowchart TB
    subgraph board["Sipeed MaixCAM2"]
        soc["AX630C<br>2x A53, NPU, 1 GB"]
        io["camera, 640x480 LCD,<br>speaker"]
    end
    board -- "USB-C, host mode" --> panda(["comma Panda"])
    panda -- "bus 0<br>TX LKAS11, CLU11 cruise buttons" --> pt["powertrain<br>cluster, brake, gear"]
    panda -- "bus 1<br>TX LKAS11, CLU11" --> mdps["MDPS<br>and SAS"]
    panda -- "bus 2<br>TX MDPS12" --> cam["stock LKAS<br>camera"]
    classDef compute fill:#ebf3fe,stroke:#2f81f7,color:#1f2328
    classDef proc fill:#f6f8fa,stroke:#8b949e,color:#1f2328
    classDef external fill:#fff3e8,stroke:#f0883e,color:#1f2328
    classDef group fill:#ffffff,stroke:#d1d9e0,color:#59636e
    class board group
    class soc compute
    class io proc
    class panda,pt,mdps,cam external
```

- Sipeed MaixCAM2 (AX630C: 2x Cortex-A53 + NPU, 1 GB), with its stock
  `ov_os04d10` camera and 640x480 LCD, on the stock image (Ubuntu 22.04 arm64,
  AX runtime in `/opt/lib`, `libmaixcam_lib` 1.2.5)
- a comma Panda on the USB-C port (the manager switches it to host mode)
- a KIA K7 YG HEV

The MaixCAM2 has a single USB-C port, so a Panda setup needs power from another
source.

## Getting started

1. **Prepare the board.** Flash the stock image to the microSD card, put the
   Wi-Fi settings on its boot partition before the first boot, and install
   your SSH key: see [Board setup](docs/board-setup.md).
2. **Build.** Fetch the SDK and the board libraries once, then build in an
   arm64 Ubuntu 22.04 container
   ([details](docs/build-and-deploy.md)). The scripts that reach the board take
   its address from `EDGEPILOT_BOARD` (or their first argument):

   ```sh
   export EDGEPILOT_BOARD=root@<board-ip>
   scripts/fetch_maixcam2_sdk.sh
   tools/docker_ax630/build.sh          # -> build-ax630/bin
   ```

3. **Upload.** The model (`models/supercombo.axmodel`, built with
   [tools/model/axmodel](tools/model/axmodel/README.md)) is in the repository and
   is sent only when it changed:

   ```sh
   scripts/upload_to_board.sh
   scripts/install_autostart.sh   # also sets maix_npu_ai_isp=1
   ```

4. **Run.** On the board:

   ```sh
   python3 /root/edgepilot/manager.py
   ```

   The manager stops the stock launcher first. Tune parameters at
   `http://<board-ip>:8080`.
5. **Shadow run first.** Verify Panda RX, safety mode, and counters with TX off
   before enabling it; the gates are listed in
   [K7 Panda port](docs/k7-panda-port.md).

## Development

```sh
./scripts/run_host_tests.sh    # build and run every host unit test through ctest
```

- [Host tests](tests/README.md): what each test covers and how to add one
- [Diagnostic tools](diagnostics/README.md): replay, dataset, and HUD tools
- [Rehearsal](docs/rehearsal.md): replay a recorded drive through the whole runtime
  on the board (hardware H.264 decode into the frame ring, CAN on the same timeline)
- [Closed-loop replay](docs/closed-loop-replay.md): rank control changes against
  a recorded drive before driving them
- [Scripts](scripts/README.md): build, deploy, and the board-side Python
- [Style guide](docs/style-guide.md): how code, comments, documents and diagrams
  are written

## Repository layout

```text
src/            runtime code, one folder per subsystem (docs/source-layout.md)
platform/       MaixCAM2 camera (VI), display (VO), CMM, and GDC wrappers
params/         runtime parameters, hot-reloaded by the processes
models/         the axmodel and its manifest
firmware/       the Panda firmware (firmware/panda)
tests/          host unit tests and the Python layout checks
diagnostics/    replay, dataset, and HUD tools
scripts/        SDK fetch, deploy, host tests, board-side Python
tools/          axmodel pipeline, camera calibration, build container, route readers
docs/           documentation (index in docs/README.md); docs/images holds the drive and HUD pictures
```

## Documentation

The [documentation index](docs/README.md) maps each part of the system to the
document that covers it and gives a reading order.

- **Get started:** [Board setup](docs/board-setup.md) · [Build and deploy](docs/build-and-deploy.md) ·
  [Boot time](docs/boot-time.md) · [Rehearsal](docs/rehearsal.md)
- **How it works:** [Runtime](docs/runtime.md) · [Model pipeline](docs/model-pipeline.md) ·
  [HUD](docs/hud.md) · [Recording](docs/recording.md) · [Source layout](docs/source-layout.md)
- **Operate and tune:** [Runtime options](docs/runtime-options.md) · [Parameters](params/README.md) ·
  [Diagnostics](docs/diagnostics.md) · [Verification](docs/verification.md)
- **Design notes:** [K7 Panda port](docs/k7-panda-port.md) · [Departure alerts](docs/departure-alerts.md) ·
  [Closed-loop replay](docs/closed-loop-replay.md) · [Flow calibration](docs/flow-calibration.md) ·
  [Lead flow](docs/lead-flow.md) · [Lane-line filter](docs/lane-line-filter.md) ·
  [Lateral NN](docs/lateral-nn.md)
- **Code and tools:** [Style guide](docs/style-guide.md) · [Host tests](tests/README.md) ·
  [Diagnostic tools](diagnostics/README.md) · [Scripts](scripts/README.md) ·
  [Model tools](tools/model/README.md) · [axmodel pipeline](tools/model/axmodel/README.md) ·
  [Model package](models/README.md) · [Panda firmware](firmware/panda/README.md)

## Acknowledgements

- [openpilot](https://github.com/commaai/openpilot) by comma.ai: the supercombo
  model, and the planner, MPC, calibration, and parameter estimators this
  runtime ports
- [panda](https://github.com/commaai/panda) by comma.ai: the CAN interface, and
  the firmware in `firmware/panda` (MIT), by way of crwusiz's openpilot import
- [MaixCDK](https://github.com/sipeed/MaixCDK) by Sipeed: the MaixCAM2 MSP SDK
  and `libmaixcam_lib` headers
- Pulsar2 by Axera: the AX630C NPU compiler
