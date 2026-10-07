# Build and deploy

[← Documentation index](../README.md)

All CMake executables are written to `bin/` below the build directory. Build
directories are generated output and are not tracked:

| Directory | Build |
| --- | --- |
| `build-ax630/` | MaixCAM2 runtime (`tools/docker_ax630/build.sh`), the upload default |
| `build-host/` | host tests and tools (`scripts/run_host_tests.sh`) |

The runtime build produces `camerad`, `modeld`, `overlayd`, `controlsd`,
`pandad`, `recordd`, `imud`, `locationd`, `replayd`, and `camcal`.
`-DEDGEPILOT_BUILD_PANDA=OFF` drops `pandad` and the libusb dependency; the
manager skips a binary that is not installed. `-DEDGEPILOT_BUILD_DIAGNOSTICS=ON` adds the tools in
[diagnostics/](../diagnostics/README.md) without changing the runtime.

## 1. Fetch the SDK and board libraries

```sh
scripts/fetch_maixcam2_sdk.sh [root@192.168.219.117]
```

This fills `deps/ax630/` (not tracked):

- the AX620E MSP SDK `v3.0.0_20250319114413`, the version the board runtime
  uses, from the MaixCDK release, checked by SHA256
- MaixCDK headers at commit `30f4b8b`: `ax_middleware.hpp` (the C++ interface
  of `libmaixcam_lib`), the `maix_basic`/`maix_image` headers it uses, and the
  OS04D10-aware MSP sample sources
- from the board over SSH, copied as-is so the ABI matches: `/opt/lib` (the AX
  runtime), `/usr/lib/libmaixcam_lib.so.1.2.5`, `libsamplerate`, and the OpenCV
  4.11 headers and `core`/`imgproc`/`imgcodecs` libraries (only `camcal` uses them)

The SDK download needs the network and the board copy needs SSH; after that the
build is offline. Run the script again after a board image update.

## 2. Build in the container

```sh
tools/docker_ax630/build.sh
```

The script checks `deps/ax630`, builds the `edgepilot-ax630-build` image
(`linux/arm64` Ubuntu 22.04, the same distribution as the board image, so the
binaries run against the board's glibc and libstdc++), and configures and builds
`build-ax630/` inside it with `AX_LIB_DIR=deps/ax630/lib` and an rpath of
`/opt/lib`. On an Apple Silicon Mac, an arm64 Docker VM such as colima builds
natively without emulation. The same run builds the [Panda
firmware](../firmware/panda/README.md) into `build-ax630/panda/obj/` with the
image's `gcc-arm-none-eabi`, the 10.3 the board's apt has, so a build on the
board gives the same bytes.

CMake stops with a clear message if `libax_engine`/`libax_sys`, the MaixCDK
headers, or the board OpenCV are missing.

> [!WARNING]
> Do not build on the board: a native build runs out of its 1 GB of memory.

## 3. Upload to the board

```sh
scripts/upload_to_board.sh [root@192.168.219.117]
```

The script copies the binaries from `build-ax630/bin`, the board-side Python, the
web console (`web_console/`, the server and its page, replaced as a whole), the
parameter defaults and `models/supercombo.axmodel`
([how it is built](../tools/model/axmodel/README.md)) to `/root/edgepilot`. The
33 MB model is sent only when its SHA-256 differs from the board's copy. The
Panda firmware image, when built, goes to `firmware/panda.bin.signed`; the
Panda itself is flashed from the web console or with `panda_flash`.

- A running binary cannot be overwritten, so files go to `.upload/` first and
  are moved into place.
- The manager is not restarted. Stop it before the upload (or restart it after)
  so the new binaries run.
- Runtime tuning and calibration JSON files already in `params/` are never
  overwritten. Repository defaults go to `params.defaults/` and seed a runtime
  file only when it does not exist.
- When the board's `/usr/lib/libmaixcam_lib.so` differs from
  `deps/ax630/maix/libmaixcam_lib.so` (the 1.2.5 copy the build links against),
  the script installs that copy as `/usr/lib/libmaixcam_lib.so.1.2.5` and points
  `libmaixcam_lib.so` at it. A stock file is kept as `libmaixcam_lib.so.stock`.
- It does not upload `replayd` ([rehearsal](rehearsal.md)). Copy it into the
  install directory by hand.
- `EDGEPILOT_BOARD_DIR` changes the install directory and `EDGEPILOT_BIN_DIR` the binary
  directory.

## Host tests

```sh
scripts/run_host_tests.sh
```

Builds `build-host/` with the runtime off and runs every test through `ctest`:
196 googletest cases, `check_web_console.py` and `check_recording_reader.py`. It needs neither the board nor
`deps/`; googletest is downloaded on the first configure.
