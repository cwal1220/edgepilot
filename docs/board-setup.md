# Board setup

[← Documentation index](../README.md)

The runtime runs on the stock Sipeed MaixCAM2 image. Nothing is reflashed: the
image already carries what the binaries link against, and the build copies those
exact files from the board (see [Build and deploy](build-and-deploy.md)).

| Item | Value |
| --- | --- |
| SoC | AX630C, 2x Cortex-A53 + NPU, 1 GB |
| Camera | `ov_os04d10`, opened at 1280x720 NV12 |
| LCD | 640x480 landscape; the panel is 480x640 and VO rotates it |
| OS | Ubuntu 22.04 arm64 |
| AX runtime | `/opt/lib` (`libax_engine`, `libax_sys`, `libax_ivps`, ...) |
| Camera/display library | `/usr/lib/libmaixcam_lib.so.1.2.5` |
| Install directory | `/root/edgepilot` |

## SSH

The scripts log in as `root` with key authentication; the default board address
is `192.168.219.117`. Install your public key once:

```sh
ssh-copy-id root@192.168.219.117
```

`fetch_maixcam2_sdk.sh` and `upload_to_board.sh` take the board as their first
argument when the address differs.

## Web console packages

The web console needs FastAPI and uvicorn:

```sh
python3 -m pip install -r /root/edgepilot/requirements-web-console.txt
```

`pandad` links `libusb-1.0`; install `libusb-1.0-0` on the board if it is
missing.

## Things to avoid on this board

- **Do not build on the board.** A native build runs out of the 1 GB memory.
  Build in the container described in [Build and deploy](build-and-deploy.md).
- **Watch `libmaixcam_lib.so` after apt or ldconfig.** The board's apt/ldconfig
  can repoint `/usr/lib/libmaixcam_lib.so` to a `_bak` file, and then camerad
  and overlayd fail to start. Restore the link:

  ```sh
  ln -sfn libmaixcam_lib.so.1.2.5 /usr/lib/libmaixcam_lib.so
  ```

- **Do not walk the whole filesystem.** The stock rootfs has ext4
  directory-checksum defects under `/usr/share/doc`; a whole-filesystem walk such
  as `find /` hits them and the kernel remounts `/` read-only. Search specific
  directories instead. The defects are fixed with an offline `e2fsck -fD` on the
  root partition (SD card in another machine, filesystem unmounted).
- **A running binary cannot be overwritten.** `upload_to_board.sh` handles this
  by uploading into `.upload/` and moving the files into place; copy by hand the
  same way.

## Stock launcher

The stock UI (`launcher.service`) and the apps it starts from `/maixapp/apps/`
use about 30% CPU and hold the camera and NPU. `manager.py` stops both
before starting the runtime unless `EDGEPILOT_STOP_LAUNCHER=0`. It does not restart
them on exit; `systemctl start launcher.service` brings the stock UI back.
