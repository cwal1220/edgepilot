"""Panda 펌웨어 카드: pandad가 1초마다 쓰는 연결·버전·플래싱 진행(src/panda/pandad.cc)과 설치된 이미지를 보여
주고, 주차 중일 때만 플래싱을 요청한다. 요청 파일에는 쓸 이미지의 버전이 들어 있고, pandad는 그 버전이 설치된
이미지와 같고 차 상태가 맞을 때만 쓴다(같은 조건을 다시 본다). 이미지 규칙과 주차 조건은
src/panda/panda_firmware.cc와 같다."""
from __future__ import annotations

import json
import os
import re
import struct
from pathlib import Path
from typing import Any, Dict

from .shm_channel import IpcReader, age_s, topic_path
from .state_layout import CONTROL_STATE_SIZE, CONTROL_STATE_TOPIC, GEAR_PARK, control_value

PANDA_STATUS_PATH = "/dev/shm/edgepilot_panda_status.json"
PANDA_FLASH_REQUEST_PATH = "/dev/shm/edgepilot_panda_flash"
# pandad와 같이 EDGEPILOT_PANDA_FIRMWARE, 없으면 설치 디렉터리의 firmware/panda.bin.signed
INSTALL_DIR = Path(__file__).resolve().parents[1]
PANDA_FIRMWARE_PATH = str(INSTALL_DIR / (os.environ.get("EDGEPILOT_PANDA_FIRMWARE") or "firmware/panda.bin.signed"))
PANDA_STATUS_STALE_S = 3.0
# panda_protocol.h와 같은 값: 서명 128바이트, 앱 영역(섹터 1~3) 48 KB, F413 보드(black, uno, dos)
PANDA_SIGNATURE_BYTES = 128
PANDA_APP_MAX_BYTES = 3 * 16 * 1024
PANDA_FLASHABLE_HW = (3, 5, 6)
PANDA_VERSION_PATTERN = re.compile(rb"(?<![0-9A-Za-z])[A-Z]{3,8}-[0-9A-Za-z]{8}-(?:DEBUG|RELEASE)(?![0-9A-Za-z])")
CONTROL_MAX_AGE_S = 1.0
PARKED_MAX_SPEED_KPH = 1.0
# pandad·panda_flasher의 이유 코드
REASON_TEXT = {
    "control_stale": "제어 상태가 없습니다 (controlsd 확인)",
    "vehicle_stale": "차 상태를 받지 못합니다 (시동·CAN 확인)",
    "engaged": "조향이 켜져 있습니다 (해제 후)",
    "not_park": "P단이 아닙니다",
    "moving": "차가 움직이고 있습니다",
    "image_invalid": "설치된 펌웨어 이미지가 없거나 잘못됐습니다",
    "image_changed": "설치된 이미지가 바뀌었습니다 (새로고침 후 다시)",
    "no_panda": "판다가 USB에 없습니다",
    "hw_unsupported": "지원하지 않는 판다입니다 (F413 보드만)",
    "busy": "다른 프로세스가 판다를 쓰고 있습니다",
    "usb_error": "USB 오류",
    "bootstub_timeout": "판다가 bootstub으로 넘어가지 않았습니다",
    "flasher_missing": "bootstub 플래셔가 응답하지 않습니다",
    "erase_failed": "플래시를 지우지 못했습니다",
    "write_failed": "쓰다가 실패했습니다",
    "verify_failed": "쓴 끝 주소가 맞지 않습니다",
    "app_timeout": "새 펌웨어가 시작되지 않았습니다 (bootstub에 머묾, 다시 쓰세요)",
    "version_mismatch": "판다가 이미지와 다른 버전을 알립니다",
}
STEP_TEXT = {"bootstub": "bootstub 진입", "erase": "지우는 중", "write": "쓰는 중", "verify": "확인",
             "reboot": "재시작", "check": "버전 확인"}


def panda_image_info(path: str) -> Dict[str, Any]:
    """설치된 앱 이미지(firmware/panda의 panda.bin.signed)를 panda_firmware.cc와 같은 규칙으로 본다."""
    info: Dict[str, Any] = {"path": path, "present": False, "valid": False, "version": "", "size": 0, "error": ""}
    try:
        data = Path(path).read_bytes()
    except OSError:
        info["error"] = "없음"
        return info
    size = len(data)
    body = size - PANDA_SIGNATURE_BYTES
    if size <= PANDA_SIGNATURE_BYTES + 16:
        error = "너무 작음"
    elif size > PANDA_APP_MAX_BYTES:
        error = f"앱 영역({PANDA_APP_MAX_BYTES} B)보다 큼"
    elif size % 4:
        error = "4바이트 단위가 아님"
    elif struct.unpack_from("<I", data)[0] != body:
        error = "서명된 이미지가 아님 (길이 필드)"
    elif data[body - 8:body - 4] != b"VERS":
        error = "VERS 꼬리 없음"
    else:
        error = ""
    match = PANDA_VERSION_PATTERN.search(data)
    if not error and not match:
        error = "버전 문자열 없음"
    info.update(present=True, size=size, valid=not error, error=error,
                version=match.group().decode() if match else "")
    return info


def flash_blocker(control: IpcReader) -> str:
    """panda_firmware.cc의 panda_flash_allowed와 같은 조건. 막는 이유 코드, 없으면 빈 문자열."""
    latest = control.read_payload()
    if latest is None or age_s(latest[1]) > CONTROL_MAX_AGE_S:
        return "control_stale"
    payload = latest[2]
    if not control_value(payload, "vehicle_fresh"):
        return "vehicle_stale"
    if control_value(payload, "engaged") or control_value(payload, "active"):
        return "engaged"
    if control_value(payload, "gear", "<i") != GEAR_PARK:
        return "not_park"
    speed = max(control_value(payload, "ego_speed_kph", "<f"), control_value(payload, "cluster_speed_kph", "<f"))
    if speed >= PARKED_MAX_SPEED_KPH:
        return "moving"
    return ""


class PandaUpdate:
    def __init__(self, status_path: str = PANDA_STATUS_PATH, request_path: str = PANDA_FLASH_REQUEST_PATH,
                 image_path: str = PANDA_FIRMWARE_PATH, control_path: str = topic_path(CONTROL_STATE_TOPIC)):
        self.status_path = Path(status_path)
        self.request_path = Path(request_path)
        self.image_path = image_path
        self.control = IpcReader(control_path, CONTROL_STATE_SIZE)

    def _pandad(self) -> tuple[Dict[str, Any] | None, float]:
        """pandad의 상태 파일과 그 나이. 없거나 오래됐으면 (None, 나이)."""
        try:
            document = json.loads(self.status_path.read_text(encoding="utf-8"))
            age = age_s(int(document["stamp_ns"]))
        except (OSError, ValueError, KeyError, TypeError):
            return None, 0.0
        return (document if age <= PANDA_STATUS_STALE_S else None), age

    def status(self) -> Dict[str, Any]:
        pandad, age = self._pandad()
        link = pandad or {}
        flash = link.get("flash") or {}
        image = panda_image_info(self.image_path)
        blocker = flash_blocker(self.control)
        version = link.get("firmware_version", "")
        return {
            "available": pandad is not None,
            "age_s": age,
            "mode": link.get("mode", "none"),
            "serial": link.get("serial", ""),
            "hw_type": link.get("hw_type", 0),
            "hw_name": link.get("hw_name", ""),
            "firmware_version": version,
            "flash": flash,
            "flash_error_text": REASON_TEXT.get(flash.get("error", ""), flash.get("error", "")),
            "flash_step_text": STEP_TEXT.get(flash.get("step", ""), ""),
            "image": image,
            "blocker": blocker,
            "blocker_text": REASON_TEXT.get(blocker, blocker),
            "request_pending": self.request_path.exists(),
            "up_to_date": bool(image["version"]) and image["version"] == version,
        }

    def request(self, version: str) -> Dict[str, Any]:
        status = self.status()
        image = status["image"]
        if not status["available"]:
            raise PermissionError("pandad가 실행 중이 아닙니다. 런타임을 멈췄다면 보드에서 panda_flash를 쓰세요.")
        if status["request_pending"] or status["flash"].get("state") == "flashing":
            raise PermissionError("이미 플래싱 중입니다.")
        if not image["valid"]:
            raise PermissionError(f"설치된 이미지를 쓸 수 없습니다: {image['error']}")
        if version != image["version"]:
            raise PermissionError(REASON_TEXT["image_changed"])
        if status["mode"] == "none":
            raise PermissionError(REASON_TEXT["no_panda"])
        if status["mode"] == "app" and status["hw_type"] not in PANDA_FLASHABLE_HW:
            raise PermissionError(REASON_TEXT["hw_unsupported"])
        if status["blocker"]:
            raise PermissionError(f"지금은 쓸 수 없습니다: {status['blocker_text']}")
        temporary = self.request_path.with_name(self.request_path.name + ".tmp")
        temporary.write_text(version + "\n", encoding="utf-8")
        os.replace(temporary, self.request_path)
        return self.status()
