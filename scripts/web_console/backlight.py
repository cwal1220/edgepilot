"""MaixCAM2 LCD 백라이트 제어. 백라이트는 PWM3(pwmchip0/pwm3, 정극성) 하나이고, 밝기는
duty = 주기 x 밝기% x 보드 최대치%(/boot/board disp_max_backlight)다. overlayd도
시작할 때 같은 식으로 /boot/configs 값을 켜 두고, 웹 콘솔이 display.json을
적용할 때 이 모듈을 쓴다."""
from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Dict


PWM_CHIP = Path("/sys/class/pwm/pwmchip0")
PWM_CHANNEL = 3
PWM_PERIOD_NS = 100_000
BOARD_FILE = Path("/boot/board")
DEFAULT_MAX_PERCENT = 95


def board_max_percent(board_file: Path = BOARD_FILE) -> int:
    try:
        for line in board_file.read_text().splitlines():
            if line.startswith("disp_max_backlight="):
                return max(1, min(100, int(line.split("=", 1)[1])))
    except (OSError, ValueError):
        pass
    return DEFAULT_MAX_PERCENT


def duty_cycle_ns(brightness_percent: int, max_percent: int = DEFAULT_MAX_PERCENT) -> int:
    """밝기 0..100%를 PWM duty로. 보드 최대치를 넘지 않는다."""
    brightness = max(0, min(100, brightness_percent))
    return PWM_PERIOD_NS * brightness * max_percent // 10_000


class DisplayBacklight:
    def __init__(self, pwm_chip: Path = PWM_CHIP, board_file: Path = BOARD_FILE) -> None:
        self.pwm_chip = pwm_chip
        self.pwm_path = pwm_chip / f"pwm{PWM_CHANNEL}"
        self.max_percent = board_max_percent(board_file)
        self._enabled = True
        self._brightness_percent = 100
        self._last_error = ""
        self._lock = threading.Lock()

    def status(self) -> Dict[str, Any]:
        return {
            "available": self.pwm_chip.exists(),
            "enabled": self._enabled,
            "brightness_percent": self._brightness_percent,
            "mode": "pwm",
            "error": self._last_error,
        }

    def apply(self, config: Dict[str, Any]) -> None:
        enabled = config.get("enabled")
        brightness = config.get("brightness_percent")
        if not isinstance(enabled, bool):
            raise ValueError("display.enabled must be a boolean")
        if type(brightness) is not int or not 1 <= brightness <= 100:
            raise ValueError("display.brightness_percent must be an integer from 1 to 100")

        with self._lock:
            try:
                if not self.pwm_chip.exists():
                    raise RuntimeError(f"PWM chip is unavailable: {self.pwm_chip}")
                if not self.pwm_path.exists():
                    (self.pwm_chip / "export").write_text(str(PWM_CHANNEL))
                (self.pwm_path / "period").write_text(str(PWM_PERIOD_NS))
                duty = duty_cycle_ns(brightness, self.max_percent) if enabled else 0
                (self.pwm_path / "duty_cycle").write_text(str(duty))
                (self.pwm_path / "enable").write_text("1")
                self._enabled = enabled
                self._brightness_percent = brightness
                self._last_error = ""
            except (OSError, RuntimeError) as exc:
                self._last_error = str(exc)
                raise RuntimeError(f"display backlight update failed: {exc}") from exc
