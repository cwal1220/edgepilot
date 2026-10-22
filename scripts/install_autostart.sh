#!/usr/bin/env bash
# MaixCAM2에 부팅 자동 시작을 설치한다: edgepilot.service를 켜고 기본 UI 런처는 부팅 때
# 뜨지 않게 하며, Wi-Fi 재연결 때 DHCP를 새로 받는 wifi-dhcp-renew.service도 켠다.
# 되돌리기: scripts/install_autostart.sh --remove
# 사용: scripts/install_autostart.sh [--remove] root@<보드 IP>   (또는 EDGEPILOT_BOARD=root@<보드 IP>,
#   설치 디렉터리 EDGEPILOT_BOARD_DIR=/root/edgepilot)
set -euo pipefail
cd "$(dirname "$0")/.."
REMOVE=0
if [ "${1:-}" = "--remove" ]; then REMOVE=1; shift; fi
BOARD="${1:-${EDGEPILOT_BOARD:-}}"
if [ -z "$BOARD" ]; then
  echo "usage: $0 [--remove] root@<board-ip>   (or set EDGEPILOT_BOARD=root@<board-ip>)" >&2
  exit 2
fi
DEST="${EDGEPILOT_BOARD_DIR:-/root/edgepilot}"
SSH=(ssh -o StrictHostKeyChecking=no "$BOARD")

if [ "$REMOVE" = 1 ]; then
  "${SSH[@]}" "systemctl disable --now edgepilot.service; rm -f /etc/systemd/system/edgepilot.service;
    systemctl daemon-reload; systemctl enable --now launcher.service"
  echo "autostart removed; stock launcher restored on $BOARD"
  exit 0
fi

# AI-ISP가 NPU 한 코어를 쓰도록 부팅 때 NPU를 분할한다(camerad·modeld가 이를 전제한다).
if "${SSH[@]}" "grep -qx 'maix_npu_ai_isp=1' /boot/configs"; then :; else
  # 순정 이미지의 /boot/configs는 마지막 줄에 줄바꿈이 없어서, 그냥 덧붙이면 앞 줄에 이어 붙는다.
  "${SSH[@]}" "sed -i '/^maix_npu_ai_isp=/d' /boot/configs && { [ -z \"\$(tail -c1 /boot/configs)\" ] || echo >> /boot/configs; } &&
    echo maix_npu_ai_isp=1 >> /boot/configs && sync"
  echo "set maix_npu_ai_isp=1 in /boot/configs; reboot the board before starting the runtime"
fi

# Wi-Fi가 다른 AP로 넘어갈 때 DHCP를 새로 받는 훅(보드 udhcpc는 이전 임대를 들고 있다).
"${SSH[@]}" "cat > /usr/local/sbin/wifi-dhcp-renew.sh && chmod 755 /usr/local/sbin/wifi-dhcp-renew.sh" \
  < scripts/wifi-dhcp-renew.sh
"${SSH[@]}" "cat > /etc/systemd/system/wifi-dhcp-renew.service && systemctl daemon-reload &&
  systemctl enable --now wifi-dhcp-renew.service" < scripts/wifi-dhcp-renew.service

sed "s|@DEST@|$DEST|g" scripts/edgepilot.service |
  "${SSH[@]}" "test -x '$DEST/manager.py' || { echo 'run scripts/upload_to_board.sh first' >&2; exit 1; }
    cat > /etc/systemd/system/edgepilot.service && systemctl daemon-reload &&
    systemctl disable launcher.service && systemctl enable edgepilot.service && sync"
echo "autostart installed on $BOARD (starts at next boot, or: ssh $BOARD systemctl start edgepilot)"
