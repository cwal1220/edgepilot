# AX630C(MaixCAM2) 런타임 빌드 환경. 보드 이미지와 같은 Ubuntu 22.04 arm64라
# 여기서 만든 바이너리가 보드의 glibc/libstdc++에서 그대로 돈다. Apple Silicon의
# colima(aarch64)에서는 에뮬레이션 없이 네이티브로 빌드된다. gcc-arm-none-eabi와 python3는 Panda
# 펌웨어(firmware/panda)용이다. 보드의 apt에서 받는 것과 같은 jammy 10.3이라, 같은 소스면 여기서든
# 보드에서든 같은 바이너리가 나온다.
FROM --platform=linux/arm64 ubuntu:22.04
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config libusb-1.0-0-dev ca-certificates git \
      gcc-arm-none-eabi python3 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
