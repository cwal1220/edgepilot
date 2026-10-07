# STM32F413 보드(black panda, uno, dos)용 Panda 펌웨어와 bootstub.
#   make          obj/panda.bin.signed(앱, debug 키 서명)와 obj/bootstub.panda.bin
#   make clean
#   make OBJ=<디렉터리>/obj   산출물을 다른 곳에(빌드 컨테이너는 build-ax630/panda/obj)
# arm-none-eabi-gcc와 python3만 있으면 된다. newlib은 쓰지 않고(-ffreestanding)
# 서명도 표준 라이브러리로 한다. debug 키로 서명하므로 debug 서명을 받는 bootstub이
# 올라간 Panda에서만 돈다. 원래의 board/SConscript와 같은 명령을 같은 순서로 낸다.
# 버전 문자열은 panda_version.py가 소스 내용으로 정한다(EDGE-<해시>-DEBUG).

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
PYTHON ?= python3
OBJ ?= obj
VERSION := $(shell $(PYTHON) panda_version.py)
ifeq ($(VERSION),)
$(error panda_version.py failed)
endif

FLAGS := -Wall -Wextra -Wstrict-prototypes -Werror -mlittle-endian -mthumb -nostdlib -fno-builtin \
         -T$(CURDIR)/board/stm32fx/stm32fx_flash.ld -std=gnu11 -ffreestanding \
         -mcpu=cortex-m4 -mhard-float -DSTM32F4 -DSTM32F413xx -mfpu=fpv4-sp-d16 \
         -fsingle-precision-constant -Os -g -DPANDA -DALLOW_DEBUG $(EXTRA_CFLAGS)
# 소스는 생성 헤더를 "obj/..."로 읽으므로 OBJ의 부모를 맨 앞에 둔다(기본이면 .)
INCLUDES := -I$(OBJ)/.. -Iboard/stm32fx/inc -I. -Iboard
APP_START := 0x8004000

all: $(OBJ)/panda.bin.signed $(OBJ)/bootstub.panda.bin

$(OBJ):
	mkdir -p $@

# 소스가 #include "obj/..."로 읽는 생성 헤더. obj/version은 버전이 바뀔 때만 다시 써서
# 그에 기대는 것만 다시 빌드한다.
$(shell mkdir -p $(OBJ); [ "$$(cat $(OBJ)/version 2>/dev/null)" = '$(VERSION)' ] || printf '%s' '$(VERSION)' > $(OBJ)/version)

$(OBJ)/gitversion.h: $(OBJ)/version
	printf 'const uint8_t gitversion[] = "%s";\n' '$(VERSION)' > $@

$(OBJ)/cert.h: certs/debug.pub certs/release.pub crypto/cert_header.py crypto/rsakey.py | $(OBJ)
	$(PYTHON) crypto/cert_header.py $@ certs

$(OBJ)/startup.o: board/stm32fx/startup_stm32f413xx.s | $(OBJ)
	$(CC) $(FLAGS) -o $@ -c $<

$(OBJ)/%.o: crypto/%.c | $(OBJ)
	$(CC) -o $@ -c $(FLAGS) -MMD -MP $(INCLUDES) $<

$(OBJ)/main.o $(OBJ)/bootstub.o: $(OBJ)/gitversion.h
$(OBJ)/bootstub.o: $(OBJ)/cert.h

$(OBJ)/%.o: board/%.c | $(OBJ)
	$(CC) -o $@ -c $(FLAGS) -MMD -MP $(INCLUDES) $<

$(OBJ)/bootstub.panda.elf: $(OBJ)/startup.o $(OBJ)/rsa.o $(OBJ)/sha.o $(OBJ)/bootstub.o
	$(CC) -o $@ $(FLAGS) $^

$(OBJ)/panda.elf: $(OBJ)/startup.o $(OBJ)/main.o
	$(CC) -o $@ -Wl,--section-start,.isr_vector=$(APP_START) $(FLAGS) $^

$(OBJ)/%.bin: $(OBJ)/%.elf
	$(OBJCOPY) -O binary $< $@

$(OBJ)/panda.bin.signed: $(OBJ)/panda.bin certs/debug crypto/sign.py crypto/rsakey.py
	SETLEN=1 $(PYTHON) crypto/sign.py $< $@ certs/debug

clean:
	rm -rf $(OBJ)

.PHONY: all clean
.DELETE_ON_ERROR:
-include $(wildcard $(OBJ)/*.d)
