# STM32F413 보드(black panda, uno, dos)용 Panda 펌웨어와 bootstub.
#   make          obj/panda.bin.signed(앱, debug 키 서명)와 obj/bootstub.panda.bin
#   make clean
# arm-none-eabi-gcc와 python3만 있으면 된다. newlib은 쓰지 않고(-ffreestanding)
# 서명도 표준 라이브러리로 한다. debug 키로 서명하므로 debug 서명을 받는 bootstub이
# 올라간 Panda에서만 돈다. 원래의 board/SConscript와 같은 명령을 같은 순서로 낸다.

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
PYTHON ?= python3
OBJ := obj
VERSION := DEV-23456789-DEBUG

FLAGS := -Wall -Wextra -Wstrict-prototypes -Werror -mlittle-endian -mthumb -nostdlib -fno-builtin \
         -T$(CURDIR)/board/stm32fx/stm32fx_flash.ld -std=gnu11 -ffreestanding \
         -mcpu=cortex-m4 -mhard-float -DSTM32F4 -DSTM32F413xx -mfpu=fpv4-sp-d16 \
         -fsingle-precision-constant -Os -g -DPANDA -DALLOW_DEBUG $(EXTRA_CFLAGS)
INCLUDES := -Iboard/stm32fx/inc -I. -Iboard
APP_START := 0x8004000

all: $(OBJ)/panda.bin.signed $(OBJ)/bootstub.panda.bin

$(OBJ):
	mkdir -p $@

# 소스가 #include "obj/..."로 읽는 생성 헤더
$(OBJ)/gitversion.h: Makefile | $(OBJ)
	printf 'const uint8_t gitversion[] = "%s";\n' '$(VERSION)' > $@
	printf '%s' '$(VERSION)' > $(OBJ)/version

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
