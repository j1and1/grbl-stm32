CC := arm-none-eabi-gcc
OBJS += \
	grbl/coolant_control.o \
	grbl/eeprom.o \
	grbl/gcode.o \
	grbl/jog.o \
	grbl/limits.o \
	grbl/main.o \
	grbl/motion_control.o \
	grbl/nuts_bolts.o \
	grbl/planner.o \
	grbl/print.o \
	grbl/probe.o \
	grbl/protocol.o \
	grbl/report.o \
	grbl/serial.o \
	grbl/settings.o \
	grbl/spindle_control.o \
	grbl/stepper.o \
	grbl/system.o \
	SPL/src/misc.o \
	SPL/src/stm32f10x_adc.o \
	SPL/src/stm32f10x_bkp.o \
	SPL/src/stm32f10x_can.o \
	SPL/src/stm32f10x_cec.o \
	SPL/src/stm32f10x_crc.o \
	SPL/src/stm32f10x_dac.o \
	SPL/src/stm32f10x_dbgmcu.o \
	SPL/src/stm32f10x_dma.o \
	SPL/src/stm32f10x_exti.o \
	SPL/src/stm32f10x_flash.o \
	SPL/src/stm32f10x_fsmc.o \
	SPL/src/stm32f10x_gpio.o \
	SPL/src/stm32f10x_i2c.o \
	SPL/src/stm32f10x_iwdg.o \
	SPL/src/stm32f10x_pwr.o \
	SPL/src/stm32f10x_rcc.o \
	SPL/src/stm32f10x_rtc.o \
	SPL/src/stm32f10x_sdio.o \
	SPL/src/stm32f10x_spi.o \
	SPL/src/stm32f10x_tim.o \
	SPL/src/stm32f10x_usart.o \
	SPL/src/stm32f10x_wwdg.o \
	src/system_stm32f10x.o \
	stm_usb_fs_lib/src/usb_core.o \
	stm_usb_fs_lib/src/usb_init.o \
	stm_usb_fs_lib/src/usb_int.o \
	stm_usb_fs_lib/src/usb_mem.o \
	stm_usb_fs_lib/src/usb_regs.o \
	stm_usb_fs_lib/src/usb_sil.o \
	usb/hw_config.o \
	usb/usb_desc.o \
	usb/usb_endp.o \
	usb/usb_istr.o \
	usb/usb_prop.o \
	usb/usb_pwr.o \
	util/stm32f10x_it.o

DIRS += \
	grbl/ \
	SPL/ \
	SPL/src/ \
	src/ \
	stm_usb_fs_lib/ \
	stm_usb_fs_lib/src/ \
	usb/ \
	util/

all: release

release_OBJS := $(addprefix build/olimexino_stm32/release/, $(OBJS))
release_DIRS := $(addprefix build/olimexino_stm32/release/, $(DIRS))

release: CPPFLAGS := -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -D_GRBL_ -DSTM32F103C8T6 -DSTM32F10X_MD -DLEDBLINK -DUSEUSB -DSTM32F103C8 -I./inc -I./src -I./cmsis -I./SPL/inc -I./SPL/src -I./util -I./usb -I./stm_usb_fs_lib/inc/ -I./grbl/
release: LDFLAGS := -Wl,--gc-sections -lm -Tstm32f103c8_flash.ld -mfloat-abi=soft -fno-strict-aliasing -nostartfiles -Wl,-Map=build/olimexino_stm32/release/GRBL_STM32.map
release: CFLAGS := -mthumb -mcpu=cortex-m3 -fdata-sections -ffunction-sections -Os -g2 -mfloat-abi=soft -fno-strict-aliasing 
release: ASFLAGS := -Wa,--no-warn
release: SUBDIR := build/olimexino_stm32/release
release: $(release_DIRS) $(release_OBJS) build/olimexino_stm32/release/src/startup_stm32f10x_md.o
	$(CC) -o $(SUBDIR)/GRBL_STM32.elf $(release_OBJS) $(SUBDIR)/src/startup_stm32f10x_md.o $(LDFLAGS)
	arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature $(SUBDIR)/GRBL_STM32.elf $(SUBDIR)/GRBL_STM32.hex
	arm-none-eabi-objcopy -O binary $(SUBDIR)/GRBL_STM32.elf $(SUBDIR)/GRBL_STM32.bin
	arm-none-eabi-size $(SUBDIR)/GRBL_STM32.elf

build/olimexino_stm32/release/src/startup_stm32f10x_md.o: src/startup_stm32f10x_md.S
	$(CC) $(ASFLAGS) -c src/startup_stm32f10x_md.S -o build/olimexino_stm32/release/src/startup_stm32f10x_md.o

build/olimexino_stm32/release/%.o : %.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

build/olimexino_stm32/release/% :
	mkdir -p $@

debug_OBJS := $(addprefix build/olimexino_stm32/debug/, $(OBJS))
debug_DIRS := $(addprefix build/olimexino_stm32/debug/, $(DIRS))

debug: CPPFLAGS := -DDEBUG -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -D_GRBL_ -DSTM32F103C8T6 -DSTM32F10X_MD -DLEDBLINK -DUSEUSB -DSTM32F103C8 -I./inc -I./src -I./cmsis -I./SPL/inc -I./SPL/src -I./util -I./usb -I./stm_usb_fs_lib/inc/ -I./grbl/
debug: LDFLAGS := -Wl,--gc-sections -lm -Tstm32f103c8_flash.ld -mfloat-abi=soft -fno-strict-aliasing -nostartfiles -Wl,-Map=build/olimexino_stm32/debug/GRBL_STM32.map
debug: CFLAGS := -mthumb -mcpu=cortex-m3 -Wall -fdata-sections -ffunction-sections -O0 -g3 -mfloat-abi=soft -fno-strict-aliasing -x c
debug: ASFLAGS := -Wa,--no-warn
debug: SUBDIR := build/olimexino_stm32/debug
debug: $(debug_DIRS) $(debug_OBJS) build/olimexino_stm32/debug/src/startup_stm32f10x_md.o
	$(CC) -o $(SUBDIR)/GRBL_STM32.elf $(debug_OBJS) $(SUBDIR)/src/startup_stm32f10x_md.o $(LDFLAGS)
	arm-none-eabi-objcopy -O ihex -R .eeprom -R .fuse -R .lock -R .signature $(SUBDIR)/GRBL_STM32.elf $(SUBDIR)/GRBL_STM32.hex
	arm-none-eabi-objcopy -O binary $(SUBDIR)/GRBL_STM32.elf $(SUBDIR)/GRBL_STM32.bin
	arm-none-eabi-size $(SUBDIR)/GRBL_STM32.elf

build/olimexino_stm32/debug/src/startup_stm32f10x_md.o: src/startup_stm32f10x_md.S
	$(CC) $(ASFLAGS) -c src/startup_stm32f10x_md.S -o build/olimexino_stm32/debug/src/startup_stm32f10x_md.o

build/olimexino_stm32/debug/%.o : %.c
	$(CC) -c $(CPPFLAGS) $(CFLAGS) -o $@ $<

build/olimexino_stm32/debug/% :
	mkdir -p $@

clean:
	rm -rf build/olimexino_stm32/release/ build/olimexino_stm32/debug/

