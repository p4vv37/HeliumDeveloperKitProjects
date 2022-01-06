ARDUINO_PATH = /home/$(USER)/apps/arduino

AVR_BIN_DIR=$(ARDUINO_PATH)/hardware/tools/avr/bin
CXX=$(GCC_PATH)/bin/arm-none-eabi-g++ 
SRC=src/main/main
CWD = $(shell pwd)

RAK_PATH = $(CWD)/tmp/RAK-nRF52-Arduino-1.0.1
CMSIS_PATH = $(CWD)/tmp/ARM.CMSIS.5.7.0
GCC_PATH = $(CWD)/tmp/gcc-arm-none-eabi-9-2019-q4-major
ADAFRUIT_PATH = $(CWD)/tmp/Adafruit_BME680-2.0.1
ADAFRUIT_IO_PATH = $(CWD)/tmp/Adafruit_BusIO-1.9.9
ADAFRUIT_SENSOR_PATH = $(CWD)/tmp/Adafruit_Sensor-1.1.4
SX126_PATH = $(CWD)/tmp/SX126x-Arduino-2.0.8
WISBLOCK_PATH = $(CWD)/tmp/WisBlock

INCLUDE= -I "$(RAK_PATH)/cores/nRF5" 
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/nrfx/mdk"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/nrfx"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/" 
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/softdevice/s132_nrf52_6.1.1_API/include"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/nrfx/hal"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/nrfx"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/freertos/Source/include"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/freertos/config"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/freertos/portable/GCC/nrf52"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/freertos/portable/CMSIS/nrf52"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/freertos/portable/CMSIS/Core/Include"
INCLUDE+= -I "$(RAK_PATH)/cores/nRF5/nordic/nrfx/drivers/include"
INCLUDE+= -I "$(RAK_PATH)/libraries/Adafruit_TinyUSB_Arduino/src/arduino"
INCLUDE+= -I "$(RAK_PATH)/libraries/SPI"
INCLUDE+= -I "$(RAK_PATH)/variants/WisCore_RAK4631_Board" 

INCLUDE+= -I "$(WISBLOCK_PATH)/bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/lib/sdk/components/libraries/timer" 
INCLUDE+= -I "$(WISBLOCK_PATH)/bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/lib/sdk/components/libraries/util" 
INCLUDE+= -I "$(WISBLOCK_PATH)/bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/src" 
INCLUDE+= -I "$(WISBLOCK_PATH)/bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/src/boards" 
INCLUDE+= -I "$(WISBLOCK_PATH)/bootloader/RAK4630/Latest/WisCore_RAK4631_Bootloader/src/boards/wiscore_rak4631_board"

INCLUDE+= -I "$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino"
INCLUDE+= -I "$(ARDUINO_PATH)/hardware/arduino/avr/libraries/Wire/src"

INCLUDE+= -I "$(ADAFRUIT_PATH)" 
INCLUDE+= -I "$(ADAFRUIT_IO_PATH)" 
INCLUDE+= -I "$(ADAFRUIT_SENSOR_PATH)" 

INCLUDE+= -I "$(CMSIS_PATH)/CMSIS/Core/Include/" 
INCLUDE+= -I "$(SX126_PATH)/src" 

MCU=-mcpu=cortex-m4 #-- CPU
CPU_SPEED=-DF_CPU=16000000UL #-- clock speed of the board (mentioned on the quartz)
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections $(INCLUDE) 
CFLAGS+= -mthumb -c -g -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float -std=gnu++11 -fno-threadsafe-statics -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -w -x c++
CFLAGS+= -DF_CPU=64000000 -DARDUINO=10819 -DARDUINO_WisCore_RAK4631_Board -DARDUINO_ARCH_NRF52 "-DARDUINO_BSP_VERSION=\"1.0.1\"" -DDEBUG -DNRF52840_XXAA -DUSBCON -DUSE_TINYUSB -DUSB_VID=0x239A -DUSB_PID=0x8029 "-DUSB_MANUFACTURER=\"RAKwireless\"" "-DUSB_PRODUCT=\"WisBlock RAK4631\"" -DSOFTDEVICE_PRESENT -DARDUINO_NRF52_ADAFRUIT -DNRF52_SERIES -DDX_CC_TEE -DLFS_NAME_MAX=64 -Ofast -DCFG_DEBUG=0 -DCFG_LOGGER=1 -DCFG_SYSVIEW=0 

LINKER_SCRIPT  := nrf52.ld
LDFLAGS = -T$(LINKER_SCRIPT)

PORT=/dev/ttyACM0
SPEED=115200

.PHONY:
upload: $(SRC).hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application $(SRC).hex $(SRC).zip
	adafruit-nrfutil dfu serial --package $(SRC).zip -p $(PORT) -b $(SPEED)

.PHONY:
build: $(SRC).hex

$(SRC).hex: $(SRC).elf
	$(GCC_PATH)/bin/arm-none-eabi-objcopy -O ihex $< $@

$(SRC).elf: $(SRC).c

	@if [ -d "$(ARDUINO_PATH)" ]]; then\
		echo "Arduino need to be installed in $(ARDUINO_PATH)"; \
	fi

	@if [ -d "tmp" ]]; then\
        echo "tmp exists"; \
	else \
		mkdir tmp;\
	fi

	@if [ -d "$(CMSIS_PATH)" ]]; then\
        echo "$(CMSIS_PATH) exists"; \
	else \
		wget https://github.com/adafruit/arduino-board-index/releases/download/build-tools/ARM.CMSIS.5.7.0.zip -O tmp/CMSIS.zip;\
		unzip tmp/CMSIS.zip -d tmp;\
	fi

	@if [ -d "$(RAK_PATH)" ]]; then\
        echo "$(RAK_PATH) exists"; \
	else \
		wget https://github.com/RAKWireless/RAK-nRF52-Arduino/archive/1.0.1.tar.gz?raw=true -O tmp/nRF52.tar.gz;\
		tar -xzf tmp/nRF52.tar.gz -C tmp;\
	fi

	@if [ -d "$(GCC_PATH)" ]; then \
        echo "$(GCC_PATH) exists"; \
	else \
		wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -O tmp/gcc.tar.bz2;\
		tar -xvjf tmp/gcc.tar.bz2 -C tmp;\
	fi
	
	@if [ -d "$(ADAFRUIT_PATH)" ]; then \
        echo "$(ADAFRUIT_PATH) exists"; \
	else \
		wget https://github.com/adafruit/Adafruit_BME680/archive/refs/tags/2.0.1.tar.gz -O tmp/ada.tar.gz;\
		tar -xzf tmp/ada.tar.gz -C tmp;\
	fi	

	@if [ -d "$(ADAFRUIT_IO_PATH)" ]; then \
        echo "$(ADAFRUIT_IO_PATH) exists"; \
	else \
		wget https://github.com/adafruit/Adafruit_BusIO/archive/refs/tags/1.9.9.tar.gz -O tmp/adaio.tar.gz;\
		tar -xzf tmp/adaio.tar.gz -C tmp;\
	fi	

	@if [ -d "$(ADAFRUIT_SENSOR_PATH)" ]; then \
        echo "$(ADAFRUIT_SENSOR_PATH) exists"; \
	else \
		wget https://github.com/adafruit/Adafruit_Sensor/archive/refs/tags/1.1.4.tar.gz -O tmp/adasensor.tar.gz;\
		tar -xzf tmp/adasensor.tar.gz -C tmp;\
	fi		

	@if [ -d "$(SX126_PATH)" ]; then \
        echo "$(SX126_PATH) exists"; \
	else \
		wget https://github.com/beegee-tokyo/SX126x-Arduino/archive/refs/tags/2.0.8.tar.gz -O tmp/sx.tar.gz;\
		tar -xzf tmp/sx.tar.gz -C tmp;\
	fi	
	
	@if [ -d "$(WISBLOCK_PATH)" ]; then \
        echo "$(WISBLOCK_PATH) exists"; \
	else \
		git clone https://github.com/RAKWireless/WisBlock tmp/WisBlock;\
	fi	
	$(CXX) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	$(GCC_PATH)/bin/arm-none-eabi-size $(SRC).elf

.PHONY:
clean:
	@echo Cleaning ...
	rm -fr tmp/*
	rm -f $(SRC).elf
	rm -f $(SRC).hex
.PHONY:
reset:
	@echo Cleaning ...
	rm -f $(SRC).elf
	rm -f $(SRC).hex