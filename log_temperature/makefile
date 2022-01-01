SRC=src/main/main
CXX=avr-gcc
INCLUDE=~/apps/arduino/hardware/arduino/avr/cores/arduino/*
INCLUDE:=tmp/RAK-nRF52-Arduino-1.0.1/cores/nRF5/*
INCLUDE:=tmp/RAK-nRF52-Arduino-1.0.1/variants/WisCore_RAK4631_Board/*
INCLUDE:=tmp/RAK-nRF52-Arduino-1.0.1/cores/nRF5/nordic/nrfx/mdk/nrf.h

#-- configurations for ATmega328P
MCU=-mmcu=atmega328p #-- CPU
CPU_SPEED=-DF_CPU=16000000UL #-- clock speed of the board (mentioned on the quartz)
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections -include $(INCLUDE) -DNRF51

.PHONY:
build: $(SRC).hex

$(SRC).hex: $(SRC).elf
	avr-objcopy -O ihex $< $@

$(SRC).elf: $(SRC).c
	wget https://github.com/RAKWireless/RAK-nRF52-Arduino/archive/1.0.1.tar.gz?raw=true -O tmp/nRF52.tar.gz
	tar -xzf tmp/nRF52.tar.gz -C tmp
	$(CXX) $(CFLAGS) $^ -o $@
	avr-size -C $(SRC).elf

.PHONY:
clean:
	@echo Cleaning ...
	rm -f tmp/*
	rm -f $(SRC).elf
	rm -f $(SRC).hex
	rm -f *.o
	@echo "done"

