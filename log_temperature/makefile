SRC=src/main/main
CXX=avr-gcc
INCLUDE=~/apps/arduino/hardware/arduino/avr/cores/arduino/*

#-- configurations for ATmega328P
MCU=-mmcu=atmega328p #-- CPU
CPU_SPEED=-DF_CPU=16000000UL #-- clock speed of the board (mentioned on the quartz)
CFLAGS=$(MCU) $(CPU_SPEED) -Os -w -Wl,--gc-sections -ffunction-sections -fdata-sections -include $(INCLUDE)

.PHONY:
build: $(SRC).hex

$(SRC).hex: $(SRC).elf
	avr-objcopy -O ihex $< $@

$(SRC).elf: $(SRC).c
	test -f 1.0.1.tar.gz || rm 1.0.1.tar.gz
	test -f $@ || wget https://github.com/RAKWireless/RAK-nRF52-Arduino/archive/1.0.1.tar.gz?raw=true
	$(CXX) $(CFLAGS) $^ -o $@
	avr-size -C $(SRC).elf

.PHONY:
clean:
	@echo Cleaning ...
	rm -f $(SRC).elf
	rm -f $(SRC).hex
	rm -f *.o
	@echo "done"

