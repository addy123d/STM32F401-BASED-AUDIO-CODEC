all: flash

flash: build/firmware.bin
	st-flash write build/firmware.bin 0x08000000

build/firmware.bin: build/firmware.elf
	arm-none-eabi-objcopy -O binary build/firmware.elf build/firmware.bin

build/firmware.elf: build/main.o
	arm-none-eabi-ld -T STM32F401CCU6.ld build/main.o -o build/firmware.elf

build/main.o: build | main.s
	cd build && arm-none-eabi-as -mcpu=cortex-m4 ../main.s -o main.o


build:
	rm -rf build && mkdir build

clean:
	rm -rf build
