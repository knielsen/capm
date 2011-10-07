capm.hex: capm.c
	avr-gcc -O2 -g -mmcu=atmega328p -DF_CPU=16000000UL -I. capm.c -o capm.elf
	avr-objcopy -O ihex -R .eeprom -S capm.elf capm.hex

capm-upload: capm.hex
	avrdude -vD -carduino -b57600 -pm328p -P/dev/ttyUSB0 -Uflash:w:capm.hex:i


serial-test.hex: serial-test.c
	avr-gcc -O2 -g -mmcu=atmega328p -DF_CPU=16000000UL -I. serial-test.c -o serial-test.elf
	avr-objcopy -O ihex -R .eeprom -S serial-test.elf serial-test.hex

serial-test-upload: serial-test.hex
	avrdude -vD -carduino -b57600 -pm328p -P/dev/ttyUSB0 -Uflash:w:serial-test.hex:i

blink.hex: blink.c
	avr-gcc -O2 -g -mmcu=atmega328p -DF_CPU=16000000UL -I. blink.c  -o blink.elf
	avr-objcopy -O ihex -R .eeprom -S blink.elf blink.hex

blink-upload: blink.hex
	avrdude -vD -carduino -b57600 -pm328p -P/dev/ttyUSB0 -Uflash:w:blink.hex:i

tty:
	stty -F/dev/ttyUSB0 raw -echo -hup cs8 -parenb -cstopb 9600
