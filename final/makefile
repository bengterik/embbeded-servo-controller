main: compile flash
compile:
	avr-gcc -Woverflow  -mmcu=atmega88 -O -Wall -o main.elf main.c

flash:
	avr-objcopy -Osrec main.elf main.sr
	avrdude -e -p atmega88 -P usb -c avrispmkII -U flash:w:main.sr:a 
clean: 
	rm main.sr main.elf
