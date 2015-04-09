
flags = -Os
 
makeCompile: Sigmux.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/security.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/wlan.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/socket.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/hci.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/cc3000_common.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/evnt_handler.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/netapp.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/nvmem.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c ./CC3000/spi.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -c Sigmux.c
	avr-gcc -g $(flags) -mmcu=atmega32u4 -o Sigmux.elf ./Sigmux.o ./spi.o ./socket.o ./wlan.o ./nvmem.o ./cc3000_common.o ./evnt_handler.o ./hci.o ./netapp.o ./security.o
	rm *.o
	avr-objcopy -j .text -j .data -O ihex Sigmux.elf Sigmux.hex
 
upload: Sigmux.hex
	sudo avrdude -c USBtiny -p atmega32u4 -U flash:w:Sigmux.hex:i
 
setExtCLK:
	sudo avrdude -c USBtiny -p atmega32u4 -U lfuse:w:0xde:m -U hfuse:w:0x99:m -U efuse:w:0xf3:m
 
setIntCLK:
	sudo avrdude -c USBtiny -p atmega32u4 -U lfuse:w:0xd2:m -U hfuse:w:0x99:m -U efuse:w:0xf3:m
 
setDefault:
	sudo avrdude -c USBtiny -p atmega32u4 -U lfuse:w:0x5e:m -U hfuse:w:0x99:m -U efuse:w:0xf3:m
 
build_lib:
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/security.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/wlan.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/socket.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/hci.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/cc3000_common.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/evnt_handler.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/netapp.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/nvmem.c
	avr-gcc -g -O3 -mmcu=atmega32u4 -c ./CC3000/spi.c
	rm *.o         
 
clean:
	rm *.o
	rm *.elf

