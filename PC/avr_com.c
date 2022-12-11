// Compile with gcc -Wall -std=c99 -o output.exe avr_com serial*

#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG 0

int sp;

int receive_int();

unsigned char USART_receive() {
	unsigned char c;

	read(sp, &c, 1);
	if (DEBUG) printf("USART Received: |%c| = |%d|  \n", c, c);
	
	return c;
}

void USART_transmit(unsigned char *c) {
	if (DEBUG) printf("USART Sent: |%c| = |%d|\n", *c, *c);
	write(sp, c, 1);
}


int main(void)
{
	/*Declaration of variables*/
	
	/*Initialise serial port */
	sp = serial_init("/dev/ttyS0",0);
	if(sp == 0)
	{
		printf("Error! Serial port could not be opened.\n");
	}
	else
	{
		printf("Serial port open with identifier %d \n",sp);
	}
		
	unsigned char term_in[1];
	unsigned char usart_in;
	
	while (1) {
		printf("Menu\n");
		scanf("%s", term_in);
		USART_transmit(&term_in[0]);
				
		usart_in = USART_receive(); // An "ACK" with the operation the AVR is about to perform
		
		switch (usart_in) {
			case 's':
				while(1) {
					int i = receive_int();
					printf("%d\n", i);
				}
			
			case 'v':
				if (DEBUG) printf("Ask for PWM\n");
				usart_in = USART_receive();
				printf("PWM was: %d\n", (int) usart_in);
				break;
				
			case 'd':
				if (DEBUG) printf("Timer counter polled\n");
				while(1) {
					printf("%d\n", receive_int());
				}
				
				break;
				
			default:
				printf("DEFAULT REACHED\n");
				break;
		}
	}
	
	/*Close the serial port */	
	serial_cleanup(sp);
	return 1;
}

int receive_int() {
	int value = 0;
	unsigned char bytes[2];
	
	for(int i = 0; i < 2; i++) {
		bytes[i] = USART_receive();
		value |= bytes[i] << (8*i);
		if (DEBUG) printf("RECEIVE_INT: value is %d\n", value);
	}
	
	return value;
}
