// Compile with gcc -Wall -std=c99 -o output.exe serial*

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
		
	//unsigned char term_in[1];
	//unsigned char usart_in;
	
	char term_in;
	int speed_in;
	
	/*
	// Loopback test
	
	*/
	
	while (1) {
		sleep(1);
		printf("\n\nInput char+Enter with following options\n");
		printf("\n");
		printf("'r' to set speed\n");
		printf("'s' for current rpm\n");
		printf("'d' to enter debug mode\n");
		printf("'l' for loopback test\n");
		scanf(" %c", &term_in); 
		USART_transmit((unsigned char *)&term_in);
		
		if (term_in == 'r') { // Set speed
			printf("\nEnter speed between 10-120\n");
			scanf(" %d", &speed_in);
			USART_transmit((unsigned char *) &speed_in);
			printf("RPM set to %d\n", speed_in);

		} else if (term_in == 's') { // Ask for speed
			printf("RPM is %u\n", receive_int());
		
		} else if (term_in == 'd'){	// Debug
			while (1) {
				printf("%d\n", receive_int());
			}
		} else if (term_in == 'l'){	// Debug
			while(1) {
				printf("Enter character\n");
				scanf(" %c", &term_in); // 
				USART_transmit((unsigned char *)&term_in);
				printf("Received %c\n", USART_receive());
			}
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
