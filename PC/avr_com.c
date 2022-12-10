#include "serialport.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG 0

unsigned char USART_receive(int fd) {
	unsigned char c;

	read(fd, &c, 1);
	if (DEBUG) printf("USART Received: |%c|\n", c);
	
	return c;
}

void USART_transmit(int fd, unsigned char *c) {
	if (DEBUG) printf("USART Sent: |%c|\n", *c);
	write(fd, c, 1);
}


int main(void)
{
	int sp;
	
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
		USART_transmit(sp, &term_in[0]);
				
		usart_in = USART_receive(sp);
		
		switch (usart_in) {
			case 'v':
				if (DEBUG) printf("Ask for PWM\n");
				usart_in = USART_receive(sp);
				printf("PWM was: %d\n", (int) usart_in);
				break;
			default:
				printf("DEFAULT REACHED\n");
				break;
		}
	}
	
	serial_cleanup(sp);
	return 1;
}

