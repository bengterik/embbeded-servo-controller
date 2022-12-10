/*
 * TestLEDs.c
 *
 * Created: 2022-11-18 15:42:29
 * Author : tmk21als
 */


#ifndef __AVR_ATmega88__
#define __AVR_ATmega88__
#endif
#include <avr/io.h>



int main(void)
{
	
    int led;
	DDRC = DDRC | (1<<DDC1)| (1<<DDC0)| (1<<DDC2) | (1<<DDC3);
	
    led=0;
	
    while (1)
    {
		if (led < 2500) {
			PORTC = (1<<PC0);
		} else if (led < 5000) {
			PORTC = (1<<PC1);
		} else if (led < 7500) {
			PORTC = (1<<PC2);
		} else if (led < 10000) {
			PORTC = (1<<PC3);
		}
		led = (led+1)%10000;
	}
    return 0;
}

