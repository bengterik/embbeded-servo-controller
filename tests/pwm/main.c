/*
 * TestLEDs.c
 *
 * Created: 2022-11-18 15:42:29
 * Author : tmk21als
 */ 

#ifndef __AVR_ATmega88__
#define __AVR_ATmega88__
#endif

#define F_CPU 1000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int init_LEDs(void);
int init_INTs(void);
int init_PWM(void);

int set_LED(int position, int value);
int updatePWM(int value);

volatile int AB = 3;
volatile int v = 0;

int main(void)
{
	init_LEDs();
	init_INTs();
	init_PWM();
	
	sei(); // Globally enable interrupts
	updatePWM(10);
	while (1)
    {
		_delay_ms(500);
	}
    return 0;
}

void pwm_duty_update(int a, int b) {
	char newAB = (a<<1) | b;
	
	switch (AB) {
		case 0: if(newAB==1) v+=1; else v-=1; break;
		case 1: if(newAB==3) v+=1; else v-=1; break;
		case 3: if(newAB==2) v+=1; else v-=1; break;
		case 2: if(newAB==0) v+=1; else v-=1; break;
	}
	
	if (v > 255) {
		v = 255;
	} else if (v < 0) {
		v = 0;
	}
	
	AB = newAB;
}

int updatePWM(int value)
{
	OCR0A = value;
	OCR0B = value;
	return value;
}

ISR(PCINT1_vect, ISR_BLOCK)
{
	int a, b;
	int oldV;
	a = (PIND & (1<<PIND7))>>PIND7; // Right-shift to get the read in first bit
	b = (PINC & (1<<PINC5))>>PINC5;
	
	set_LED(3,a);
	set_LED(2,b);
	
	oldV = v;
	pwm_duty_update(a, b);
	
	if (v-oldV < 0) {
		set_LED(1,0);
		set_LED(0,1);
	} else if (v-oldV > 0){
		set_LED(0,0);
		set_LED(1,1);
	} else {
		set_LED(0,0);
		set_LED(1,0);
	} 	
}

ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect)); // Redirect interrupt on PCINT2_vect PCINT1_vect routine (no need to copy the same code)


int init_LEDs(void)
{
	DDRC = DDRC | (1<<DDC1)| (1<<DDC0)| (1<<DDC2) | (1<<DDC3); // Init LEDs
	PORTC &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3));	// Initially all LED pins set to 0
	return 1;
}

int init_INTs(void)
{
	DDRD &= ~(1<<DDD7);	// "PCINT" set as input
	PORTD &= (1<<PD7);
	
	DDRC &= ~(1<<DDC5);	// "FT speed" set as input
	PORTC &= (1<<PC5);
	
	
	PCMSK1 |= (1<<PCINT13);   // PCINT13 "FT speed" enabled
	PCMSK2 |= (1<<PCINT23);   // PCINT23 "PCINT" enabled
	
	PCICR |= (1<<PCIE1);	// The PC interrupt group 1 (PCINT8 -> PCINT14) enabled
	PCICR |= (1<<PCIE2);	// The PC interrupt group 2 (PCINT16 -> PCINT23) enabled
	return 1;
}

int init_PWM(void)
{
	DDRD |= (1<<DDD5);	//Set PIND5 output
	TCCR0A |= 0b10110011;			//Configure fast PWM mode, non-inverted output on OCA and inverted output on OCB
	TCCR0B |= 0x01;					//Internal clock selector, no prescaler
	return 1;
}

int set_LED(int led, int value)
{
	switch(led)
	{
		case 0:
			if (value == 0)
			{
				PORTC &= ~(1 << PC0);
			}
			else
			{
				PORTC |= (1 << PC0);
			}
			break;
		case 1:
			if (value == 0)
			{
				PORTC &= ~(1 << PC1);
			}
			else
			{
				PORTC |= (1 << PC1);
			}
			break;
		case 2:
			if (value == 0)
			{
				PORTC &= ~(1 << PC2);
			}
			else
			{
				PORTC |= (1 << PC2);
			}
			break;
		case 3:
			if (value == 0)
			{
				PORTC &= ~(1 << PC3);
			}
			else
			{
				PORTC |= (1 << PC3);
			}
			break;
		
	}
	return 1;
}


