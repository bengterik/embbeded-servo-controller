/*
 * MeasureMotor.c
 *
 * Created: 2022-12-07 14:46:37
 * Author : tmk21als
 */ 

#ifndef __AVR_ATmega88__
#define __AVR_ATmega88__
#endif

#define F_CPU 1000000
#define BAUD 2400
#define MYUBRR 25

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/atomic.h>

int init_LEDs(void);
int init_INTs(void);
int init_PWM(void);

int set_LED(int position, int value);
int updatePWM(int value);

//USART
void init_RxTx(void);
void USART_Init(unsigned int ubrr);
void USART_Transmit(char data);
unsigned char USART_Receive(void);
void send_speed(int value);
void send_int(int value, int chars);

//TIMER
#define COUNTER_BUF_SIZE 8
volatile unsigned int counter_register[COUNTER_BUF_SIZE];
volatile unsigned int cur_buff_index = 0;

volatile int AB = 3;
volatile int v = 255;



int main(void)
{
	init_LEDs();
	init_INTs();
	init_PWM();
	
	init_LEDs();
	init_RxTx();
	USART_Init(MYUBRR);
	
	sei(); // Globally enable interrupts
	
	//USART_Transmit('s');
	updatePWM(v);
	
	int led = 0;
	unsigned char c;

	set_LED(0,1);

	while (1)
    {
		c =	USART_Receive();
		
		USART_Transmit(c);

		switch (c) {
			case 's':
				//send_speed(counter_register[cur_buff_index]);
				break;
			case 'v':
				USART_Transmit((char) v);
				set_LED(1, !led);
				break;
			default:
				set_LED(3, 1);
				break;
		}
	}

    return 0;
}

void send_speed(int value) {
	char chars[4];
	sprintf(chars, "%d", value); 
	for(int i=0; i < 3; i++) {
		USART_Transmit(chars[i]);
	}
}

void send_int(int value, int chars) {
	char s[11];
	sprintf(s, "%d", value);
	for(int i=0; i < chars; i++) {
		USART_Transmit(s[i]);
	}
}

void USART_Transmit(char data) {
	while(!(UCSR0A & (1<<UDRE0))); // Wait for empty transmit buffer
	UDR0 = data;
}

unsigned char USART_Receive(void) {
	while (!(UCSR0A & (1<<RXC0))); // Checks RXEN0 flag (receive complete)
	return UDR0;
}

void init_RxTx(void){
	DDRD &= ~(1<<DDD0);	// Set PD0 set as input
	//PORTD &= (1<<PD0);
	
	DDRD |= (1<<DDD1);	//Set PD1 output
}

void USART_Init(unsigned int ubrr) {
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	//UCSR0C &= ~(1<<USBS0); // 1 stop bit
	UCSR0C = (3<<UCSZ00); // 8 bits
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

void set_timer_16(unsigned int i) {	
	ATOMIC_BLOCK(ATOMIC_FORCEON) // Block that disables interrupts and saves SREG
	{
		TCNT1 = i;	
	}
}

void register_time(unsigned int i) {
	counter_register[cur_buff_index%COUNTER_BUF_SIZE] = i;
	
	cur_buff_index++;	
}

ISR(PCINT1_vect, ISR_BLOCK)
{
	//register_time(TCNT1);
	//set_timer_16(0);
	
	unsigned char sreg = SREG ; // remember status register
	cli (); // disable interrupts
	
	int a, b;
	int oldV;
	a = (PIND & (1<<PIND7))>>PIND7; // Right-shift to get the read in first bit
	b = (PINC & (1<<PINC5))>>PINC5;
	set_LED(1, a);
	set_LED(2, b);
	
	oldV = v;
	pwm_duty_update(a, b);

	SREG = sreg ;
	sei();
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