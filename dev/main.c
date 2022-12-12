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
void send_int(unsigned int value);
unsigned char USART_Receive(void);

//TIMER
#define COUNTER_BUF_SIZE 8

void init_timer_16(void);
void register_time(int i);

// SPEED
volatile unsigned int counter_register[COUNTER_BUF_SIZE];
volatile unsigned int cur_buff_index = 0;
unsigned int average_ticks();
unsigned int rpm();

volatile int AB = 3;
volatile int v = 255;

int main(void)
{
	MCUSR &= ~(1<<WDRF);
	//WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;

	init_LEDs();
	init_INTs();
	init_PWM();
	
	init_LEDs();
	init_RxTx();
	USART_Init(MYUBRR);

	init_timer_16();
	
	
	sei(); // Globally enable interrupts
		
	unsigned char c;

	set_LED(0,1);
	_delay_ms(100);
	set_LED(0,0);
	set_LED(1,1);
	_delay_ms(100);
	set_LED(1,0);
	set_LED(2,1);
	_delay_ms(100);
	set_LED(2,0);
	set_LED(3,1);
	_delay_ms(1000);
	set_LED(3,0);
	

	while (1)
    {
		c =	USART_Receive();
		
		USART_Transmit(c);

		switch (c) {
			case 's':
				
				break;
			case 'v':
				USART_Transmit((char) v);
				set_LED(1, 1);
				break;

			case 'd':
				set_LED(2, 1);
				for(int i = 0; i < COUNTER_BUF_SIZE; i++) {
					//send_int(counter_register[i]);
				}

				break;
			default:
				set_LED(3, 1);
				break;
		}
	}

	while(1) {
		set_LED(3,1);
		_delay_ms(100);
		set_LED(3,0);
		set_LED(2,1);
		_delay_ms(100);
		set_LED(2,0);
		set_LED(1,1);
		_delay_ms(100);
		set_LED(1,0);
		set_LED(0,1);
		_delay_ms(100);
		set_LED(0,0);
	}

	return 0;

}

unsigned int rpm() {
	return 60*F_CPU/(average_ticks()*8);
}

unsigned int average_ticks() {
	unsigned int sum = 0;
	for(int i = 0; i < COUNTER_BUF_SIZE; i++) {
		sum += counter_register[i];
	}
	return sum/COUNTER_BUF_SIZE;
}

void send_int(unsigned int value) {
	unsigned char bytes[2];

	for(int i = 0; i < 2; i++) {
		bytes[i] = (char) (value >> (i*8));
		USART_Transmit(bytes[i]);
	}
}

ISR(PCINT1_vect, ISR_BLOCK)
{
	int i = TCNT1; // Read timer
	
	counter_register[cur_buff_index%COUNTER_BUF_SIZE] = i; // Store timer value in buffer
	cur_buff_index++; 
	send_int(cur_buff_index);
	
	TCNT1 = 0;	// Timer = 0
}

ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect)); // Redirect interrupt on PCINT2_vect PCINT1_vect routine (no need to copy the same code)

void init_timer_16(void) {
	TCCR1B |= (1<<CS11); // Prescaler 8
	
	TIFR1 |= (1<<TOV1); // Clear overflow flag

	TIMSK1 |= (1<<TOIE1); // Enable overflow interrupt
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

int updatePWM(int value)
{
	OCR0A = value;
	OCR0B = value;
	return value;
}

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