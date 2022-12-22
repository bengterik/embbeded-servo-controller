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
#include <inttypes.h>

#define COUNTER_BUF_SIZE 16
#define PRESCALER 8

#define TICK_LOWER_BOUND 200
#define TICK_UPPER_BOUND 17000

#define CONTROL_INTERVAL 16 //Derived from Timer2 prescaler, in ms

#define ANAOLG_CHANGE_THRESHOLD 8

#define I_SAT_UPR 50
#define I_SAT_LOWR -50

volatile unsigned int counter_register[COUNTER_BUF_SIZE];
volatile unsigned int cur_buff_index = 0;

volatile long timer_2_counter = 0;

volatile int AB = 0;
volatile int duty = 0;
volatile int aw_speed = 0;

// Flags
volatile int f_rec_speed = 0;
volatile int f_send_rpm = 0;

// Fixed point
#define SHIFT_AMOUNT 8
typedef int16_t fp_float; // Q8.8 signed floating point number

// Control variables
volatile int ref = 20;
fp_float I = 0;
fp_float Kp = 0x0100; // 1
fp_float Ki = 0x0200; // 2

#define CONTROL_INTEGRAL_CONSTANT 0x0004 // 0b 0000 0000 . 0000 0100 = 0.015625

volatile int prev_adc = 128;
volatile int nbr_ints = 0;

unsigned long ticks_sum();

fp_float fp_mul(fp_float a, fp_float b) {
	uint32_t temp = a * b;
    int8_t a_int = a >> SHIFT_AMOUNT;
    int8_t b_int = b >> SHIFT_AMOUNT;

    // Overflow check
    if (a_int>=0 && b_int>=0 && temp > 0x7F0000) { // Both positive, max 127
        temp = 0x7F0000;
    } else if (a_int<0 && b_int<0 && temp > 0x7F000) { // Both negative, max 127
        temp = 0x7F0000;
    } else if (((a_int<0 && b_int>=0) || (a_int>=0 && b_int<0)) && temp < 0x800000) { // Different signs, min -128
        temp = 0x800000;
    }

    fp_float res = temp >> SHIFT_AMOUNT;
	return res;
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
	
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0); // Enable RX, TX and RX interrupt
	//UCSR0C &= ~(1<<USBS0); // 1 stop bit
	UCSR0C = (3<<UCSZ00); // 8 bits
	// enable interrupt on data receive
}

void send_int(unsigned int value) {
	unsigned char bytes[2];

	for(int i = 0; i < 2; i++) {
		bytes[i] = (char) (value >> (i*8));
		USART_Transmit(bytes[i]);
	}
}

void init_timer_16(void) {
	TCCR1B |= (1<<CS11); // Prescaler 8
	
	TIFR1 |= (1<<TOV1); // Clear overflow flag

	TIMSK1 |= (1<<TOIE1); // Enable overflow interrupt // IF NOT CAUGHT WILL RESET
}

void init_timer_8(void) {
	TCCR2B |= (1<<CS22);// | (1<<CS21); //| (1<<CS20); // Prescaler 8
	
	TIFR2 |= (1<<TOV2); // Clear overflow flag

	TIMSK2 |= (1<<TOIE2); // Enable overflow interrupt // IF NOT CAUGHT WILL RESET
}

int update_pwm(int pwm) {
	duty = pwm;
	OCR0B = pwm;
	return 0;
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
	TCCR0B |= 0x11;					//Internal clock selector, no prescaler
	//TIMSK0 |= (1<<TOIE1); // Enable OVF interrupt
	return 1;
}

int init_encoder(void){
	int a, b;
	a = (PIND & (1<<PIND7))>>PIND7; // Right-shift to get the read in first bit
	b = (PINC & (1<<PINC5))>>PINC5;
	
	AB = (a<<1) | b;
	return 0;
}

int init_adc(void)
{
	DDRC &= ~(1<<DDC4); // Set ADC4 as input

	ADMUX = (1<<REFS0) | (1<<ADLAR) | (1<<MUX2); // AVcc as reference and ADC4 as channel
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (0b111<<ADPS0); // Enable, start, Auto-trigger, int. enable, prescaler 128
	ADCSRB = 0; // Free running mode
	
	return 0;
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

ISR(PCINT1_vect, ISR_BLOCK)
{
	int i = TCNT1; // Read timer

	int index = cur_buff_index%COUNTER_BUF_SIZE;

	if (i > TICK_LOWER_BOUND && i < TICK_UPPER_BOUND) { //
		counter_register[index] = i; // Store timer value in buffer
	} else {
		int previous_index = (index+COUNTER_BUF_SIZE-1)%COUNTER_BUF_SIZE; // Some arithmetic to avoid negative indices
		counter_register[index] = counter_register[previous_index]; // If value is outside range take previous value
	}
	
	cur_buff_index++; 
	
	TCNT1 = 0;	// Timer = 0
}

ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect)); // Redirect interrupt on PCINT2_vect PCINT1_vect routine (no need to copy the same code)

unsigned long ticks_sum() {
	unsigned long sum = 0;
	for(int i = 0; i < COUNTER_BUF_SIZE; i++) {
		sum += counter_register[i];
	}
	return sum;
}

unsigned char rpm() {
	return (60*F_CPU*COUNTER_BUF_SIZE)/((long) ticks_sum()*PRESCALER*96);
}

ISR(USART_RX_vect, ISR_BLOCK){
	unsigned char c = USART_Receive();
	if (f_rec_speed == 1) {
		ref = c;
		f_rec_speed = 0;
		set_LED(3,0);
	} else if (c == 'r') {
		set_LED(3,1);
		f_send_rpm = 1;
		f_rec_speed = 1;
	} else if (c == 's') {
		f_send_rpm = 1;
	}
}

ISR(ADC_vect, ISR_BLOCK){
	unsigned char adc = ADCH;
	signed int diff = prev_adc - adc;

	if (diff > ANAOLG_CHANGE_THRESHOLD) {
		ref++;
		prev_adc = adc;
	} else if (diff < -ANAOLG_CHANGE_THRESHOLD) {
		ref--;
		prev_adc = adc;
	}
	//send_int(0x00 | ref);
}

ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
	/* Timer overflow means that the motor is standing still but
	 * won't update RPM as there are no encoder interrupts */
	for(int i = 0; i < COUNTER_BUF_SIZE; i++) {
		counter_register[i] = 65535;
	}
}

fp_float sat(int x, int min, int max) {
	if (x < min) {
		return min;
	} else if (x > max) {
		return max;
	} else {
		return x;
	}
}

void control(){
	signed int y;
	fp_float e;
	int p;
	
	y = rpm();
	e = (int) ref - y;
	p = fp_mul((fp_mul(Kp, e) + fp_mul(Ki,I)), 2.125)>>SHIFT_AMOUNT; // Correct rounding?
	if (p < 0 || p > 65000) p = 0; // might overflow depending on type
	if (p > 255) p = 255;
	update_pwm(p);

	fp_float integral = fp_mul(e<<SHIFT_AMOUNT,CONTROL_INTEGRAL_CONSTANT); // e * (Ki * INTERVAL / 1000)
	I += integral;
	//I = sat(I + integral, I_SAT_LOWR, I_SAT_UPR);
}

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	control();
}

void startup_led_loop() {
	for (int i = 0; i < 4; i++) {
		set_LED(i, 1);
		_delay_ms(10);
		set_LED(i, 0);
	}
}

int main(void){
	init_LEDs();
	init_INTs();
	init_PWM();
	
	init_LEDs();
	init_RxTx();
	USART_Init(MYUBRR);

	init_timer_8();
	init_timer_16();

	init_encoder();

	init_adc();
		
	startup_led_loop();

	sei(); // Globally enable interrupts


	while (1)
    {
		if (f_send_rpm == 1) {
			send_int(0x00 | rpm());
			f_send_rpm = 0;
		} 
	}

	return 0;

}