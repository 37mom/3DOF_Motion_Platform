/*
 * Timers.c
 *
 * Created: 3/29/2018 3:26:42 PM
 *  Author: moham
 */ 
#include "avr/io.h"
#include "Timers.h"

//------TIMER for main loop timing--------------
//with prescale 8
//At 8MHz 0.01s = 10 000 unit
void init_loop_timer(void)
{
	TCCR1A=0X00;
	TCCR1B |=1<<CS11;
	TCCR1C =0x00;
}

//--------------TIMER for PWM-------------------
// fpwm = (F_CPU)/(prescalar*(1+TOP)) ;
//PIN 0C0-OC2 inverted FAST PWM with prescale 8. This means a ~3.9kHz signal to the motor controller.
//PIN 0C0-OC2 inverted FAST PWM with prescale 1. This means a ~31.25kHz signal to the motor controller.

void int_PWM_timers(void)
{
	/*TCCR2 |=1<<WGM21 | 1<<WGM20 | 1<<COM21 | 1<<CS20;
	TCCR0 |=1<<WGM01 | 1<<WGM00 | 1<<COM01 | 1<<CS00;
	OCR2 = 0; OCR0 = 0;*/
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1)|(1<<WGM31);        //NON Inverted PWM
	TCCR3B |= (1<<WGM33)|(1<<WGM32)|(1<<CS30);
	ICR3 =250 ;
	DDRE|=(1<<PINE3)|(1<<PINE4)|(1<<PINE3);   //PWM Pins as Out
	OCR3C = 0;
	OCR3B = 0;
	OCR3A = 0;
	
}