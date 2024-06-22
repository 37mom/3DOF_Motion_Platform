/*
 * ADC.c
 *
 * Created: 3/30/2018 1:46:15 PM
 *  Author: moham
 */ 

#include "ADC.h"
#include <avr/io.h>

void ADC_init(void)
{
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1; //64 prescalar
	ADMUX |= 1<<REFS0;
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;
	ADCSRA |= 1<<ADSC;				//Start ADC
	//ADCSRA |=(1<<ADEN) | (1<<ADPS0) ;
}

uint16_t ADC_read(uint8_t channel_num)
{
	/*channel_num = channel_num>>5 ;	// get the first 3 bits 
	ADMUX &= 0xE0 ;					//clear first 5 bits in the ADMUX*/
	channel_num &= 0x07; /* channel number must be from 0 --> 7 */
	ADMUX &= 0xE0; /* clear first 5 bits in the ADMUX (channel number MUX4:0 bits) before set the required channel */
	ADMUX = ADMUX | channel_num ;
	/*ADCSRA |=(1<<ADSC) ;  //start conversion 
	(GET_BIT(ADCSRA,ADIF));
	SET_BIT(ADCSRA,ADIF);*/
	//SET_BIT(SFIOR,ADTS1);
	
	return ADCW ;
}

void ADC_startConversion(void)
{
	ADCSRA |=(1<<ADSC) ;
}