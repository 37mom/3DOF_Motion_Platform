/*
 * ADC.h
 *
 * Created: 3/30/2018 1:46:35 PM
 *  Author: moham
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "bit_fiddling.h"
#include <avr/io.h>

void ADC_init(void);
uint16_t ADC_read(uint8_t channel_num);
void ADC_startConversion(void);


#endif /* ADC_H_ */