/*
 * Motor_positionControl_128.c
 *
 * Created: 6/20/2018 12:16:07 AM
 * Author : moham
 */ 

#ifdef F_CPU
#undef F_CPU
#define F_CPU 8000000UL
#endif

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/atomic.h> 
#include "util/delay.h"
#include <avr/interrupt.h>
#include "Timers.h"
#include "motorController.h"
#include "ADC.h"
#include "USART_128.h"
#include "data_types.h"
#include "pid.h"


#define loopTIME						10000	//10ms
#define Hz								100
#define CHANNEL_NUM						 0

PID_t position_pid[3] ={0};

float dt = 0;

float filter[2]={0};
float filter2[2]={0};
float filter3[2]={0};
	
uint32_t ADC_accu[3] = {0} ;
float       Angle[3] = {0} ;
int       PWMDuty[3] = {0} ;
uint16_t  counter[3] = {0} ;
float    setPoint[3] = {0};
uint8_t  UarTemp=0 ;

char buffer[20];

void LPfilter(float rawDATA, float *filteredARRAY, float LPFgain);
float calc_Angle(float ADC_value);
void receiveAngles(char *str,float *receivedSetPoints);
void cleanArr(char* StringPtr);



int main(void)
{
	float kp=  5  ;    float kp2=  5  ;	 float  kp3 =  5  ;    
	float ki = 0.0  ;  float ki2 = 0.0  ; float ki3 = 0.0  ;
	float kd = 0.031 ; float kd2 = 0.031 ;float kd3 = 0.031 ;
	
	init_loop_timer() ;
	int_PWM_timers() ;
	init_MotorController();
	ADC_init();
	USART_init(0);
	sei(); // Enable Global interrupts
	
	pid_init(&position_pid[0],kp,ki,kd);
	pid_set(&position_pid[0] ,setPoint[0]);
	
	pid_init(&position_pid[1],kp2,ki2,kd2);
	pid_set(&position_pid[1] ,setPoint[1]);
	
	pid_init(&position_pid[2],kp3,ki3,kd3);
	pid_set(&position_pid[2],setPoint[2]);
	_delay_ms(2000);
	
	setPoint[0]=  Angle[0];
	setPoint[1]=  Angle[1];
	setPoint[2]=  Angle[2];
	while (1)
	{
		
		if (TCNT1>=loopTIME)
		{
			dt = (float)TCNT1/1000000;
			TCNT1 = 0;
			
			PWMDuty[0] =pid_calculate(&position_pid[0],(int)Angle[0]-setPoint[0],dt);
			PWMDuty[1] =pid_calculate(&position_pid[1],(int)Angle[1]-setPoint[1],dt);
			PWMDuty[2] =pid_calculate(&position_pid[2],(int)Angle[2]-setPoint[2],dt);
			driveMotor(PWMDuty[0],PWMDuty[1],PWMDuty[2]);
			
			pid_set(&position_pid[0] ,0);
			pid_set(&position_pid[1], 0);
			pid_set(&position_pid[2] ,0);
		}
	}
}




void LPfilter(float rawDATA, float *filteredARRAY, float LPFgain)
{
	filteredARRAY[0] = rawDATA*LPFgain + filteredARRAY[1]*(1-LPFgain);
	filteredARRAY[1] = filteredARRAY[0];
}

float calc_Angle(float ADC_value)
{
	float Angle = 0;
	Angle = 360 * ADC_value / 1024;
	return Angle;
}

void receiveAngles(char *str,float *receivedSetPoints)
{
	uint8_t i =0 ;
	char counter = 0 ;
	char strSetPoint[5]= {'\0'} ;
	char strSetPoint2[5]={'\0'} ;
	char strSetPoint3[5]={'\0'} ;
	float receivedAngles[3]={0} ;
		
	while (*str !=0x00)
	{
		if (*str != ',')
		{
			switch(counter)
			{
				case 0:
				strSetPoint[i]=*str;
				break ;
				case 1:
				strSetPoint2[i]=*str;
				break ;
				case 2:
				strSetPoint3[i]=*str;
				break ;
			}
			i++;
		}
		else
		{
			counter++;
			i=0 ;
		}
		str++ ;
	}
/*
	USART_Transmit_string(strSetPoint,0);	USART_Transmit_string("  ,  ",0) ;
	USART_Transmit_string(strSetPoint,0);   USART_Transmit_string("  ,  ",0) ;
	USART_Transmit_string(strSetPoint,0);   USART_Transmit_string("\r\n",0) ;*/
	
	receivedAngles[0] = atoi(strSetPoint) ;
	receivedAngles[1] = atoi(strSetPoint2) ;
	receivedAngles[2] = atoi(strSetPoint3) ;
	
	receivedSetPoints[0] = constrain(receivedAngles[0],0,90);
	receivedSetPoints[1] = constrain(receivedAngles[1],0,90);
	receivedSetPoints[2] = constrain(receivedAngles[2],0,90);
}

void cleanArr(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		*StringPtr=0x00 ;
		StringPtr++;
	}
}

ISR(USART0_RX_vect)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		UarTemp=UDR0 ;
		static uint8_t i=0;
		if(UarTemp == 10)
		{
			
			receiveAngles(buffer,setPoint);
			i=0 ;
			cleanArr(buffer);
		}
		else
		{
			buffer[i]=UarTemp ;
			i++ ;
		}
	}
}



ISR(ADC_vect)
{
	uint8_t LMB = ADCL;
	uint16_t ADCvalue = ADCH <<8 | LMB;

	switch(ADMUX)
	{	
		case 0b01000000:
		ADMUX = 0b01000001;
		break;
			
		case 0b01000001:
		ADC_accu[0] +=ADCvalue ;
		counter[0]++ ;
		//USART_Transmit_int(ADCvalue,0)    ; USART_Transmit_string("   ,  ",0) ;
		ADMUX = 0b01000010;
		break;
		
		case 0b01000010:
		ADC_accu[1] +=ADCvalue ;
		counter[1]++ ;
		//USART_Transmit_int(ADCvalue,0)    ; USART_Transmit_string("   ,  ",0) ;
		ADMUX = 0b01000011;
		break;
		
		case 0b01000011:
		ADC_accu[2] +=ADCvalue ;
		counter[2]++ ;
		//USART_Transmit_int(ADCvalue,0)    ; USART_Transmit_string("\r\n",0) ;
		ADMUX = 0b01000001;
		break;
	}
	
	if (counter[0]>= 100)
	{
		ADC_accu[0] = (ADC_accu[0])/(counter[0]) ;
		LPfilter(ADC_accu[0],filter,0.65) ;
		Angle[0] = calc_Angle(filter[0]) ;
		
		ADC_accu[1] = (ADC_accu[1])/(counter[1]) ;
		LPfilter(ADC_accu[1],filter2,0.65) ;
		Angle[1] = calc_Angle(filter2[0]) ;
		
		ADC_accu[2] = (ADC_accu[2])/(counter[2]) ;
		LPfilter(ADC_accu[2],filter3,0.65) ;
		Angle[2] = calc_Angle(filter3[0]) ;
		
		/*USART_Transmit_int(setPoint[0],0);   USART_Transmit_string("   ,  ",0) ;
		USART_Transmit_int(setPoint[1],0);	 USART_Transmit_string("  , ",0) ;
		USART_Transmit_int(setPoint[2],0);   USART_Transmit_string("\r\n",0) ;*/
		
		/*USART_Transmit_int(filter[0],0)   ; USART_Transmit_string(" , ",0) ;
		USART_Transmit_int(filter2[0],0)  ; USART_Transmit_string(" , ",0) ;
		USART_Transmit_int(filter3[0],0)  ; USART_Transmit_string("----",0) ;*/
		
		USART_Transmit_int(Angle[0],0)    ; USART_Transmit_string(" , ",0) ;
		USART_Transmit_int(Angle[1],0)    ; USART_Transmit_string(" , ",0) ;
		//USART_Transmit_int(Angle[2],0)    ; USART_Transmit_string("----",0) ;
		USART_Transmit_int(Angle[2],0)    ; USART_Transmit_string(" \r\n ",0) ;
		
		/*USART_Transmit_int(PWMDuty[0],0)  ; USART_Transmit_string(" , ",0) ;
		USART_Transmit_int(PWMDuty[1],0)  ; USART_Transmit_string(" , ",0) ;
		USART_Transmit_int(PWMDuty[2],0)  ; USART_Transmit_string("\r\n",0) ;*/
		
		for (uint8_t i=0 ;i<3;i++)
		{
			ADC_accu[i] = 0;
			counter[i] = 0 ;
		}
		
	}
	ADC_startConversion();
}