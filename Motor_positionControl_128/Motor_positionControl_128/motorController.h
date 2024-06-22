/*
 * IncFile1.h
 *
 * Created: 2014.07.03. 9:43:48
 *  Author: F
 */ 
/************************************************************************/
/* 
 ATMEGA128				  L298 (motor drive)
|-------------|			  |-----------|
|		 PC3  |___________| DIR1	  |
|		 PC4  |___________| DIR2      |
|		 PC5  |___________|	DIR3	  |
|			  |			  |			  |
|       PE3   |___________| PWM1	  |
|       PE4   |___________| PWM2      |
|       PE5	  |___________|	PWM3      |
|	  		  |			  |	  		  |
|-------------|			  |-----------|
                                                     */
/************************************************************************/
#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

//#include "lowPassFilter.h"
#include <stdlib.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define Motor_controll_DDR	DDRC
#define Motor_controll_PORT	PORTC
#define Motor_controll_PIN	PINC


//#define STBY				PC4		//Standby
#define DIR1				PC2		//logic input Motor A
#define DIR2				PC3		//logic input Motor B
#define DIR3				PC4		//logic input Motor C

#define PWM1				PE3		//connected to ENA in L298
#define PWM2				PE4		//connected to ENB in L298
#define PWM3				PE5

#define PWMA_DDR			DDRE
#define PWMB_DDR			DDRE
#define PWMC_DDR			DDRE

#define CONSTRAIN			40

//#define PWM_LPF_GAIN		MOTOR_PWM_LPF_GAIN

void driveMotor(int motorA,int motorB,int motorC);
void init_MotorController(void);


void driveMotor(int motorA,int motorB,int motorC)
{
	if (motorA>0)
	{
		//positive direction
		Motor_controll_PORT |= 1<<DIR1;
		OCR3A = constrain(motorA,0,CONSTRAIN);
	}
	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value)
	else if (motorA<=0)
	{
		//negative direction
		Motor_controll_PORT &= ~(1<<DIR1);
		OCR3A = constrain(abs(motorA),0,CONSTRAIN);
 	}
	if (motorB>0)
	{	
		//positive direction	
		Motor_controll_PORT |= 1<<DIR2;
		OCR3B = constrain(motorB,0,CONSTRAIN);
	}
// 	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value) 
 	else if (motorB<=0)
	{
		//negative direction
		Motor_controll_PORT &= ~(1<<DIR2);
		OCR3B = constrain(abs(motorB),0,CONSTRAIN);
	}
	if (motorC>0)
	{
		//positive direction
		Motor_controll_PORT |= 1<<DIR3;
		OCR3C = constrain(motorC,0,CONSTRAIN);
	}
	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value)
	else if (motorC<=0)
	{
		//negative direction
		Motor_controll_PORT &= ~(1<<DIR3);
		OCR3C = constrain(abs(motorC),0,CONSTRAIN);
	}	
}

void init_MotorController(void)
{
	PWMA_DDR |= 1<<PWM1;
	PWMB_DDR |= 1<<PWM2;
	PWMC_DDR |= 1<<PWM3;
	Motor_controll_DDR |= 1<<DIR1 | 1<<DIR2 | 1<<DIR3 ; 
}


#endif /* INCFILE1_H_ */