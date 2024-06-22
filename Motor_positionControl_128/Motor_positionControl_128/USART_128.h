/*
 * USART_128.h
 *
 * Created: 6/18/2018 3:49:36 PM
 *  Author: moham
 */ 


#ifndef USART_128_H_
#define USART_128_H_


#include <avr/io.h>
#include <stdlib.h>
/*Formula 151 Page of the datasheet*/

#define UART_BAUD_0		115200
#define UART_BAUD_1		115200

#define FOSC 8000000// Clock Speed
#define BAUD 28800
#define MYUBRR FOSC/16/BAUD-1

void USART_init(uint8_t portChannel);
void USART_Transmit(unsigned char data,uint8_t portChannel);
void USART_Transmit_float(float USART_data,uint8_t portChannel);
unsigned char USART_Receive(uint8_t portChannel);
void USART_Transmit_string(char* StringPtr,uint8_t portChannel);

void USART_init(uint8_t portChannel)
{
	unsigned int ubrr = MYUBRR ;
	if (portChannel==0)
	{
		/* Set baud rate */
		UBRR0H = (unsigned char)(ubrr>>8);
		UBRR0L = (unsigned char)ubrr;
		/* Enable receiver and transmitter */
		UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE);// enable receive interrupt;
		/* Set frame format: 8data, 2stop bit */
		UCSR0C |=(3<<UCSZ00);
	}
	else
	{
		/* Set baud rate */
		UBRR1H = (unsigned char)(ubrr>>8);
		UBRR1L = (unsigned char)ubrr;
		/* Enable receiver and transmitter */
		UCSR1B = (1<<RXEN1)|(1<<TXEN1);
		/* Set frame format: 8data, 2stop bit */
		UCSR1C |=(3<<UCSZ10);
	}
}

void USART_Transmit( unsigned char data,uint8_t portChannel )	//155. Datasheet
{
	if (portChannel==0)
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) )
		;
		/* Put data into buffer, sends the data */
		UDR0 = data;
	}
	else
	{
		/* Wait for empty transmit buffer */
		while ( !( UCSR1A & (1<<UDRE1)) )
		;
		/* Put data into buffer, sends the data */
		UDR1 = data;
	}
}

unsigned char USART_Receive( uint8_t portChannel )			//258. Datasheet
{
	if (portChannel==0)
	{
		/* Wait for data to be received */
		while ( !(UCSR0A & (1<<RXC0)) )
		;
		/* Get and return received data from buffer */
		return UDR0;
	}
	else
	{
		/* Wait for data to be received */
		while ( !(UCSR1A & (1<<RXC1)) )
		;
		/* Get and return received data from buffer */
		return UDR1;
	}
	
}

void USART_Transmit_float(float USART_data,uint8_t portChannel)	//End line 0x0A
{
	uint8_t i=0;
	char CSV_data[10];
	
	dtostrf(USART_data,7,2,CSV_data);
	while (CSV_data[i]!=0)
	{
		USART_Transmit(CSV_data[i],portChannel);
		i++;
	}
}

void USART_Transmit_int(int int_to_string,uint8_t portChannel)	//End line 0x0A
{
	uint8_t Ns=10;
	uint8_t i=0;
	char string_to_USART[10];
	
	itoa(int_to_string,string_to_USART,Ns);
	while (string_to_USART[i]!=0)
	{
		USART_Transmit(string_to_USART[i], portChannel);
		i++;
	}
}

void USART_Transmit_string(char* StringPtr,uint8_t portChannel)
{
	while(*StringPtr != 0x00)
	{
		USART_Transmit(*StringPtr,portChannel);
		StringPtr++;
	}
}




#endif /* USART_128_H_ */