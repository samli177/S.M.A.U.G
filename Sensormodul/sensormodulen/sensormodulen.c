/*
* Sensormodulen.c
*
* Created: 3/28/2014 9:43:46 AM
*  Author: perjo018
*/


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "display.h"

void send_data(void);
void init_TWI_sensor(void);

int main(void)
{
	init_display();
	init_TWI_sensor();
	sei();
	displaytest();
	while(1)
	{
		
	}
}

void init_mux()
{
	DDRA |= 0b00111110;
	DDRA &= !(1<<PORTA0);
	PORTA &= !(1<<PORTA5);
	PORTA &= 0b11100001;
}

void init_UL()
{
	DDRA |= (1<<PORTA7);
	DDRA &= !(1<<PORTA6);
	TCCR0B = 0x05;
}

void select_sensor(int sensor)
{
	PORTA &= 0b11100001;
	switch(sensor)
	{
		case(8):
		{
			break;
		}
		case(7):
		{
			PORTA |= 0b00000010;
			break;
		}
		case(6):
		{
			PORTA |= 0b00001110;
			break;
		}
		case(5):
		{
			PORTA |= 0b00001100;
			break;
		}
		case(4):
		{
			PORTA |= 0b00001010;
			break;
		}
		case(3):
		{
			PORTA |= 0b00001000;
			break;
		}
		case(2);
		{
			PORTA |= 0b00000110;
			break;
		}
		default:
		{
			PORTA |= 0b00000100;
			break;
		}
	}
}

int get_sensor()
{
	return (PORTA & (1<<PORTA0));
}

unsigned int UL-sensor()
{
	unsigned int i;
	PORTA |= (1<<PORTA7);
	_delay_us(15);
	PORTA &= !(1<<PORTA7);
	//while(!(PORTA & (1<<PORTA6)));
	TCNT0 = 0;
	while((PORTA & (1<<PORTA6)));
	i = TCNT0;
	return i;
}

void displaytest(void)
{
	print_line(0, "Initiating AI");
}

void init_TWI_sensor(void)
{
	TWBR = (1<<TWBR0); //bit rate
	TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE); //TWI enable, TWI interrupt enable
	//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
	TWAR = (1<<TWA3); // Address 001000, General Call Not Accepted	
}

void send_data(void) //Gör om så att all data skickas i rad med repeated START, lägg till säkerhet!
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//if ((TWSR & 0xF8) != 0x08)	
	TWDR = 0; //General Call
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//TWDR = Sensornr, Servoposition
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//TWDR = Sensorutslag
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

ISR(TWI_vect)
{
	if (TWSR == 0x60)
		//Dags att utföra ett svep (och sen skicka resultat kanske?)
	TWCR |= (1<<TWINT) | (1<<TWEA);
}