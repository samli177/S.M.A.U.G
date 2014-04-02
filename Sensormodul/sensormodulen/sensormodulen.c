/*
* Sensormodulen.c
*
* Created: 3/28/2014 9:43:46 AM
*  Author: perjo018
*/


#define F_CPU 18300000UL

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

void displaytest(void)
{
	print_text("Test display");
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