/*
 * kommunikationsmodulen.c
 *
 * Created: 4/3/2014 10:12:45 AM
 *  Author: samli177
 */ 


#define F_CPU 18432000
#define BaudRate 115200

#define POLY 0x8408

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "twi.h"
#include "fifo.h"
#include "usart.h"


// -- Global variables from TWI --
int my_adress;
bool instruction;
int current_instruction;


// -- Declarations --
void init();



// -- MAIN --

int main(void)
{
	init();
	USART_init();
	
	// init TWI
	my_adress = C_ADRESS;
	init_TWI(my_adress);
	
	sei();
	_delay_ms(500);
	while(1)
	{
		PORTA ^= (1<<PORTA0);
		
		
		USART_DecodeRxFIFO();
		USART_SendSensors();
		
		_delay_ms(2000);
		send_status(S_ADRESS);
	}
}

// --  END MAIN --


void init()
{
	DDRA |= (1<<PORTA0|1<<PORTA1); //set status diodes to outputs
	
}



// -- Interrupts -- 



ISR(TWI_vect)
{
	cli();
	if(CONTROL == SLAW || CONTROL == ARBIT_SLAW)
	{
		instruction = true;
		
	}
	else if(CONTROL == DATA_SLAW)
	{
		if(instruction)
		{
			current_instruction = get_data();
			instruction = false;
		}
		else
		{
			switch(current_instruction)
			{
				case(I_SETTINGS):
				{
					get_settings_from_bus();
					break;
				}
				case(I_STRING):
				{
					get_char_from_bus();
					break;
				}
			}
		}
	}
	else if (CONTROL == DATA_GENERAL)
	{
		//temp
		PORTA |= (1<<PORTA1); // turn on/off led
		//temp
		
		get_sensor_from_bus();
	}
	else if (CONTROL == STOP)
	{
		PORTA ^= (1<<PORTA1);
		switch(current_instruction)
		{
			case(I_SETTINGS):
			{
				//get_settings();
				break;
			}
			case(I_STRING):
			{
				//get_char(1);
				break;
			}
		}
		stop_twi();
	}
	reset_TWI();
	sei();
}