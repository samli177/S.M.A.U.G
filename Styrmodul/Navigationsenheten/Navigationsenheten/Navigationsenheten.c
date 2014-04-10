/*
 * Navigationsenheten.c
 *
 * Created: 4/9/2014 9:52:11 AM
 *  Author: perjo018
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include "twi.h"

//Global variables for TWI
bool instruction;
int current_instruction;
int my_adress;

int main(void)
{
	sei();
	my_adress = ST_ADRESS;
	init_TWI(my_adress);
	DDRA |= (1<<PORTA0 | 1<<PORTA1);
    while(1)
    {
		_delay_ms(500);
		//send_settings(C_ADRESS, 4);
        PORTA |= (1<<PORTA0);
		_delay_ms(1000);
		//send_string(0x40, "I AM DEAD!");
		PORTA &= ~(1<<PORTA0);
		_delay_ms(1000);
    }
}

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
				case(I_COMMAND):
				{
					get_command_from_bus();
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
		get_sensor_from_bus();
	}
	else if (CONTROL == STOP)
	{
		switch(current_instruction)
		{
			case(I_COMMAND):
			{
				PORTA ^= (1<<PORTA1);
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