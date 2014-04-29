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
#include <stdlib.h>
#include <string.h>
#include "twi.h"
#include "usart.h"
#include "counter.h"
#include "Navigation.h"
#include "autonomouswalk.h"

int main(void)
{
	USART_init();
	sei();
	TWI_init(ST_ADDRESS);
	init_counters();
	DDRA |= (1<<PORTA0 | 1<<PORTA1);
	
	_delay_ms(5000);
	navigation_set_autonomous_walk(1);
    while(1)
    {
		if(TWI_sensor_flag())
		{
			navigation_fill_buffer();
		}
		if(TWI_autonom_settings_flag())
		{
			uint8_t sett = TWI_get_autonom_settings();
			if(sett == 0)
			{
				navigation_set_autonomous_walk(0);
			}
			else if(sett == 1)
			{
				navigation_set_autonomous_walk(1);
				navigation_set_algorithm(1);
			}
			else //sett == 2
			{
				navigation_set_autonomous_walk(1);
				navigation_set_algorithm(0);
			}
		}
		
		if(navigation_autonomous_walk() == 1)
		{
			if(TWI_control_settings_flag())
			{
				navigation_set_Kp(TWI_get_control_setting(0));
			}
			autonomouswalk_walk();
		}
		else
		{
			if(TWI_command_flag()){
				PORTA ^= (1<<PORTA1);
				USART_SendCommand();
			}
		}
		USART_DecodeRxFIFO();
    }
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	TCNT1 = 0;
	
	if(USART_ready())
	{
		PORTA ^= (1<<PORTA0);
	}
}

ISR(TIMER3_COMPA_vect)
{
	TCNT3 = 0;
}