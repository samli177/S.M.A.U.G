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
    while(1)
    {
		
		//navigation_set_autonomous_walk(TWI_get_autonom_settings());
		
		/*
		_delay_ms(500);
		//TWI_send_autonom_settings(C_ADRESS, 4);
        PORTA |= (1<<PORTA0);
		_delay_ms(1000);
		//TWI_send_string(0x40, "I AM DEAD!");
		PORTA &= ~(1<<PORTA0);
		_delay_ms(1000);
		USART_SendMessage("apa");
		TWI_send_string(S_ADRESS, "Hue");
		*/
		
		//USART_send_command_parameters(0,50,100);
		//_delay_ms(1000);
		
		if(navigation_autonomous_walk() == 1)
		{
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
// Redan definierade i navigation.c
//---------------------------------------------------------------------------------------

