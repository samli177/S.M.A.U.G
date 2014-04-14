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
#include "usart.h"
#include "counter.h"


int main(void)
{
	USART_init();
	sei();
	TWI_init(ST_ADRESS);
	init_counters();
	DDRA |= (1<<PORTA0 | 1<<PORTA1);
    while(1)
    {
		_delay_ms(500);
		//TWI_send_autonom_settings(C_ADRESS, 4);
        PORTA |= (1<<PORTA0);
		_delay_ms(1000);
		//TWI_send_string(0x40, "I AM DEAD!");
		PORTA &= ~(1<<PORTA0);
		_delay_ms(1000);
		USART_SendMessage("apa");
		TWI_send_string(S_ADRESS, "Hue");
		if(TWI_command_flag())
			PORTA ^= (1<<PORTA0);
    }
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	TCNT1 = 0;
}

ISR(TIMER2_COMPA_vect)
{
	TCNT2 = 0;
}

//---------------------------------------------------------------------------------------