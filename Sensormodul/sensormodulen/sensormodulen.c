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
#include <stdbool.h>
#include <string.h>

#include "display.h"
#include "twi.h"
#include "counter.h"
#include "sensors.h"	

void send_data(void);
void init_TWI_sensor(void);

uint8_t displayFlag = 0;

int main(void)
{	
	display_init();
	sensors_init();
	
	// init TWI
	TWI_init(S_ADDRESS);
	init_counters();
	
	set_counter_1(100);
	set_counter_2(3000);
	
	// Activate interrupts
	sei();

	display_text("Hello");
	
	while(1)
	{	
		_delay_ms(1);
		if(displayFlag)
		{
			if(decode_message_TwiFIFO())
			{
				sensors_display_data();
				set_counter_2(2000);
			} else {
				set_counter_2(3000);
			}
			displayFlag = 0;
			TCNT3 = 0;
		}
	}
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	sensors_start_sample();
	TCNT1 = 0;
}

ISR(TIMER3_COMPA_vect)
{
	displayFlag = 1;
	TCNT3 = 0;
}

//---------------------------------------------------------------------------------------