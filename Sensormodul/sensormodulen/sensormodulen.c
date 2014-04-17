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

char display_buffer[64][20];
int buffer_size = 0;


int main(void)
{
	display_init();
	sensors_init();
	
	// init TWI
	TWI_init(S_ADDRESS);
	init_counters();
	
	// Activate interrupts	
	sei();
	
	set_counter_1(2000);
	
	//display_text("Hello");
	
	while(1)
	{
		if(sensors_sampling_done())
		{
			sensors_reset_flag();
			sensors_display_data();
		}
		_delay_ms(100);
		
		//send_sensors(gSensorBuffer, 0);
		//TWI_send_autonom_settings(C_ADDRESS, 5);
	}
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	sensors_start_sample();
	TCNT1 = 0;
}

ISR(TIMER2_COMPA_vect)
{
	decode_message_TwiFIFO();
	TCNT2 = 0;
}

//---------------------------------------------------------------------------------------