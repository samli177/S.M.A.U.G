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
#include "counter.h"
#include "LED.h"

// -- Declarations --
void init();



// -- MAIN --

int main(void)
{
	init();
	sei();
	
	_delay_ms(500);
	while(1)
	{
		//PORTA ^= (1<<PORTA0);
		
		USART_DecodeRxFIFO();
		if(TWI_sensor_flag())
		{
			USART_SendSensors();
		}
		decode_message_TwiFIFO();
		
		// TODO: put this on timer
		//if(TWI_send_status(ST_ADRESS))
			//LED1_TOGGLE;
		//TWI_send_string(S_ADDRESS, "Hejsan lulzi!");
	}
}

// --  END MAIN --


void init()
{
	LED_INIT;
	USART_init();
	TWI_init(C_ADDRESS);
	init_counters();
	set_counter_2(2000);
}

//Interrupt vectors

ISR(TIMER1_COMPA_vect)
{
	TCNT1 = 0;
}

ISR(TIMER3_COMPA_vect)
{
	TCNT3 = 0;
}