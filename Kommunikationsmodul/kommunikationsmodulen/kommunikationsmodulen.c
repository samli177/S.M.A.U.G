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
		USART_SendSensors();
		
		// TODO: put this on timer
		//if(TWI_send_status(ST_ADRESS))
			//PORTA ^= (1<<PORTA1);
			
		_delay_ms(50);
	}
}

// --  END MAIN --


void init()
{
	DDRA |= (1<<PORTA0|1<<PORTA1); //set status diodes to outputs
	USART_init();
	TWI_init(C_ADDRESS);
	init_counters();
}

//Interrupt vectors

ISR(TIMER1_COMPA_vect)
{
	decode_message_TwiFIFO();
	TCNT1 = 0;
}

ISR(TIMER3_COMPA_vect)
{
	TCNT3 = 0;
}