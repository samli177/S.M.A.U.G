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

int sensorBufferSize = 3;
uint8_t sensorBuffer[8][3]; // sensorBufferSize = 3!!
uint8_t currentBufferLine;

int compare (const void * a, const void * b)
{
	return ( *(uint8_t*)a - *(uint8_t*)b );
}

uint8_t get_sensor(int sensorNr)
{
	uint8_t temp[sensorBufferSize];
	for(int i = 0; i < sensorBufferSize; ++i)
	{
		temp[i] = sensorBuffer[sensorNr][i];
	}
	qsort(temp, sensorBufferSize, sizeof(uint8_t), compare);
	return temp[sensorBufferSize / 2];
}

void fill_buffer()
{
	for(int i = 0; i < 8; ++i)
	{
		sensorBuffer[i][currentBufferLine] = TWI_get_sensor(i);
	}
	if(currentBufferLine == sensorBufferSize - 1)
		currentBufferLine = 0;
	else
		currentBufferLine += 1;
}

int main(void)
{
	USART_init();
	sei();
	TWI_init(ST_ADDRESS);
	init_counters();
	DDRA |= (1<<PORTA0 | 1<<PORTA1);
    while(1)
    {
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
		
		if(TWI_command_flag()){
			PORTA ^= (1<<PORTA1);
			USART_SendCommand();
		}
			
    }
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	if(TWI_sensor_flag())
		fill_buffer();
	TCNT1 = 0;
}

ISR(TIMER2_COMPA_vect)
{
	TCNT2 = 0;
}

//---------------------------------------------------------------------------------------