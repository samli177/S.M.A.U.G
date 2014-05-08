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
	USART_set_twi_message_destination(C_ADDRESS); // send messages from gang to the display, not the computer
	sei();
	TWI_init(ST_ADDRESS);
	init_counters();
	
	//LED
	DDRA |= (1<<PORTA0 | 1<<PORTA1);
	DDRC |= (1<<PORTC6 | 1<<PORTC7);
	
	//Buttons
	DDRA &= ~(1<<PORTA6 | 1<<PORTA7); //For emphasize
	PCICR |= (1<<PCIE0); //Interrupt enable
	PCMSK0 |= (1<<PCINT6 | 1<<PCINT7); //mask for porta6 and porta7
	
	
	_delay_ms(5000);
	navigation_set_autonomous_walk(0);
	set_counter_1(100);
	
    while(1)
    {
		/*if(TWI_sensor_flag())
		{
			PORTA ^= (1<<PORTA1);
			navigation_fill_buffer();
		}*/
		
		
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
			if(TWI_command_flag())
			{
				PORTA ^= (1<<PORTA1);
				USART_SendCommand();
			}
			
			if(TWI_elevation_flag())
			{
				PORTA ^= (1<<PORTA0);
				USART_SendElevation();
			}
			
			/*if(USART_GyroFlag())
			{
				TWI_send_float(C_ADDRESS, USART_gyro_get_Y() * 180/PI);
			}*/
		}
		
		
		
		USART_DecodeRxFIFO();
    }
}

//---------------------------------------COUNTERS/TIMERS interrupt vectors-----------

ISR(TIMER1_COMPA_vect)
{
	//USART_RequestGyro();
	TCNT1 = 0;
}

ISR(TIMER3_COMPA_vect)
{
	//TWI_send_float(C_ADDRESS, (float)navigation_get_sensor(0));
	TCNT3 = 0;
}

//-------------------Buttons PinChange interrupt---------------------

ISR(PCINT0_vect)
{
	if(PINA & (1<<PINA6)) //Left walk
	{
		navigation_set_autonomous_walk(1);
		navigation_set_algorithm(1);
		//test
		PORTC ^= (1<<PORTC6);
	}
	else if(PINA & (1<<PINA7)) //Right walk
	{
		navigation_set_autonomous_walk(1);
		navigation_set_algorithm(0);
		//test
		PORTC ^= (1<<PORTC7);
	}	
}