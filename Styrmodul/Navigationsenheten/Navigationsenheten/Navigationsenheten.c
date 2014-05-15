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
#include "LED.h"

//Flag to know if to send the autonom settings to the computer.
uint8_t autonom_flag = 1;

int main(void)
{
	USART_init();
	USART_set_twi_message_destination(C_ADDRESS); //<<<????>>> send messages from gang to the display, not the computer
	sei();
	TWI_init(ST_ADDRESS);
	init_counters();
	
	LED_INIT;
	
	//Buttons
	DDRA &= ~(1<<PORTA6 | 1<<PORTA7); //For emphasize
	PCICR |= (1<<PCIE0); //Interrupt enable
	PCMSK0 |= (1<<PCINT6 | 1<<PCINT7); //mask for porta6 and porta7
	
	
	_delay_ms(5000);
	navigation_set_autonomous_walk(0);
	//set_counter_1(100);
	set_counter_2(200);
	
    while(1)
    {
		/*if(TWI_sensor_flag())
		{
			LED1_TOGGLE;
			navigation_fill_buffer();
		}*/
		
		
		if(autonom_flag)
		{
			TWI_send_autonom_settings(C_ADDRESS, navigation_left_algorithm());
			autonom_flag = 0;
		}
		
		if(TWI_status_settings_flag())
		{
			autonomouswalk_set_return_status(TWI_get_status_settings());
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
				LED1_TOGGLE;
				USART_send_command();
			}
			
			if(TWI_elevation_flag())
			{
				LED0_TOGGLE;
				USART_send_elevation();
			}
		USART_decode_rx_fifo();
	    }
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
		autonom_flag = 1;
	}
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
		LED2_TOGGLE;
	}
	else if(PINA & (1<<PINA7)) //Right walk
	{
		navigation_set_autonomous_walk(1);
		navigation_set_algorithm(0);
		//test
		LED3_TOGGLE;
	}
	autonom_flag = 1;
}
