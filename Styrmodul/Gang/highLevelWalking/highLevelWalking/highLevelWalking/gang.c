#define F_CPU 16000000

#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serialServoControl.h"
#include "inverseKinematics.h"
#include "highLevelWalking.h"
#include "usart.h"
#include "counter.h"

uint8_t std_pos_flag = 1;

int main(void)
{
	DDRC |= (1<<PORTC6) | (1<<PORTC7); //init LEDs
	//servoTx;

	
	
	SERVO_init(); //Init servos
	USART_init();
	init_counters();
	set_counter_1(10000);
	initvar();
	sei();
	
	SERVO_update_EEPROM(BROADCASTING_ID);
	
	//move_to_std();

	// ------ TESTCODE FOR READING SERVO -------
		
	//servoGoto(1, 3.14/3, 0x200);
	SERVO_update_EEPROM(BROADCASTING_ID); // NOTE: needs to run once for SERVO_get position to work	
	//----------------------------
	
	_delay_ms(3000);
	
	reset_counter_1();
	set_counter_1(3000);
	
    while(1)
    {
		/*
		uint8_t r = USART_getRotation();
		uint8_t s = USART_getSpeed();
		uint8_t d = USART_getDirection();
		if(s != 0 || r != 50)
		{
			std_pos_flag = 0;
			reset_counter_1();
		}
		
		move_robot(d, r, s);
		
		
		if(r == 50 && s == 0 && d == 0)
		{
			_delay_ms(50);
			//PORTD ^= (1<<PORTD5);
			cli();
			USART_send_ready();
			sei();
		}
		*/

		
/*change_z(-130);
move_to_std();
		for(int i = 0; i < 5; ++i)
		{
			move_robot(0,50,100);
			//_delay_ms(2000);
			
		}
		_delay_ms(1000);
		move_to_std();
	
		_delay_ms(1000);
		
		
		*/
		
		climb();
		//climb_all_one_leg();
		
		//SERVO_update_data(12);
		//USART_SendValue(SERVO_get_load());
		USART_DecodeRxFIFO();
		_delay_ms(200);
	
    }
}

ISR(TIMER1_COMPA_vect)
{
	if(std_pos_flag == 0)
	{
		std_pos_flag = 1;
		move_to_std();
	}
	TCNT1 = 0;
}