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
	DDRD |= (1<<PORTD5); //init LED
	//servoTx;
	
	servoRetrunLevel(BROADCASTING_ID, 1); //turns off return packets
	
	
	initServoSerial(); //Init servos
	USART_init();
	init_counters();
	set_counter_1(10000);
	initvar();
	sei();
	
	move_to_std();
	move_to_std();

	
	_delay_ms(5000);
	
	reset_counter_1();
	set_counter_1(3000);
	
    while(1)
    {
		
		uint8_t r = USART_getRotation();
		uint8_t s = USART_getSpeed();
		uint8_t d = USART_getDirection();
		if(s != 0 || r != 50)
		{
			std_pos_flag = 0;
			reset_counter_1();
		}
		
		move_robot(d, r, s);

		
		
		/*
		for(int i = 0; i < 5; ++i)
		{
			move_robot(0,50,100);
			leg1.lift = -leg1.lift;
			leg2.lift = -leg2.lift;
			leg3.lift = -leg3.lift;
			leg4.lift = -leg4.lift;
			leg5.lift = -leg5.lift;
			leg6.lift = -leg6.lift;
		}
		move_to_std();
		move_to_std();

		_delay_ms(5000);
		*/
		
		USART_DecodeRxFIFO();
    }
}

ISR(TIMER1_COMPA_vect)
{
	if(std_pos_flag == 0)
	{
		std_pos_flag = 1;
		move_to_std();
		move_to_std();
	}
	TCNT1 = 0;
}