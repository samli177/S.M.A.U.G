/*
 * gang.c
 *
 * Created: 28/7/2014 
 *  Author: tobli914
 */ 

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

int main(void)
{
	DDRD |= (1<<PORTD5); //init LED
	//servoTx;
	
	servoRetrunLevel(BROADCASTING_ID, 1); //turns off return packets
	
	sei();
	initServoSerial(); //Init servos
	USART_init();
	init_counters();
	set_counter_1(10000);
	initvar();
	
	move_to_std();
	move_to_std();
	/*
	moveLeg1too(x0_1, y0_1, z0, speed);
	moveLeg2too(x0_2, y0_2, z0, speed);
	moveLeg3too(x0_3, y0_3, z0, speed);
	moveLeg4too(x0_4, y0_4, z0, speed);
	moveLeg5too(x0_5, y0_5, z0, speed);
	moveLeg6too(x0_6, y0_6, z0, speed);
	*/
	
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
		leg1.lift = -leg1.lift;
		leg2.lift = -leg2.lift;
		leg3.lift = -leg3.lift;
		leg4.lift = -leg4.lift;
		leg5.lift = -leg5.lift;
		leg6.lift = -leg6.lift;
		
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

		_delay_ms(5000);
		*/
		
		USART_DecodeRxFIFO();
    }
}