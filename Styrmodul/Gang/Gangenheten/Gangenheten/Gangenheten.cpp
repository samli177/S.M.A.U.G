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
#include "MpuInit.h"
#include "LED.h"

uint8_t std_pos_flag = 1;
uint8_t move_to_std_flag = 0;

/**
 * \brief 
 * Waits 20 s, until the gyro is stable.
 * 
 * \return void
 */
void wait_until_gyro_stable();

int main(void)
{
	LED_INIT;
	//servoTx;
	sei();
	USART_init();
	MPU_init();
	SERVO_init();
	init_counters();
	
	initvar();
	
	SERVO_update_EEPROM(BROADCASTING_ID);
	
	wait(10);
	move_to_std();
	wait_until_gyro_stable();
	LED0_ON;
	USART_send_message("Gyro Stable");
	
	// ------ TESTCODE FOR READING SERVO -------
	
	//servoGoto(1, 3.14/3, 0x200);
	SERVO_update_EEPROM(BROADCASTING_ID); // NOTE: needs to run once for SERVO_get position to work	
	//----------------------------
	
	reset_counter_1();
	set_counter_1(3000);
	
    while(1)
    {
		MPU_update();
		
		if(USART_get_turn_flag())
		{
			turn_degrees(USART_get_turn_angle(), USART_get_turn_dir());
		}
		if(USART_get_climb_flag())
		{
			climb();
		}
		
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
			wait(50);
			cli();
			USART_send_ready();
			sei();
		}
		
		
		
		if(move_to_std_flag == 1)
		{
			move_to_std_flag = 0;
			move_to_std();
		}
		
		
		/*climb();
		for(int i = 0; i < 10; ++i)
		{
			move_robot(0,50,100);
			//wait(2000);
		}
		*/
		/*
		
		change_z(-120);
		move_to_std();
		turn_degrees(180,1);
		
		
		// Takes a predecided number of steps forward
		// This is good when testing different things.
		wait(100);
		for(int i = 0; i < 10; ++i)
		{
			move_robot(0,50,100);
			//wait(2000);
		}
		*/
		
	USART_decode_rx_fifo();

	}
}

ISR(TIMER1_COMPA_vect)
{
	if(std_pos_flag == 0)
	{
		std_pos_flag = 1;
		move_to_std_flag = 1;
	}
	
	//USART_SendValue(MPU_get_y() * 180/M_PI);
	TCNT1 = 0;
}

void wait_until_gyro_stable()
{
	wait(4000);
	wait(4000);
	wait(4000);
	wait(4000);
	wait(4000);
	/*MPU_update();
	float value1 = MPU_get_y();
	float value2 = 100;
	do
	{
		wait(100);
		value2 = value1;
		value1 = MPU_get_y();
		//USART_SendValue(fabs(value1 - value2));
	} while (fabs(value1 - value2) > 0.001);*/
}