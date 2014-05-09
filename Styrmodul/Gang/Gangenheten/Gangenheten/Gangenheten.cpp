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

#define LED1_ON PORTC |= (1<<PORTC6)
#define LED1_OFF PORTC &= ~(1<<PORTC6)
#define LED1_TOGGLE PORTC ^= (1<<PORTC6)

#define LED2_ON PORTC |= (1<<PORTC7)
#define LED2_OFF PORTC &= ~(1<<PORTC7)
#define LED2_TOGGLE PORTC ^= (1<<PORTC7)

uint8_t std_pos_flag = 1;
uint8_t move_to_std_flag = 0;

void wait_until_gyro_stable();

int main(void)
{
	DDRC |= (1<<PORTC6 | 1<<PORTC7); //init LED
	//servoTx;
	sei();
	USART_init();
	MPU_init();
	SERVO_init(); //Init servos
	init_counters();
	
	initvar();
	
	SERVO_update_EEPROM(BROADCASTING_ID);
	
	wait(10);
	move_to_std();
	wait_until_gyro_stable();
	USART_SendMessage("Gyro Stable");
	
	// ------ TESTCODE FOR READING SERVO -------
		
	//servoGoto(1, 3.14/3, 0x200);
	SERVO_update_EEPROM(BROADCASTING_ID); // NOTE: needs to run once for SERVO_get position to work	
	//----------------------------
	
	wait(5000);
	
	reset_counter_1();
	set_counter_1(3000);
	
	//climb();
	
	while(1)
	{
		turn_degrees(90, -1);
		wait(3000);
	}
	
	wait(2000);
	
	
	
    while(1)
    {
		MPU_update();
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
			wait(50);
			PORTD ^= (1<<PORTD5);
			cli();
			USART_send_ready();
			sei();
		}
		*/
		
		if(move_to_std_flag == 1)
		{
			move_to_std_flag = 0;
			move_to_std();
		}
		
		for(int i = 0; i < 5; ++i)
		{
			move_robot(0,50,100);
			wait(2000);
			
		}
		move_to_std();

		wait(5000);
		
		
		
		USART_DecodeRxFIFO();
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
	_delay_ms(20000);
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