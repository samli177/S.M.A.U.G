/*
 * autonomouswalk.c
 *
 * Created: 2014-04-18 14:39:34
 *  Author: Jonas
 */ 

#define F_CPU 16000000UL

#include "autonomouswalk.h"
#include "Navigation.h"
#include "usart.h"
#include "twi.h"
#include <avr/io.h>
#include <util/delay.h>

#define MAX_ROTATION_CLOCKWISE 70
#define MAX_ROTATION_COUNTER_CLOCKWISE 30
#define MAX_ROTATION_RADIANS 0.52
#define STEPPING_TIME 400
#define TURN_EXIT_ITTERATIONS 0
#define TURN_ENTRY_ITTERATIONS_RIGHT 0
#define TURN_ENTRY_ITTERATIONS_LEFT 0
#define CLIMB_LENGTH_LIMIT 35
#define ANGLE_SCALE_FACTOR 0.3
//Variable for the speed parameter in movement commands. 
uint8_t gSpeed = 90;

//Variable to decide if status messages are to be
//sent back to the PC.
uint8_t gStatus = 0;

//A counter to keep track of how many times the robot have
//found itself unable to make a decision.
uint8_t decisionCounter = 0;

//Flags to make sure turning doesn't repeat
uint8_t may_turn_left = 1;
uint8_t may_turn_right = 1;

void autonomouswalk_set_speed(uint8_t speed)
{
	gSpeed=speed;
}

uint8_t autonomouswalk_get_speed()
{
	return gSpeed;
}

void autonomouswalk_set_return_status(uint8_t status)
{
	gStatus = status;
}

uint8_t autonomouswalk_get_return_status()
{
	return gStatus;
}

void turn_left()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting turning left.");
	}
	
	USART_send_turn(90, 0);
	while(USART_turn_done() == 0)
	{
		USART_decode_rx_fifo();
		_delay_ms(10);
	}
	
	for(int i = 0; (i < TURN_EXIT_ITTERATIONS && navigation_autonomous_walk() != 0); ++i)
	{
		walk_forward();
	}
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Done turning left.");
	}
}

void turn_right()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting turning right.");
	}
	
	USART_send_turn(90, 1);
	while(USART_turn_done() == 0)
	{
		USART_decode_rx_fifo();
		_delay_ms(10);
	}
	
	for(int i = 0; (i < TURN_EXIT_ITTERATIONS && navigation_autonomous_walk() != 0); ++i)
	{
		walk_forward();
	}
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Done turning right.");
	}
}

void turn_around()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting to turn around.");
	}
	
	USART_send_turn(180, 0);
	while(USART_turn_done() == 0)
	{
		USART_decode_rx_fifo();
		_delay_ms(10);
	}
	
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Corridor ahead, done turning around.");
	}
}

void walk_forward()
{
	float angleOffset = navigation_angle_offset();
	float directionCompensationAngle = navigation_direction_regulation(angleOffset);
	int adjustmentRotation = (50 + 50 * angleOffset * ANGLE_SCALE_FACTOR);
	if (adjustmentRotation >= 100)
	{
		adjustmentRotation = 100;
	}
	else if(adjustmentRotation <= 0)
	{
		adjustmentRotation = 0;
	}
	int adjustmentDirection = 90 * directionCompensationAngle/(2*PI);
	USART_send_command_parameters((uint8_t)adjustmentDirection, (uint8_t)adjustmentRotation, gSpeed);
	navigation_stepping_delay();
}

void climb()
{
	USART_send_climb();	
}

void autonomouswalk_walk()
{
	navigation_low_pass_obstacle();
	if(navigation_left_algorithm())
	{
		if(navigation_check_left_turn() == 2)
		{
			if(may_turn_left)
			{
				for(int i = 0;i < TURN_ENTRY_ITTERATIONS_LEFT; ++i)
				{
					walk_forward();
				}
				
				turn_left();
				may_turn_left = 0;
				may_turn_right = 1;
			}
			else
			{
				walk_forward();
				may_turn_left = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_get_sensor(4) > 60)
		{
			if(navigation_get_sensor(6) < CLIMB_LENGTH_LIMIT)
			{
				/*while(navigation_get_sensor(0) - navigation_get_sensor(2) != 0)
				{
					float angleOffset = navigation_angle_offset();
					uint8_t adjustmentRotation = (50 + 50 * angleOffset * 2.0/PI);
					
					USART_send_command_parameters(0, adjustmentRotation, 0);
				}*/
				
				if(gStatus)
				{
					TWI_send_string_fixed_length(C_ADDRESS, "Starting climb", 14);
				}
				
				//Denna del behövs inte om roboten står rakt vid hindret
				float tempAngle = navigation_angle_offset();
				if (tempAngle != 0)
				{
					if(tempAngle>0)
					{
						USART_send_turn((uint16_t)tempAngle*180/M_PI,0);
					}
					else if (tempAngle < 0)
					{
						USART_send_turn((uint16_t)-tempAngle*180/M_PI,1);
					}
					while(USART_turn_done() == 0)
					{
						USART_decode_rx_fifo();
						_delay_ms(10);
					}			
				}
				
				/*
				// FUL HÅRDKODNING FÖR ATT KOMPENSERA FÖR DÅLIG REGLERING! TA BORT OM ROBOTEN ALLTID KOMMER RAKT TILL HINDRET
				navigation_stepping_delay();
				USART_send_command_parameters(22+45*navigation_left_algorithm(),50,50);
				*/
				
				climb();
				while(USART_climb_done() == 0)
				{
					USART_decode_rx_fifo();
					_delay_ms(10);
				}
				
				if(gStatus)
				{
					TWI_send_string_fixed_length(C_ADDRESS, "Climb done", 10);
				}
			}
			else
			{
				walk_forward();
				may_turn_left = 1;
				may_turn_right = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_check_right_turn() == 2)
		{
			if(may_turn_right)
			{
				turn_right();
				may_turn_right = 0;
				may_turn_left = 1;
			}
			else
			{
				walk_forward();
				may_turn_right = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_check_left_turn() == 0 && navigation_check_right_turn() == 0 && navigation_get_sensor(4) < 40)
		{
			turn_around();
			decisionCounter = 0;
		}
		else if(decisionCounter < 10)
		{
			walk_forward();
			++decisionCounter;
		}
		else
		{
			decisionCounter = 0;
			navigation_set_autonomous_walk(0);
			TWI_send_string_fixed_length(C_ADDRESS, "ERROR: Can't make a decision, turning off autonomous mode", 57);
		}
	}
	else
	{
		if(navigation_check_right_turn() == 2)
		{
			if(may_turn_right)			
			{
				for(int i = 0;i < TURN_ENTRY_ITTERATIONS_RIGHT; ++i)
				{
					walk_forward();
				}
				turn_right();
				may_turn_right = 0;
				may_turn_left = 1;
			}
			else
			{
				walk_forward();
				may_turn_right = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_get_sensor(4) > 60)
		{
			if(navigation_get_sensor(6) < CLIMB_LENGTH_LIMIT)
			{
				if(gStatus)
				{
					TWI_send_string_fixed_length(C_ADDRESS, "Starting climb", 14);
				}
				
				climb();
				while(USART_climb_done() == 0)
				{
					USART_decode_rx_fifo();
					_delay_ms(10);
				}
				
				if(gStatus)
				{
					TWI_send_string_fixed_length(C_ADDRESS, "Climb done", 10);
				}
			}
			else
			{
				walk_forward();
				may_turn_left = 1;
				may_turn_right = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_check_left_turn() == 2)
		{
			if(may_turn_left)
			{
				turn_left();
				may_turn_left = 0;
				may_turn_right = 1;
			}
			else
			{
				walk_forward();
				may_turn_left = 1;
			}
			decisionCounter = 0;
		}
		else if(navigation_check_left_turn() == 0 && navigation_check_right_turn() == 0  && navigation_get_sensor(4) < 40)
		{
			turn_around();
			decisionCounter = 0;
		}
		else if(decisionCounter < 10)
		{
			walk_forward();
			++decisionCounter;
		}
		else
		{
			decisionCounter = 0;
			navigation_set_autonomous_walk(0);
			TWI_send_string_fixed_length(C_ADDRESS, "ERROR: Can't make a decision, turning off autonomous mode", 57);
		}
	}
}