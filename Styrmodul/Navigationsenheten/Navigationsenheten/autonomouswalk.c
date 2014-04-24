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

#define MAX_ROTATION_CLOCKWISE 10
#define MAX_ROTATION_COUNTER_CLOCKWISE 90
#define MAX_ROTATION_RADIANS 3.1415/6
#define STEPPING_TIME 200

//Variable for the speed parameter in movement commands. 
static uint8_t gSpeed = 80;

//Variable to decide if status messages are to be
//sent back to the PC.
static uint8_t gStatus = 0;

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
	walk_forward();
	_delay_ms(STEPPING_TIME);
	while(navigation_get_sensor(4) < CORRIDOR_WIDTH && navigation_get_sensor(5) > (CORRIDOR_WIDTH/2))
	{
		if(gStatus)
		{
			TWI_send_string(C_ADDRESS, "Rotating left.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_COUNTER_CLOCKWISE, gSpeed);
		_delay_ms(STEPPING_TIME);
	}
	for(int i = 0; i < 3; ++i)
	{
		walk_forward();
	}
}

void turn_right()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting turning right.");
	}
	walk_forward();
	_delay_ms(STEPPING_TIME);
	while(navigation_get_sensor(4) < CORRIDOR_WIDTH && navigation_get_sensor(5) > (CORRIDOR_WIDTH/2))
	{
		if(gStatus)
		{
			TWI_send_string(C_ADDRESS, "Rotating right.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_CLOCKWISE, gSpeed);
		_delay_ms(STEPPING_TIME);
	}
	for(int i = 0; i < 3; ++i)
	{
		walk_forward();
	}
}

void turn_around()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting to turn around.");
	}
	while (navigation_get_sensor(4) < CORRIDOR_WIDTH)
	{
		if(gStatus)
		{
			TWI_send_string(C_ADDRESS, "Turning around.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_COUNTER_CLOCKWISE, gSpeed);
		_delay_ms(STEPPING_TIME);
	}
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Corridor ahead, done turning around.");
	}
}

void walk_forward()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Finding regulation parameters.");
	}
	float angleOffset = navigation_angle_offset();
	float directionCompensationAngle = navigation_direction_regulation(angleOffset);
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Found regulation parameters.");
	}
	int adjustmentRotation = (50 + 50 * angleOffset/MAX_ROTATION_RADIANS);
	if (adjustmentRotation >= 100)
	{
		adjustmentRotation = 100;
	}
	else if(adjustmentRotation <= 0)
	{
		adjustmentRotation = 0;
	}
	int adjustmentDirection = 90 * directionCompensationAngle/(2*PI);
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Taking a step.");
	}
	USART_send_command_parameters((uint8_t)adjustmentDirection, (uint8_t)adjustmentRotation, gSpeed);
	_delay_ms(STEPPING_TIME);
}

void autonomouswalk_walk()
{
	if(navigation_left_algorithm())
	{
		if(navigation_check_left_turn() == 2)
		{
			turn_left();
		}
		else if(navigation_get_sensor(4) > CORRIDOR_WIDTH)
		{
			walk_forward();
		}
		else if(navigation_check_right_turn() == 2)
		{
			turn_right();
		}
		else
		{
			turn_around();
		}
	}
	else
	{
		if(navigation_check_right_turn() == 2)
		{
			turn_left();
		}
		else if(navigation_get_sensor(4) > CORRIDOR_WIDTH)
		{
			walk_forward();
		}
		else if(navigation_check_left_turn() == 2)
		{
			turn_right();
		}
		else
		{
			turn_around();
		}
	}
}