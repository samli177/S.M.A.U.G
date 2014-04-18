/*
 * autonomouswalk.c
 *
 * Created: 2014-04-18 14:39:34
 *  Author: Jonas
 */ 

#include "autonomouswalk.h"
#include "Navigation.h"
#include "usart.h"
#include "twi.h"
#include <avr/io.h>


//Variable for the speed parameter in movement commands. 
uint8_t gSpeed = 80;

//Variable to decide if status messages are to be
//sent back to the PC.
uint8_t gStatus = 1;

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
}

void turn_right()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting turning right.");
	}
}

void turn_around(uint8_t frontSensor)
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting to turn around.");
	}
	while (frontSensor < CORRIDOR_WIDTH)
	{
		if(gStatus)
		{
			TWI_send_string(C_ADDRESS, "Turning around.");
		}
		USART_send_command_parameters(0, MAX_ROTATION, gSpeed);
	}
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Corridor ahead, done turning around.");
	}
}

void walk_forward(uint8_t sensors[5])
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Finding regulation parameters.");
	}
	float angleOffset = navigation_angle_offset(sensors);
	float directionCompensationAngle = navigation_direction_regulation(sensors, angleOffset);
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Found regulation parameters.");
	}
	int adjustmentRotation = (50 + 50 * angleOffset/MAX_ROTATION_RADIANS);
	if (adjustmentRotation >= 100)
	{
		adjustmentRotation = 100;
	}
	else if(adjustmentRotation<=0)
	{
		adjustmentRotation = 0;
	}
	int adjustmentDirection = 90* directionCompensationAngle/(2*PI);
	if(adjustmentDirection < 0)
	{
		adjustmentDirection = 90 + adjustmentDirection;
	}
	USART_send_command_parameters((uint8_t)adjustmentDirection, (uint8_t)adjustmentRotation, gSpeed);
}

void autonomouswalk_walk(uint8_t sensors[5])
{
	uint8_t leftSideAlgorithm = navigation_left_algorithm();
	if(leftSideAlgorithm)
	{
		if(navigation_check_left_turn(sensors[0], sensors[2]) == 2)
		{
			turn_left();
		}
		else if(sensors[4] > CORRIDOR_WIDTH)
		{
			walk_forward(sensors);
		}
		else if(navigation_check_right_turn(sensors[1], sensors[3]) == 2)
		{
			turn_right();
		}
		else
		{
			turn_around(sensors[4]);
		}
	}
	else
	{
		if(navigation_check_right_turn(sensors[1], sensors[3]) == 2)
		{
			turn_left();
		}
		else if(sensors[4] > CORRIDOR_WIDTH)
		{
			walk_forward(sensors);
		}
		else if(navigation_check_left_turn(sensors[0], sensors[2]) == 2)
		{
			turn_right();
		}
		else
		{
			turn_around(sensors[4]);
		}
	}
}