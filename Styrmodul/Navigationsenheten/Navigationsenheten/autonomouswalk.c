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
//Variable for the speed parameter in movement commands. 
uint8_t gSpeed = 50;

//Variable to decide if status messages are to be
//sent back to the PC.
uint8_t gStatus = 1;

//A counter to keep track of how many times the robot have
//found itself unable to make a decision.
uint8_t decisionCounter = 0;

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

void turn_degrees(int16_t degrees)
{
	if(degrees == 0)
	{
		return;
	}
	
	USART_RequestGyro();
	while(USART_GyroFlag() == 0)
	{
		USART_DecodeRxFIFO();
	}
	
	float startDegree = USART_gyro_get_Y() * 180/PI;
	float newDegree = startDegree;
	uint8_t steps = 0;
	float tolerance = 0;
	uint8_t done = 0;
	
	do
	{
		if(degrees > 0)
		{
			USART_send_command_parameters(0, MAX_ROTATION_COUNTER_CLOCKWISE, 0);
		} else {
			USART_send_command_parameters(0, MAX_ROTATION_CLOCKWISE, 0);
		}
		
		while(USART_ready() == 0)
		{
			PORTA ^= (1 << PORTA1);
			USART_RequestGyro();
			while(USART_GyroFlag() == 0)
			{
				USART_DecodeRxFIFO();
			}
		}
		newDegree = USART_gyro_get_Y() * 180/PI;
		++steps;
		
		float turned = fmod(fabs(newDegree - startDegree), 180);
		if(turned >= fabs(degrees) - tolerance)
		{
			done = 1;
			PORTA ^= (1 << PORTA0);
		}
	} while (done == 0);
}

void turn_left()
{
	if(gStatus)
	{
		TWI_send_string(C_ADDRESS, "Starting turning left.");
	}
	for(int i = 0; (i < 20 && TWI_get_autonom_settings() != 0); ++i)
	{
		if(gStatus)
		{
			//TWI_send_string(C_ADDRESS, "Rotating right.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_COUNTER_CLOCKWISE, 0);
		navigation_stepping_delay();
	}
	for(int i = 0; (i < 12 && TWI_get_autonom_settings() != 0); ++i)
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
	for(int i = 0; (i < 20 && TWI_get_autonom_settings() != 0); ++i)
	{
		if(gStatus)
		{
			//TWI_send_string(C_ADDRESS, "Rotating right.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_CLOCKWISE, 0);
		navigation_stepping_delay();
	}
	for(int i = 0; (i < 12 && TWI_get_autonom_settings() != 0); ++i)
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
	for(int i = 0; (i < 40 && TWI_get_autonom_settings() != 0); ++i)
	{
		if(gStatus)
		{
			//TWI_send_string(C_ADDRESS, "Rotating right.");
		}
		USART_send_command_parameters(0, MAX_ROTATION_COUNTER_CLOCKWISE, 0);
		navigation_stepping_delay();
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
		//TWI_send_string(C_ADDRESS, "Finding regulation parameters.");
	}
	float angleOffset = navigation_angle_offset();
	float directionCompensationAngle = navigation_direction_regulation(angleOffset);
	if(gStatus)
	{
		//TWI_send_string(C_ADDRESS, "Found regulation parameters.");
	}
	int adjustmentRotation = (51 + 50 * angleOffset * 2.0/PI);
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
		//TWI_send_string(C_ADDRESS, "Taking a step.");
	}
	USART_send_command_parameters((uint8_t)adjustmentDirection, (uint8_t)adjustmentRotation, gSpeed);
	//TWI_send_float(C_ADDRESS, adjustmentDirection);
	navigation_stepping_delay();
	//TWI_send_float(C_ADDRESS, adjustmentRotation);
}

void autonomouswalk_walk()
{
	navigation_low_pass_obstacle();
	if(navigation_left_algorithm())
	{
		if(navigation_check_left_turn() == 2)
		{
			for(int i = 0;(i < 4 && TWI_get_autonom_settings() != 0); ++i)
			{
				walk_forward();
			}
			turn_left();
		}
		else if(navigation_get_sensor(4) > CORRIDOR_WIDTH / 2)
		{
			walk_forward();
		}
		else if(navigation_check_right_turn() == 2)
		{
			turn_right();
		}
		else if(navigation_check_left_turn() == 0 && navigation_check_right_turn() == 0)
		{
			turn_around();
		}
		else
		{
			walk_forward();
		}
	}
	else
	{
		if(navigation_check_right_turn() == 2)
		{
			for(int i = 0;(i < 4 && TWI_get_autonom_settings() != 0); ++i)
			{
				walk_forward();
			}
			turn_right();
		}
		else if(navigation_get_sensor(4) > CORRIDOR_WIDTH / 2)
		{
			walk_forward();
		}
		else if(navigation_check_left_turn() == 2)
		{
			turn_left();
		}
		else if(navigation_check_left_turn() == 0 && navigation_check_right_turn() == 0)
		{
			turn_around();
		}
		else if(decisionCounter < 4)
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