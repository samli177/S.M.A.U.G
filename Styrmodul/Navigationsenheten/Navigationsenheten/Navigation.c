/*
 * Navigation.c
 *
 * Created: 4/17/2014 1:10:11 PM
 *  Author: jonha860
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "twi.h"
#include "usart.h"
#include "counter.h"
#include "Navigation.h"
#include "LED.h"

#define SENSOR_BUFFER_SIZE 5

float RIGHT_SIDE_OFFSET = 5;
float LEFT_SIDE_OFFSET = 5;
float REGULATION_MAX_SENSOR_DIFF = 10;
float REGULATION_MAX_SENSOR_DIST_ROT = 60;
float REGULATION_MAX_SENSOR_DIST_D = 50;

//A regulation parameter to determine how
//hard to punish offset. Small Kp => small punishment.
float G_KP = 0.05;

// 0 means use a right side algorithm.
// 1 means use a left side algorithm.
uint8_t gAlgorithm = 1;

// 0 means autonomous walk is disabled.
// 1 means autonomous walk is enabled.
uint8_t gAutonomousWalk = 1;

// Used to take the median of the most recent measurements.
float sensorMedianBuffer[8][SENSOR_BUFFER_SIZE];
uint8_t medianBuffer[8];

// A help variable to be used in fill_buffer().
uint8_t currentBufferLine = 0;

// A flag to note if the robot is in the middle of a 
// low pass obstacle.
uint8_t lowPassObstacleFlag = 0;

//------------- Internal declarations ---------------

//static int compare(const void * a, const void * b);
static float sortAndFilter(float data[SENSOR_BUFFER_SIZE]);

//--------------- Internal functions ----------------

/**
 * \brief 
 * A help function to decide which of two numbers is larger.
 * Only works for positive numbers. 
 * 
 * \param a
 * The first number to be compared.
 *
 * \param b
 * The second number to be compared.
 * 
 * \return int
 * Returns a positive number if a is larger than b, returns
 * 0 if a = b, and a negative number if b is larger than a.
 */
/*int compare (const void * a, const void * b)
{
	return ( *(uint8_t*)a - *(uint8_t*)b );
}*/


/**
 * \brief 
 * A function that takes a vector, sorts it using bubble sort and
 * returns the median of it. 
 *
 * \param data
 * The vector to be sorted.
 * 
 * \return uint8_t
 * The returned median as a uint8_t
 */
float sortAndFilter(float data[SENSOR_BUFFER_SIZE])
{
	float largest;
	for (int i=0; i < SENSOR_BUFFER_SIZE-1; ++i)
	{
		for(int j = 0; j < SENSOR_BUFFER_SIZE - i - 1; ++j)
		{
			if(data[j] > data[j+1])
			{
				largest = data[j];
				data[j] = data[j + 1];
				data[j + 1] = largest;
			}
		}
	}
	return data[SENSOR_BUFFER_SIZE / 2];
}

//--------------- External functions ----------------

void navigation_init()
{
	//INT0 & INT1 on PD2 & PD3 respectively inputs
	DDRD &= ~((1<<PORTD2) | (1<<PORTD3));
	//Interrupt on INT0 & INT1 on rising edge.
	EICRA |= (1<<ISC00) | (1<<ISC01) | (1<<ISC10) | (1<<ISC11);
	//Interrupt enable on INT0 & INT1
	EIMSK |= (1<<INT0) | (1<<INT1);
}

uint8_t navigation_get_Kp()
{
	return G_KP;
}

void navigation_set_Kp(uint8_t Kp)
{
	G_KP = Kp;
}

uint8_t navigation_left_algorithm()
{
	return gAlgorithm;
}

void navigation_set_algorithm(uint8_t alg)
{
	gAlgorithm = alg;
}

uint8_t navigation_autonomous_walk()
{
	return gAutonomousWalk;
}

void navigation_set_autonomous_walk(uint8_t walk)
{
	gAutonomousWalk = walk;
}

void navigation_stepping_delay()
{
	while(USART_ready() == 0)
	{
		USART_decode_rx_fifo();
		_delay_ms(10);
	}
}

void navigation_low_pass_obstacle()
{
	if(navigation_detect_low_pass_obsticle() && !(lowPassObstacleFlag))
	{
		TWI_send_string_fixed_length(S_ADDRESS,"Low pass obstacle detected.", 27);
		_delay_ms(30);
		if(autonomouswalk_get_return_status())
		{
			TWI_send_string_fixed_length(C_ADDRESS,"Low pass obstacle detected.", 27);	
		}
		lowPassObstacleFlag = 1;
	}
	else if(!(navigation_detect_low_pass_obsticle()))
	{
		lowPassObstacleFlag = 0;
	}
	
}

float navigation_angle_offset()
{
	float angle = 0;
	if (gAlgorithm)
	{
		if(abs(navigation_get_sensor(2) - navigation_get_sensor(0)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(0) < REGULATION_MAX_SENSOR_DIST_ROT)
		{
			// Use wall to the left
			angle = atanf((navigation_get_sensor(2) - navigation_get_sensor(0))/DISTANCE_FRONT_TO_BACK);
		}
		else if(abs(navigation_get_sensor(1) - navigation_get_sensor(3)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(1) < REGULATION_MAX_SENSOR_DIST_ROT)
		{
			// Use wall to the right
			angle = atanf((navigation_get_sensor(1) - navigation_get_sensor(3))/DISTANCE_FRONT_TO_BACK);
		}
	}
	else 
	{
		if(abs(navigation_get_sensor(1) - navigation_get_sensor(3)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(1) < REGULATION_MAX_SENSOR_DIST_ROT)
		{
			// Use wall to the right
			angle = atanf((navigation_get_sensor(1) - navigation_get_sensor(3))/DISTANCE_FRONT_TO_BACK);
		}
		else if(abs(navigation_get_sensor(2) - navigation_get_sensor(0)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(0) < REGULATION_MAX_SENSOR_DIST_ROT)
		{
			// Use wall to the left
			angle = atanf((navigation_get_sensor(2) - navigation_get_sensor(0))/DISTANCE_FRONT_TO_BACK);
		}
	}
	
	if(fabs(angle) > ACCEPTABLE_OFFSET_ANGLE)
	{
		return angle;
	}
	else
	{
		return 0;
	}
}

float navigation_direction_regulation(float angleOffset)
{
	int d = 0;
	if(gAlgorithm)
	{
		if(abs(navigation_get_sensor(2) - navigation_get_sensor(0)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(0) < REGULATION_MAX_SENSOR_DIST_D)
		{
			d = ((navigation_get_sensor(2) + navigation_get_sensor(0)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE - LEFT_SIDE_OFFSET) * cosf(angleOffset) - CORRIDOR_WIDTH / 2;
		}
		else if(abs(navigation_get_sensor(1) - navigation_get_sensor(3)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(1) < REGULATION_MAX_SENSOR_DIST_D)
		{
			d = CORRIDOR_WIDTH / 2 - ((navigation_get_sensor(1) + navigation_get_sensor(3)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE - RIGHT_SIDE_OFFSET) * cosf(angleOffset);
		}
	}
	else 
	{
		if(abs(navigation_get_sensor(1) - navigation_get_sensor(3)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(1) < REGULATION_MAX_SENSOR_DIST_D)
		{
			d = CORRIDOR_WIDTH / 2 - ((navigation_get_sensor(1) + navigation_get_sensor(3)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE - RIGHT_SIDE_OFFSET) * cosf(angleOffset);
		}
		else if(abs(navigation_get_sensor(2) - navigation_get_sensor(0)) < REGULATION_MAX_SENSOR_DIFF && navigation_get_sensor(0) < REGULATION_MAX_SENSOR_DIST_D)
		{
			d = ((navigation_get_sensor(2) + navigation_get_sensor(0)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE - LEFT_SIDE_OFFSET) * cosf(angleOffset) - CORRIDOR_WIDTH / 2;
		}
	}
	
	if(abs(d) < ACCEPTABLE_DISTANCE_OFFSET)
	{
		return 0;
	}
	else
	{
		float dir = atanf(d * G_KP);
		if(dir < 0)
		{
			dir += 2*PI;
		} else if(dir >= 2*PI)
		{
			dir -= 2*PI;
		}
		// Dir is between 0 and 2*PI radians
		return dir;
	}
}

uint8_t navigation_check_left_turn()
{
	if(navigation_get_sensor(0) >= (CORRIDOR_WIDTH / 2 + 20))
	{
		return 2;
	}
	else if(navigation_get_sensor(0) >= (CORRIDOR_WIDTH / 2 + 10))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_check_right_turn()
{
	if(navigation_get_sensor(1) >= (CORRIDOR_WIDTH / 2 + 20))
	{
		return 2;
	}
	else if(navigation_get_sensor(1) >= (CORRIDOR_WIDTH / 2 + 10))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_detect_low_pass_obsticle()
{
	if (navigation_get_sensor(7) < HEIGHT_LIMIT)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_dead_end(float angleOffset)
{
	if(navigation_get_sensor(0) < (CORRIDOR_WIDTH / 2 + 20)
		&& navigation_get_sensor(1) < (CORRIDOR_WIDTH / 2 + 20)
		&& navigation_get_sensor(4) < (CORRIDOR_WIDTH / 2 - 10)
		&& fabs(angleOffset) < ACCEPTABLE_OFFSET_ANGLE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void navigation_fill_buffer()
{
	for(int i = 0; i < 8; ++i)
	{
		sensorMedianBuffer[i][currentBufferLine] = TWI_get_sensor(i);
	}
	if(currentBufferLine == SENSOR_BUFFER_SIZE - 1)
	{
		currentBufferLine = 0;
	} else {
		currentBufferLine += 1;
	}
}

float navigation_get_sensor(int sensorNr)
{
	float temp[SENSOR_BUFFER_SIZE];
	for(int j = 0; j < SENSOR_BUFFER_SIZE; ++j)
	{
		temp[j] = sensorMedianBuffer[sensorNr][j];
	}
	//qsort(temp, sensorBufferSize, sizeof(uint8_t), compare);
	return sortAndFilter(temp);
}

void navigation_update_parameters(uint8_t parameters[37]){
	// Do stuff
}

//-------------------------------Interrupts--------------------------------

//External interrupt INT0, to activate autonomous walk and set left hand navigation
ISR(INT0_vect)
{
	navigation_set_algorithm(1); //Left hand
	navigation_set_autonomous_walk(1);
}

//External interrupt INT1, to activate autonomous walk and set right hand navigation
ISR(INT1_vect)
{
	navigation_set_algorithm(0); //Right hand
	navigation_set_autonomous_walk(1);
}