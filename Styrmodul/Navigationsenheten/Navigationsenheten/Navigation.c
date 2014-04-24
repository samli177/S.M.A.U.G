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
#include "twi.h"
#include "usart.h"
#include "counter.h"
#include "Navigation.h"
#include <math.h>

#define sensorBufferSize 3

// 0 means use a right side algorithm.
// 1 means use a left side algorithm.
static uint8_t gAlgorithm = 1;

//A regulation parameter to determine how 
//hard to punish offset. Small Kp => small punishment.
static float gKp = 0.1;

// 0 means autonomous walk is disabled.
// 1 means autonomous walk is enabled.
static uint8_t gAutonomousWalk = 0;

// Used to take the median of the most recent measurements.
static uint8_t sensorBuffer[8][sensorBufferSize];
static uint8_t medianBuffer[8];

// A help variable to be used in fill_buffer().
static uint8_t currentBufferLine = 0;

//------------- Internal declarations ---------------

static int compare(const void * a, const void * b);
static void update_median();

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
int compare (const void * a, const void * b)
{
	return ( *(uint8_t*)a - *(uint8_t*)b );
}

/**
 * \brief 
 * Uses sensorBuffer to calculate the median of each sensor
 * over the last x measurements, where x is sensorBufferSize.
 *
 * \return void
 */
void update_median()
{
	for(int i = 0; i < 8; ++i)
	{
		uint8_t temp[sensorBufferSize];
		for(int j = 0; j < sensorBufferSize; ++j)
		{
			temp[j] = sensorBuffer[i][j];
		}
		qsort(temp, sensorBufferSize, sizeof(uint8_t), compare);
		medianBuffer[i] = temp[sensorBufferSize / 2];
	}
}

//--------------- External functions ----------------

uint8_t navigation_get_Kp()
{
	return gKp;
}

void navigation_set_Kp(uint8_t Kp)
{
	gKp = Kp;
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

float navigation_angle_offset()
{
	if (gAlgorithm && (navigation_get_sensor(2) + navigation_get_sensor(0)) < (CORRIDOR_WIDTH + 20))
	{
		// Use wall to the left
		return atanf((navigation_get_sensor(2) - navigation_get_sensor(0))/DISTANCE_FRONT_TO_BACK);
	}
	else if(gAlgorithm && navigation_get_sensor(4) > CORRIDOR_WIDTH)
	{
		// No wall to the left, use wall to the right
		return atanf((navigation_get_sensor(1) - navigation_get_sensor(3))/DISTANCE_FRONT_TO_BACK);
	}
	else if((navigation_get_sensor(1) + navigation_get_sensor(3)) < (CORRIDOR_WIDTH + 20))
	{
		// Use wall to the right
		return atanf((navigation_get_sensor(1) - navigation_get_sensor(3))/DISTANCE_FRONT_TO_BACK);
	}
	else if(navigation_get_sensor(4) > CORRIDOR_WIDTH)
	{
		// No wall to the right, use wall to the left
		return atanf((navigation_get_sensor(2) - navigation_get_sensor(0))/DISTANCE_FRONT_TO_BACK);
	}
	else
	{
		// Default, don't care about angle
		return 0;
	}
}

float navigation_direction_regulation(float angleOffset)
{
	// ---------------------------------- Skum funktion ----------------------------------- (Jonas f�rklara...)
	// �r positiv riktning �t v�nster???
	// Ska vi inte reglera om vi n�rmar oss en v�gg framf�r?
	
	int d;
	if(navigation_get_sensor(4) > CORRIDOR_WIDTH)
	{
		if(gAlgorithm)
		{
			d = ((navigation_get_sensor(2) + navigation_get_sensor(0)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE) * cosf(angleOffset) - CORRIDOR_WIDTH / 2;
		} else {
			d = CORRIDOR_WIDTH / 2 - ((navigation_get_sensor(1) + navigation_get_sensor(3)) / 2.0 + DISTANCE_MIDDLE_TO_SIDE) * cosf(angleOffset);
		}
		
		if(abs(d) < ACCEPTABLE_DISTANCE_OFFSET)
		{
			return 0;
		}
		else
		{
			float dir = atanf(d * gKp);
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
	return 0;
}

uint8_t navigation_check_left_turn()
{
	if(navigation_get_sensor(0) >= CORRIDOR_WIDTH && navigation_get_sensor(2) >= CORRIDOR_WIDTH)
	{
		return 2;
	}
	else if(navigation_get_sensor(0) >= CORRIDOR_WIDTH)
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
	if(navigation_get_sensor(1) >= CORRIDOR_WIDTH && navigation_get_sensor(3) >= CORRIDOR_WIDTH)
	{
		return 2;
	}
	else if(navigation_get_sensor(1) >= CORRIDOR_WIDTH)
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
		sensorBuffer[i][currentBufferLine] = TWI_get_sensor(i);
	}
	if(currentBufferLine == sensorBufferSize - 1)
	{
		currentBufferLine = 0;
	} else {
		currentBufferLine += 1;
	}
	update_median();
}

uint8_t navigation_get_sensor(int sensorNr)
{
	return medianBuffer[sensorNr];
}

// -------------- COUNTERS/TIMERS interrupt vectors ----------------

ISR(TIMER1_COMPA_vect)
{
	if(TWI_sensor_flag())
	navigation_fill_buffer();
	TCNT1 = 0;
}

ISR(TIMER3_COMPA_vect)
{
	TCNT3 = 0;
}