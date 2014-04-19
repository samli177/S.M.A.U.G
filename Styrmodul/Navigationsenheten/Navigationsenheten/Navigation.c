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


// 0 means use a right side algorithm.
// 1 means use a left side algorithm.
uint8_t gAlgorithm = 1;

//A regulation parameter to determine how 
//hard to punish offset.
uint8_t gKp = 1;

// 0 means autonomous walk is disabled.
// 1 means autonomous walk is enabled.
uint8_t gAutonomousWalk = 0;

// A variable that shows how many sensor samples that will be used
// when taking the median sensor data.
int sensorBufferSize = 3;

// SensorBufferSize = 3.
uint8_t sensorBuffer[8][3]; 

// A help variable to be used in fill_buffer().
uint8_t currentBufferLine;

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

float navigation_angle_offset(uint8_t sensors[5])
{
	if (gAlgorithm && (sensors[2]+sensors[0]) < (CORRIDOR_WIDTH + 20))
	{
		return atanf((sensors[2]-sensors[0])/DISTANCE_FRONT_TO_BACK);
	}
	else if(gAlgorithm && sensors[4] > CORRIDOR_WIDTH)
	{
		return atanf((sensors[1]-sensors[3])/DISTANCE_FRONT_TO_BACK);
	}
	else if((sensors[1]+sensors[3]) < (CORRIDOR_WIDTH + 20))
	{
		return atanf((sensors[1]-sensors[3])/DISTANCE_FRONT_TO_BACK);
	}
	else if(sensors[4] > CORRIDOR_WIDTH)
	{
		return atanf((sensors[2]-sensors[0])/DISTANCE_FRONT_TO_BACK);
	}
	else
	{
		return 0;
	}
}

float navigation_direction_regulation(uint8_t sensors[5], float angleOffset)
{
	int d;
	if(gAlgorithm && sensors[4] > CORRIDOR_WIDTH)
	{
		d = (1/2 * (sensors[2]+sensors[0]) + DISTANCE_MIDDLE_TO_SIDE) * cosf(angleOffset);
	}
	else if (sensors[4] > CORRIDOR_WIDTH)
	{
		d = (CORRIDOR_WIDTH - (1/2 * (sensors[1]+sensors[3]) + DISTANCE_MIDDLE_TO_SIDE) * cosf(angleOffset));
	}
	else
	{
		return 0;
	}
	if((CORRIDOR_WIDTH/2 - d) < ACCEPTABLE_DISTANCE_OFFSET)
	{
		return 0;
	}
	else
	{
		return atanf((CORRIDOR_WIDTH/2 - d) * gKp);
	}
}

uint8_t navigation_check_left_turn(uint8_t frontLeftSensor, uint8_t backLeftSensor)
{
	if(frontLeftSensor > CORRIDOR_WIDTH && backLeftSensor > CORRIDOR_WIDTH)
	{
		return 2;
	}
	else if(frontLeftSensor > CORRIDOR_WIDTH)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_check_right_turn(uint8_t frontRightSensor, uint8_t backRightSensor)
{
	if(frontRightSensor > CORRIDOR_WIDTH && backRightSensor > CORRIDOR_WIDTH)
	{
		return 2;
	}
	else if(frontRightSensor > CORRIDOR_WIDTH)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_detect_low_pass_obsticle(uint8_t ultraSoundSensor)
{
	if (ultraSoundSensor < HEIGHT_LIMIT)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t navigation_dead_end(uint8_t sensor0, uint8_t sensor1, uint8_t sensor4, float angleOffset)
{
	if(sensor0 < (CORRIDOR_WIDTH/2 + 20) 
		&& sensor1 < (CORRIDOR_WIDTH/2 + 20)
		&& sensor4 < (CORRIDOR_WIDTH/2 - 10)
		&& fabs(angleOffset) < ACCEPTABLE_OFFSET_ANGLE)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int compare (const void * a, const void * b)
{
	return ( *(uint8_t*)a - *(uint8_t*)b );
}

uint8_t navigation_get_sensor(int sensorNr)
{
	uint8_t temp[sensorBufferSize];
	for(int i = 0; i < sensorBufferSize; ++i)
	{
		temp[i] = sensorBuffer[sensorNr][i];
	}
	qsort(temp, sensorBufferSize, sizeof(uint8_t), compare);
	return temp[sensorBufferSize / 2];
}

void navigation_fill_buffer()
{
	for(int i = 0; i < 8; ++i)
	{
		sensorBuffer[i][currentBufferLine] = TWI_get_sensor(i);
	}
	if(currentBufferLine == sensorBufferSize - 1)
	currentBufferLine = 0;
	else
	currentBufferLine += 1;
}

/************************************************************************/
/*      COUNTERS/TIMERS interrupt vectors                               */
/************************************************************************/
ISR(TIMER1_COMPA_vect)
{
	if(TWI_sensor_flag())
	navigation_fill_buffer();
	TCNT1 = 0;
}

ISR(TIMER2_COMPA_vect)
{
	TCNT2 = 0;
}