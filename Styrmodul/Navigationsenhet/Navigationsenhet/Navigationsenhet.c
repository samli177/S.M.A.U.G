/*
 * Navigationsenhet.c
 *
 * Created: 4/9/2014 1:26:00 PM
 *  Author: camal940
 */ 


#include <avr/io.h>

bool automonity = false;
int algorithm = 0; // 0 = right algorithm is being used.

int right_sensor_front;
int right_sensor_back;
int left_sensor_front;
int left_sensor_back;
int forward_sensor;
int low_pass_sensor;
int high_pass_sensor[2]; // Hur många punkter?

void forward();
bool right_turn_possible?();
bool left_turn_possible?();
bool forward_possible?();
bool obstacle?();
bool low_pass_obstacle?();
void turn_right();
void turn_left();
void walk_forward();
void low_pass_walk();
void high_pass_walk();
void dead_end();

int main(void)
{
    while(1)
    {
        if(autonomity)
		{
			if(algorithm == 1)
			{
				if(right_turn_possible?())
				{
					turn_right();
				}
				else
				{
					forward();
				}
			}
			else
			{
				if(left_turn_possible?())
				{
					turn_left();
				}
				else
				{
					forward();
				}
			}
			
		}
		else // Manual mode
		{
			// Send instructions from kommunikationsmodulen to styrenheten 
		}
    }
}


void forward()
{
	if(forword_possible?())
	{
		walk_forward();
	}
	else if(obstacle?())
	{
		if(low_pass_obstacle?())
		{
			low_pass_walk();
		}
		else
		{
			high_pass_walk();
		}
	}
	else if(algorithm == 1)
	{
		if(right_turn_possible?())
		{
			turn_right();
		}
	}
	else if(algorithm == 0)
	{
		if(left_turn_possible?())
		{
			turn_left();
		}
	}
	else
	{
		dead_end();
	}
}

bool right_turn_possible?()
{
	return (right_sensor_front > 40);
}

bool left_turn_possible?()
{
	return (left_sensor_front > 40);
}

bool forward_possible?()
{
	return (forward_sensor > 70);
}

bool obstacle?()
{
	// begär svep från sensormodulen här.
	return ((low_pass_sensor < 30) | (high_pass_sensor[0] < high_pass_sesnor[1] - 40));
}

bool low_pass_obstacle?()
{
	return(low_pass_sensor < 30);
}

/* since it only checks the front sensors to the side it should walk a bit 
 forward before turning */
void turn_right()
{
	walk_forward();
	when(right_sensor_back > 40)
	{
		// Send turn command to styrenheten
	}
}

void turn_left()
{
	walk_forward();
	when(left_sensor_back > 40)
	{
		// Send turn command to styrenheten
	}
}

void walk_forward()
{
	// reglering för riktning här?
}

void low_pass_walk()
{
	
}

void high_pass_walk()
{
	
}

void dead_end()
{
	// Turn 180 degrees and start walking forward.
}