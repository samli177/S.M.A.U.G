/*
 * highLevelWalking.c
 *
 * Created: 4/9/2014 3:38:22 PM
 *  Author: jonha860
 */ 


#include <avr/io.h>
#include <math.h>
#include "serialServoControl.h"
#include "inverseKinematics.c"
#define frontLegDistance 200
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85
int x0_1 = -2000; //standard x pos for leg 1
int y0_1 = 2200; //standard y pos for leg 1
int x0_2 = -1800; //standard x pos for leg 2
int y0_2 = 0; //standard y pos for leg 2
int x0_3 = -2000; //standard x pos for leg 3
int y0_3 = -2200; //standard y pos for leg 3
int x0_4 = 2000; //standard x pos for leg 4
int y0_4 = -2200; //standard y pos for leg 4
int x0_5 = 1800; //standard x pos for leg 5
int y0_5 = 0; //standard y pos for leg 5
int x0_6 = -2000; //standard x pos for leg 6
int y0_6 = 2200; //standard y pos for leg 6

	
void moveLeg2too(int x,int y,int z,int speed)
{}

void moveLeg3too(int x,int y,int z,int speed)
{}

void moveLeg4too(int x,int y,int z,int speed)
{}

void moveLeg5too(int x,int y,int z,int speed)
{}

void moveLeg6too(int x,int y,int z,int speed)
{}

void moveRobot(int direction,int distance, int rotation, int z, int servoSpeed, int rotationX, int rotationY)
{
	double sinrotation = sin(rotation);
	double cosrotation = cos(rotation);
	double sindirection = sin(direction);
	double cosdirection = cos(direction);
	double sinaroundx = sin(rotationX);
	double sinaroundy = cos(rotationY);
	
	//First state-------------------
	servoGoto(4, 3.1415/4, servoSpeed); //raise legs
	servoGoto(10, 3.1415/4, servoSpeed);
	servoGoto(15, -3.1415/4, servoSpeed);
	_delay_ms(50);
	moveLeg1too(x0_1*cosrotation-y0_1*sinrotation-sindirection*distance, y0_1*cosrotation-x0_1*sinrotation+cosdirection*distance, z+sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg3too(x0_3*cosrotation-y0_3*sinrotation-sindirection*distance, y0_3*cosrotation-x0_3*sinrotation+cosdirection*distance, z+sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg5too(x0_5*cosrotation-y0_5*sinrotation-sindirection*distance, y0_5*cosrotation+x0_5*sinrotation+cosdirection*distance, z-centerToSideLegs*sinaroundy, servoSpeed);
	_delay_ms(1000);
		
	//Second state-------------------
	servoGoto(3, -3.1415/4, servoSpeed); //raise legs
	servoGoto(9, -3.1415/4, servoSpeed);
	servoGoto(16, 3.1415/4, servoSpeed);
	_delay_ms(50);
	moveLeg2too(x0_2*cosrotation-y0_2*sinrotation-sindirection*distance, y0_2*cosrotation+x0_2*sinrotation+cosdirection*distance, z+centerToSideLegs*sinaroundy, servoSpeed);
	moveLeg4too(x0_4*cosrotation-y0_4*sinrotation-sindirection*distance, y0_4*cosrotation-x0_4*sinrotation+cosdirection*distance, z-sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg6too(x0_6*cosrotation-y0_6*sinrotation-sindirection*distance, y0_6*cosrotation-x0_6*sinrotation+cosdirection*distance, z-sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	_delay_ms(1000);
		
	//Third state-------------------
	moveLeg1too(x0_1, y0_1, z+sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg2too(x0_2, y0_2, z+centerToSideLegs*sinaroundy, servoSpeed);
	moveLeg3too(x0_3, y0_3, z+sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg4too(x0_4, y0_4, z-sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg5too(x0_5, y0_5, z-centerToSideLegs*sinaroundy, servoSpeed);
	moveLeg6too(x0_6, y0_6, z-sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	
}

int main(void)
{
	DDRD |= (1<<PORTD5); //init LED
	servoTx;
	initServoSerial(); //Init servos
	
	
    while(1)
    {
        //TODO:: Please write your application code 
    }
}