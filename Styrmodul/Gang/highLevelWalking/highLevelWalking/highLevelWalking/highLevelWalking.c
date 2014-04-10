/*
 * highLevelWalking.c
 *
 * Created: 4/9/2014 3:38:22 PM
 *  Author: jonha860
 */ 

#define F_CPU 16000000 

#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "serialServoControl.h"
#include "inverseKinematics.h"
#include "usart.h"

#define frontLegDistance 200
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85
int x0_1 = -240; //standard x pos for leg 1
int y0_1 = 250; //standard y pos for leg 1
int x0_2 = -250; //standard x pos for leg 2
int y0_2 = 0; //standard y pos for leg 2
int x0_3 = -240; //standard x pos for leg 3
int y0_3 = -250; //standard y pos for leg 3
int x0_4 = 240; //standard x pos for leg 4
int y0_4 = -250; //standard y pos for leg 4
int x0_5 = 250; //standard x pos for leg 5
int y0_5 = 0; //standard y pos for leg 5
int x0_6 = -240; //standard x pos for leg 6
int y0_6 = 250; //standard y pos for leg 6

void moveRobot(int direction,int distance, int rotation, int z, int servoSpeed, int rotationX, int rotationY)
{
	double sinrotation = sin(rotation);
	double cosrotation = cos(rotation);
	double sindirection = sin(direction);
	double cosdirection = cos(direction);
	double sinaroundx = sin(rotationX);
	double sinaroundy = cos(rotationY);
	
	//First state-------------------
	servoGoto(4, 3.1415/2, servoSpeed); //raise legs
	servoGoto(10, 3.1415/2, servoSpeed);
	servoGoto(15, -3.1415/2, servoSpeed);
	_delay_ms(1000);
	moveLeg1too(x0_1*cosrotation-y0_1*sinrotation-sindirection*distance, y0_1*cosrotation-x0_1*sinrotation+cosdirection*distance, z+sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg3too(x0_3*cosrotation-y0_3*sinrotation-sindirection*distance, y0_3*cosrotation-x0_3*sinrotation+cosdirection*distance, z+sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg5too(x0_5*cosrotation-y0_5*sinrotation-sindirection*distance, y0_5*cosrotation+x0_5*sinrotation+cosdirection*distance, z-centerToSideLegs*sinaroundy, servoSpeed);
	_delay_ms(3000);
		
	//Second state-------------------
	servoGoto(3, -3.1415/2, servoSpeed); //raise legs
	servoGoto(9, -3.1415/2, servoSpeed);
	servoGoto(16, 3.1415/2, servoSpeed);
	_delay_ms(1000);
	moveLeg2too(x0_2*cosrotation-y0_2*sinrotation-sindirection*distance, y0_2*cosrotation+x0_2*sinrotation+cosdirection*distance, z+centerToSideLegs*sinaroundy, servoSpeed);
	moveLeg4too(x0_4*cosrotation-y0_4*sinrotation-sindirection*distance, y0_4*cosrotation-x0_4*sinrotation+cosdirection*distance, z-sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY, servoSpeed);
	moveLeg6too(x0_6*cosrotation-y0_6*sinrotation-sindirection*distance, y0_6*cosrotation-x0_6*sinrotation+cosdirection*distance, z-sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY, servoSpeed);
	_delay_ms(3000);
		
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
	//servoTx;
	
	servoRetrunLevel(BROADCASTING_ID, 1); //turns off return packets
	
	sei();
	initServoSerial(); //Init servos
	USART_init();
	
	/*double alpha = 3.1415/4;
	double beta = 3.1415/2.2;
	int speed = 180;
	
	//dummy code that puts robot in standard position
	
	servoGoto(1, 0, speed);
	servoGoto(2, 0, speed);
	servoGoto(7, 0, speed);
	servoGoto(8, 0, speed);
	servoGoto(13, 0, speed);
	servoGoto(14, 0, speed);
	
	_delay_ms(100);
	
	servoGoto(15, -alpha, speed);
	servoGoto(3, -alpha, speed);
	servoGoto(9, -alpha, speed);
	servoGoto(16, alpha, speed);
	servoGoto(4, alpha, speed);
	servoGoto(10, alpha, speed);
	
	_delay_ms(100);
	
	servoGoto(12, -beta, speed);
	servoGoto(18, -beta, speed);
	servoGoto(6, -beta, speed);
	servoGoto(5, beta, speed);
	servoGoto(11, beta, speed);
	servoGoto(17, beta, speed);
	
	_delay_ms(5000);*/
	
	moveLeg1too(x0_1, y0_1, 70, 80);
	moveLeg2too(x0_2, y0_2, 70, 80);
	moveLeg3too(x0_3, y0_3, 70, 80);
	moveLeg4too(x0_4, y0_4, 70, 80);
	moveLeg5too(x0_5, y0_5, 70, 80);
	moveLeg6too(x0_6, y0_6, 70, 80);
	
	_delay_ms(10000);
	
    while(1)
    {
		USART_DecodeRxFIFO();
		_delay_ms(3000);
		moveRobot(0,25,0,70,100,0,0);
        //TODO:: Please write your application code 
    }
}