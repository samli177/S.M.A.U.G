/*
 * serialServoControl.c
 *
 * Created: 4/10/2014 0:56:49 PM
 *  Author: jonha860
 */ 

#include <avr/io.h>
#include "serialServoControl.h"
#include "inverseKinematics.h"


#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


float alpha;
float beta;
float gamma;
float d;

void calc_d(float x,float y,float z)
{
	d = sqrtf(powf(sqrtf(x*x + y*y)-coxa, 2) + z*z);
}

float get_gamma(float x,float y)
{
gamma = atanf(y/x);
return gamma;
}

float get_beta()
{
	beta = pi - acosf((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	return beta;
}

float get_alpha(float z)
{
	alpha = acosf((femur*femur-tibia*tibia+d*d)/(2*femur*d))+asinf(z/d);
	return alpha;
}


float basis_change_leg1x(float x, float y) {
return (x + x0_1 + centerToFrontLegsX)/(-sqrt2)-(y + y0_1 - centerToFrontLegsY)/(-sqrt2);}

float basis_change_leg1y(float x, float y) {
return (x + x0_1 + centerToFrontLegsX)/(-sqrt2)+(y + y0_1 - centerToFrontLegsY)/(-sqrt2);}


float basis_change_leg2x(float x) {
return -(x + x0_2 + centerToSideLegs);}

float basis_change_leg2y(float y) {
return -(y + y0_2);}


float basis_change_leg3x(float x, float y) {
return (x + x0_3 + centerToFrontLegsX)/(-sqrt2)-(y + y0_3 +centerToFrontLegsY)/sqrt2;}

float basis_change_leg3y(float x, float y) {
return (x + x0_3 + centerToFrontLegsX)/sqrt2+(y + y0_3 + centerToFrontLegsY)/(-sqrt2);}


float basis_change_leg4x(float x, float y) {
return (x + x0_4 - centerToFrontLegsX)/sqrt2-(y + y0_4 + centerToFrontLegsY)/sqrt2;}

float basis_change_leg4y(float x, float y) {
return (x + x0_4 - centerToFrontLegsX)/sqrt2+(y + y0_4 + centerToFrontLegsY)/sqrt2;}


float basis_change_leg5x(float x) {
return (x + x0_5 - centerToSideLegs);}

float basis_change_leg5y(float y) {
return (y + y0_5);}


float basis_change_leg6x(float x, float y) {
return (x + x0_6 - centerToFrontLegsX)/sqrt2-(y + y0_6 - centerToFrontLegsY)/(-sqrt2);}

float basis_change_leg6y(float x, float y) {
	return (x + x0_6 - centerToFrontLegsX)/(-sqrt2)+(y + y0_6 - centerToFrontLegsY)/sqrt2;}


/*

void LegGoto(float x,float y, int z, int servospeed, int side, int servo1, int servo2, int servo3)
{
	*/
	//Ett slut vid bortkommentering av allt
	/*
	float alpha;
	float beta;
	float gamma;
	float d;
	*/
	/*
	
	gamma = atanf(y/x);
	d = sqrt(pow(sqrt(x*x + y*y)-coxa, 2) + z*z);
	beta = 3.1415 - acosf((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	alpha = acosf((femur*femur-tibia*tibia+d*d)/((float)2*femur*d))-asinf(fabs(z)/d);
	
	servoGoto(servo1, gamma, servospeed);
	_delay_ms(1);
	servoGoto(servo2, side*(alpha + femurAngleAddition),servospeed);
	_delay_ms(1);
	servoGoto(servo3, side*(-beta + tibiaAngleAddition),servospeed);
}



void moveLeg1too(float x, float y,float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x + centerToFrontLegsX)*cosf((float)(-3*3.1415/4))-(y-centerToFrontLegsY)*sinf((float)(-3*3.1415/4));
	float b = (x + centerToFrontLegsX)*sinf((float)(-3*3.1415/4))+(y-centerToFrontLegsY)*cosf((float)(-3*3.1415/4));
	LegGoto(a, b, (int) z, servospeed,1,8,10,12);
}


void moveLeg2too(float x, float y, float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x + centerToSideLegs)*cosf(3.1415)-(y)*sinf(-3.1415);
	float b = (x + centerToSideLegs)*sinf(-3.1415)+(y)*cosf(3.1415);
	LegGoto(a, b, (int) z, servospeed,1,14,16,18);
}

void moveLeg3too(float x, float y, float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x + centerToFrontLegsX)*cosf(-5*3.1415/4)-(y+centerToFrontLegsY)*sinf(-5*3.1415/4);
	float b = (x + centerToFrontLegsX)*sinf(-5*3.1415/4)+(y+centerToFrontLegsY)*cosf(-5*3.1415/4);
	LegGoto(a, b, (int) z, servospeed,1,2,4,6);
}

void moveLeg4too(float x, float y, float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x - centerToFrontLegsX)*cosf(3.1415/4)-(y+centerToFrontLegsY)*sinf(3.1415/4);
	float b = (x - centerToFrontLegsX)*sinf(3.1415/4)+(y+centerToFrontLegsY)*cosf(3.1415/4);
	LegGoto(a, b, (int) z, servospeed,-1,1,3,5);
}

void moveLeg5too(float x, float y, float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x - centerToSideLegs)*cosf(0)-(y)*sinf(0);
	float b = (x - centerToSideLegs)*sinf(0)+(y)*cosf(0);
	LegGoto(a, b, (int) z, servospeed,-1,13,15,17);
}

void moveLeg6too(float x, float y, float z, int servospeed) //Help function to describe position of leg in stanfdard base x,y,z
{
	float a = (x - centerToFrontLegsX)*cosf(-3.1415/4)-(y-centerToFrontLegsY)*sinf(-3.1415/4);
	float b = (x - centerToFrontLegsX)*sinf(-3.1415/4)+(y-centerToFrontLegsY)*cosf(-3.1415/4);
	LegGoto(a, b, (int) z, servospeed,-1,7,9,11);
}

*/