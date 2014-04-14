/*
 * serialServoControl.c
 *
 * Created: 4/10/2014 0:56:49 PM
 *  Author: jonha860
 */ 

#include <avr/io.h>
#include "serialServoControl.h"

#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


#define coxa (float)56
#define femur (float)66
#define tibia (float)131
#define femurAngleAddition (float)0.2426
#define tibiaAngleAddition (float)(-3.1415/6)
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85


#define x0_1 -215 //stanfdard x pos for leg 1
#define y0_1  275 //stanfdard y pos for leg 1
#define x0_2  -340 //stanfdard x pos for leg 2
#define y0_2  0 //stanfdard y pos for leg 2
#define x0_3  -215 //stanfdard x pos for leg 3
#define y0_3  -275 //stanfdard y pos for leg 3
#define x0_4 215 //stanfdard x pos for leg 4
#define y0_4  -275 //stanfdard y pos for leg 4
#define x0_5  340 //stanfdard x pos for leg 5
#define y0_5  0 //stanfdard y pos for leg 5
#define x0_6  215 //stanfdard x pos for leg 6
#define y0_6  275 //stanfdard y pos for leg 6

#define Pi 3.14159265

float sinfAngleToFrontLegs = centerToFrontLegsY / centerToFrontLegs;
float cosfAngleToFrontLegs = centerToFrontLegsX / centerToFrontLegs;

float alpha;
float beta;
float gamma;
float d;



void LegGoto(float x,float y, int z, int servospeed, int side, int servo1, int servo2, int servo3)
{
	/*
	float alpha;
	float beta;
	float gamma;
	float d;
	*/
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


//New functions by Tobias
void Calc_d(float x,float y,float z)
{
	d = sqrt(pow(sqrt(x*x + y*y)-coxa, 2) + z*z);
}

float Calc_gamma(float x,float y)
{
gamma = atanf(y/x);
return gamma;
}

float Calc_Beta(float x,float y,float z)
{
	beta = Pi - acosf((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	return beta;
}

float Calc_Alpha(float x,float y,float z)
{
	alpha = acosf((femur*femur-tibia*tibia+d*d)/(2*femur*d))+asinf(z/d);
	return alpha;
}


float basis_change_Leg1x(float x, float y, float z) {
return (x + x0_1 + centerToFrontLegsX)*cosf(-3*Pi/4)-(y + y0_1 - centerToFrontLegsY)*sinf(-3*Pi/4);}

float basis_change_Leg1y(float x, float y, float z) {
return (x + x0_1 + centerToFrontLegsX)*sinf(-3*Pi/4)+(y + y0_1 - centerToFrontLegsY)*cosf(-3*Pi/4);}


float basis_change_Leg2x(float x, float y, float z) {
return -(x + x0_2 + centerToSideLegs);}

float basis_change_Leg2y(float x, float y, float z) {
return -(y + y0_2);}


float basis_change_Leg3x(float x, float y, float z) {
return (x + x0_3 + centerToFrontLegsX)*cosf(3*Pi/4)-(y + y0_3 + centerToFrontLegsY)*sinf(3*Pi/4);}

float basis_change_Leg3y(float x, float y, float z) {
return (x + x0_3 + centerToFrontLegsX)*sinf(3*Pi/4)+(y + y0_3 + centerToFrontLegsY)*cosf(3*Pi/4);}


float basis_change_Leg4x(float x, float y, float z) {
return (x + x0_4 - centerToFrontLegsX)*cosf(Pi/4)-(y + y0_4 + centerToFrontLegsY)*sinf(Pi/4);}

float basis_change_Leg4y(float x, float y, float z) {
return (x + x0_4 - centerToFrontLegsX)*sinf(Pi/4)+(y + y0_4 + centerToFrontLegsY)*cosf(Pi/4);}


float basis_change_Leg5x(float x, float y, float z) {
return (x + x0_5 - centerToSideLegs);}

float basis_change_Leg5y(float x, float y, float z) {
return (y + y0_5);}


float basis_change_Leg6x(float x, float y, float z) {
return (x + x0_6 - centerToFrontLegsX)*cosf(-Pi/4)-(y + y0_6 - centerToFrontLegsY)*sinf(-Pi/4);}

float basis_change_Leg6y(float x, float y, float z) {
return (x + x0_6 - centerToFrontLegsX)*sinf(-Pi/4)+(y + y0_6 - centerToFrontLegsY)*cosf(-Pi/4);}
