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

#define coxa 56
#define femur 66
#define tibia 131
#define femurAngleAddition 0.2426
#define tibiaAngleAddition -3.1415/6
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85

double sinAngleToFrontLegs = centerToFrontLegsY / centerToFrontLegs;
double cosAngleToFrontLegs = centerToFrontLegsX / centerToFrontLegs;

void LegOneGoto(double x,double y,double z, int servospeed)
{
	double alpha;
	double beta;
	double gamma;
	double d;
	
	gamma = atan(y/x);
	d = sqrt(pow(sqrt(x*x + y*y)-coxa, 2) + z*z);
	beta = 3.1415 - acos((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	alpha = acos((femur*femur-tibia*tibia+d*d)/(2*femur*d))-asin(fabs(z)/d);
	
	servoGoto(8, gamma, servospeed);
	servoGoto(10, alpha + femurAngleAddition,servospeed);
	servoGoto(12, -beta + tibiaAngleAddition,servospeed);
}

void moveLeg1too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x + centerToFrontLegsX)*cos(-3*3.1415/4)-(y-centerToFrontLegsY)*sin(-3*3.1415/4);
	double b = (x + centerToFrontLegsX)*sin(-3*3.1415/4)+(y-centerToFrontLegsY)*cos(-3*3.1415/4);
	LegOneGoto(a, b, z, servospeed);
}



