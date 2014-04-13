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
#define frontLegDistance 200
#define femurAngleAddition 0.2426
#define tibiaAngleAddition -3.1415/6
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85

#define x0_1 -215 //standard x pos for leg 1
#define y0_1  275 //standard y pos for leg 1
#define x0_2  -340 //standard x pos for leg 2
#define y0_2  0 //standard y pos for leg 2
#define x0_3  -215 //standard x pos for leg 3
#define y0_3  -275 //standard y pos for leg 3
#define x0_4 215 //standard x pos for leg 4
#define y0_4  -275 //standard y pos for leg 4
#define x0_5  340 //standard x pos for leg 5
#define y0_5  0 //standard y pos for leg 5
#define x0_6  215 //standard x pos for leg 6
#define y0_6  275 //standard y pos for leg 6

#define Pi 3.14159265

double sinAngleToFrontLegs = centerToFrontLegsY / centerToFrontLegs;
double cosAngleToFrontLegs = centerToFrontLegsX / centerToFrontLegs;

double alpha;
double beta;
double gamma;
double d;



void LegGoto(double x,double y,double z, int servospeed, int side, int servo1, int servo2, int servo3)
{
	/*
	double alpha;
	double beta;
	double gamma;
	double d;
	*/
	gamma = atan(y/x);
	d = sqrt(pow(sqrt(x*x + y*y)-coxa, 2) + z*z);
	beta = 3.1415 - acos((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	alpha = acos((femur*femur-tibia*tibia+d*d)/(2*femur*d))-asin(fabs(z)/d);
	
	servoGoto(servo1, gamma, servospeed);
	_delay_ms(1);
	servoGoto(servo2, side*(alpha + femurAngleAddition),servospeed);
	_delay_ms(1);
	servoGoto(servo3, side*(-beta + tibiaAngleAddition),servospeed);
}

void moveLeg1too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x + centerToFrontLegsX)*cos(-3*3.1415/4)-(y-centerToFrontLegsY)*sin(-3*3.1415/4);
	double b = (x + centerToFrontLegsX)*sin(-3*3.1415/4)+(y-centerToFrontLegsY)*cos(-3*3.1415/4);
	LegGoto(a, b, z, servospeed,1,8,10,12);
}

void moveLeg2too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x + centerToSideLegs)*cos(3.1415)-(y)*sin(-3.1415);
	double b = (x + centerToSideLegs)*sin(-3.1415)+(y)*cos(3.1415);
	LegGoto(a, b, z, servospeed,1,14,16,18);
}

void moveLeg3too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x + centerToFrontLegsX)*cos(-5*3.1415/4)-(y+centerToFrontLegsY)*sin(-5*3.1415/4);
	double b = (x + centerToFrontLegsX)*sin(-5*3.1415/4)+(y+centerToFrontLegsY)*cos(-5*3.1415/4);
	LegGoto(a, b, z, servospeed,1,2,4,6);
}

void moveLeg4too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x - centerToFrontLegsX)*cos(3.1415/4)-(y+centerToFrontLegsY)*sin(3.1415/4);
	double b = (x - centerToFrontLegsX)*sin(3.1415/4)+(y+centerToFrontLegsY)*cos(3.1415/4);
	LegGoto(a, b, z, servospeed,-1,1,3,5);
}

void moveLeg5too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x - centerToSideLegs)*cos(0)-(y)*sin(0);
	double b = (x - centerToSideLegs)*sin(0)+(y)*cos(0);
	LegGoto(a, b, z, servospeed,-1,13,15,17);
}

void moveLeg6too(double x, double y, double z, int servospeed) //Help function to describe position of leg in standard base x,y,z
{
	double a = (x - centerToFrontLegsX)*cos(-3.1415/4)-(y-centerToFrontLegsY)*sin(-3.1415/4);
	double b = (x - centerToFrontLegsX)*sin(-3.1415/4)+(y-centerToFrontLegsY)*cos(-3.1415/4);
	LegGoto(a, b, z, servospeed,-1,7,9,11);
}


//New functions by Tobias
void Calc_d(double x,double y,double z)
{
	d = sqrt(pow(sqrt(x*x + y*y)-coxa, 2) + z*z);
}

double Calc_gamma(double x,double y)
{
gamma = atan(y/x);
return gamma;
}

double Calc_Beta(double x,double y,double z)
{
	beta = Pi - acos((femur*femur+tibia*tibia-d*d)/(2*femur*tibia));
	return beta;
}

double Calc_Alpha(double x,double y,double z)
{
	alpha = acos((femur*femur-tibia*tibia+d*d)/(2*femur*d))+asin(z/d);
	return alpha;
}


double basis_change_Leg1x(double x, double y, double z) {
return (x + x0_1 + centerToFrontLegsX)*cos(-3*Pi/4)-(y + y0_1 - centerToFrontLegsY)*sin(-3*Pi/4);}

double basis_change_Leg1y(double x, double y, double z) {
return (x + x0_1 + centerToFrontLegsX)*sin(-3*Pi/4)+(y + y0_1 - centerToFrontLegsY)*cos(-3*Pi/4);}


double basis_change_Leg2x(double x, double y, double z) {
return -(x + x0_2 + centerToSideLegs);}

double basis_change_Leg2y(double x, double y, double z) {
return -(y + y0_2);}


double basis_change_Leg3x(double x, double y, double z) {
return (x + x0_3 + centerToFrontLegsX)*cos(3*Pi/4)-(y + y0_3 + centerToFrontLegsY)*sin(3*Pi/4);}

double basis_change_Leg3y(double x, double y, double z) {
return (x + x0_3 + centerToFrontLegsX)*sin(3*Pi/4)+(y + y0_3 + centerToFrontLegsY)*cos(3*Pi/4);}


double basis_change_Leg4x(double x, double y, double z) {
return (x + x0_4 - centerToFrontLegsX)*cos(Pi/4)-(y + y0_4 + centerToFrontLegsY)*sin(Pi/4);}

double basis_change_Leg4y(double x, double y, double z) {
return (x + x0_4 - centerToFrontLegsX)*sin(Pi/4)+(y + y0_4 + centerToFrontLegsY)*cos(Pi/4);}


double basis_change_Leg5x(double x, double y, double z) {
return (x + x0_5 - centerToSideLegs);}

double basis_change_Leg5y(double x, double y, double z) {
return (y + y0_5);}


double basis_change_Leg6x(double x, double y, double z) {
return (x + x0_6 - centerToFrontLegsX)*cos(-Pi/4)-(y + y0_6 - centerToFrontLegsY)*sin(-Pi/4);}

double basis_change_Leg6y(double x, double y, double z) {
return (x + x0_6 - centerToFrontLegsX)*sin(-Pi/4)+(y + y0_6 - centerToFrontLegsY)*cos(-Pi/4);}
