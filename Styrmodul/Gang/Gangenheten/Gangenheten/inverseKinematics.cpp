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

float x0_1 = -100/sqrt2-61.85; //standard x pos for leg 1
float y0_1 = 100/sqrt2+120; //standard y pos for leg 1
float x0_2 = -100-100; //standard x pos for leg 2
float y0_2 = 0; //standard y pos for leg 2
float x0_3 = (-100/sqrt2-61.85); //standard x pos for leg 3
float y0_3 = (-100/sqrt2-120); //standard y pos for leg 3
float x0_4 = (100/sqrt2+61.85); //standard x pos for leg 4
float y0_4 = (-100/sqrt2-120); //standard y pos for leg 4
float x0_5 = (100+100); //standard x pos for leg 5
float y0_5 = 0; //standard y pos for leg 5
float x0_6 = (100/sqrt2+61.85); //standard x pos for leg 6
float y0_6 = (100/sqrt2+120); //standard y pos for leg 6

float x0 = 100;
float y0 = 0;
float z0 = -120;

void height_change_x0(float new_z)
{
	x0 = 100 + (new_z - z0);
}

void height_change_leg1(float new_z)
{
	x0_1 = -(120 + (new_z - z0))/sqrt2-61.85; 
	y0_1 = (120 + (new_z - z0))/sqrt2+120;
}

void height_change_leg2(float new_z)
{
	x0_2 = -(120 + (new_z - z0))-100;
}

void height_change_leg3(float new_z)
{
	x0_3 = -(120 + (new_z - z0))/sqrt2-61.85;
	y0_3 = -(120 + (new_z - z0))/sqrt2-120; 
}

void height_change_leg4(float new_z)
{
	x0_4 = (120 + (new_z - z0))/sqrt2+61.85;
	y0_4 = -(120 + (new_z - z0))/sqrt2-120; 
}

void height_change_leg5(float new_z)
{
	x0_5 = (120 + (new_z - z0))+100;
}

void height_change_leg6(float new_z)
{
	x0_6 = (120 + (new_z - z0))/sqrt2+61.85;
	y0_6 = (120 + (new_z - z0))/sqrt2+120;
}

void height_change_all(float new_z)
{
	height_change_x0(new_z);
	height_change_leg1(new_z);
	height_change_leg2(new_z);
	height_change_leg3(new_z);
	height_change_leg4(new_z);
	height_change_leg5(new_z);
	height_change_leg6(new_z);

}

float get_x0_1()
{
	return x0_1;
}

float get_y0_1()
{
	return y0_1;
}

float get_x0()
{
	return x0;
}

float get_y0()
{
	return y0;
}

float get_z0()
{
	return z0;
}

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