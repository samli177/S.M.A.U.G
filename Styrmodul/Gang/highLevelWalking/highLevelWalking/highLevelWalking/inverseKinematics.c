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


