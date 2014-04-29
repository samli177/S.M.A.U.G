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
#include "highLevelWalking.h"
#include "usart.h"
#include "counter.h"

int speed;
float iterations;


struct LegData 
{
	float side;
	float lift;
	float xRot;
	float yRot;
	float newPosz;
	float newPosx;
	float newPosy;
	float prevPosx;
	float prevPosy;
	float prevAngleGamma;
	float prevAngleBeta;
	float prevAngleAlpha;
	float newAngleGamma;
	float newAngleBeta;
	float newAngleAlpha;
	float temp1AngleGamma;
	float temp1AngleBeta;
	float temp1AngleAlpha;
	float temp2AngleGamma;
	float temp2AngleBeta;
	float temp2AngleAlpha;
	int servoGamma;
	int servoBeta;
	int servoAlpha;
};

float newPosxWB;
float newPosyWB;

struct LegData leg1;
struct LegData leg2;
struct LegData leg3;
struct LegData leg4;
struct LegData leg5;
struct LegData leg6;

float speedAlpha;
float speedBeta;
float speedGamma;
float speedMultiplier;

float tempx;
float tempy;
float tempz;

float z = z0;

float direction;
float rotation;
float speedf;

float xDirection;
float yDirection;

float stepMax;
float stepTest;
float stepScaling;

//L�ngd fr�n origo till standardposition f�r ben 1,3,4,6
float stdLength;

//Max m�jliga stegl�ngd fr�n grundpositionen
float maxStepLength;


void init_struct(struct LegData* leg)
{
	leg->prevAngleAlpha = Alpha0;
	leg->prevAngleBeta = Beta0;
	leg->prevAngleGamma = Gamma0;
	
	leg->newAngleAlpha = Alpha0;
	leg->newAngleBeta = Beta0;
	leg->newAngleGamma = Gamma0;
	
	leg->newPosx = x0;
	leg->newPosy = y0;
	leg->newPosz = z0;
}

void initvar()
{
	speedMultiplier = 1500;
	speed = 300;
	iterations = 5;
	maxStepLength = 60;
	stdLength = sqrtf(x0_1*x0_1 + y0_1*y0_1);
	

	
	init_struct(&leg1);
	init_struct(&leg2);
	init_struct(&leg3);
	init_struct(&leg4);
	init_struct(&leg5);
	init_struct(&leg6);
	
	leg1.lift = 1;
	leg2.lift = -1;
	leg3.lift = 1;
	leg4.lift = -1;	
	leg5.lift = 1;
	leg6.lift = -1;
	
	leg1.side = 1;
	leg2.side = 1;
	leg3.side = 1;
	leg4.side = -1;
	leg5.side = -1;
	leg6.side = -1;
	
	leg1.servoAlpha = 10;
	leg1.servoBeta = 12;
	leg1.servoGamma = 8;
	
	leg2.servoAlpha = 16;
	leg2.servoBeta = 18;
	leg2.servoGamma = 14;
	
	leg3.servoAlpha = 4;
	leg3.servoBeta = 6;
	leg3.servoGamma = 2;
	
	leg4.servoAlpha = 3;
	leg4.servoBeta = 5;
	leg4.servoGamma = 1;
	
	leg5.servoAlpha = 15;
	leg5.servoBeta = 17;
	leg5.servoGamma = 13;
	
	leg6.servoAlpha = 9;
	leg6.servoBeta = 11;
	leg6.servoGamma = 7;
	
}

void step_start(struct LegData* leg)
{
	leg->prevPosx = leg->newPosx;
	leg->prevPosy = leg->newPosy;
	leg->prevAngleAlpha = leg->newAngleAlpha;
	leg->prevAngleBeta = leg->newAngleBeta;
	leg->prevAngleGamma = leg->newAngleGamma;
}

void step_part1_calculator(struct LegData* leg)
{
	newPosxWB = leg->lift * maxStepLength / stepMax * (xDirection + leg->xRot) * stepScaling;
	newPosyWB = leg->lift * maxStepLength / stepMax * (yDirection + leg->yRot) * stepScaling;
	leg->newPosz = z;
}

void step_part2_calculator(struct LegData* leg)
{
	calc_d(leg->newPosx, leg->newPosy, leg->newPosz);
	leg->newAngleGamma = get_gamma(leg->newPosx, leg->newPosy);
	leg->newAngleBeta = get_beta();
	leg->newAngleAlpha = get_alpha(leg->newPosz);
}


//Tar in styrkommandon (format � la Martin) och uppdaterar variabler f�r positionen
//av ben f�r f�rra steget och r�knar ut position f�r n�sta steg.
void move_robot(int dir, int rot, int spd)
{
	direction = (float)dir;
	rotation = (float)rot / 50 -1;
	speedf = (float)spd;
	if (speedf != 0 || rotation != 0)
	{
	step_start(&leg1);
	step_start(&leg2);
	step_start(&leg3);
	step_start(&leg4);
	step_start(&leg5);
	step_start(&leg6);
	
	leg1.lift = -leg1.lift;
	leg2.lift = -leg2.lift;
	leg3.lift = -leg3.lift;
	leg4.lift = -leg4.lift;
	leg5.lift = -leg5.lift;
	leg6.lift = -leg6.lift;

	//x och y riktning f�r f�rflyttning, skalad med hastigheten
	xDirection = -sinf(direction * pi / 45) * speedf / 100;
	yDirection = cosf(direction * pi / 45) * speedf / 100;

	

	//x och y riktning f�r rotation, skalad med rotationshastighet
	
	
	
	leg1.xRot = rotation * y0_1/ stdLength;
	leg1.yRot = rotation * x0_1 / stdLength;

	leg2.xRot = 0;
	leg2.yRot = rotation;

	leg3.xRot = rotation * (- y0_1 / stdLength);
	leg3.yRot = rotation * x0_1 / stdLength;

	leg4.xRot = rotation * (- y0_1 / stdLength);
	leg4.yRot = rotation * (- x0_1 / stdLength);

	leg5.xRot = 0;
	leg5.yRot = -rotation;

	leg6.xRot = rotation  * y0_1 / stdLength;
	leg6.yRot = rotation  * (- x0_1 / stdLength);


	//Addera f�rflyttningarna och s�tter Step_max till l�ngsta stegets l�ngd
	stepMax = powf((xDirection + leg1.xRot),2) + powf((yDirection + leg1.yRot),2);
	stepTest = powf((xDirection + leg2.xRot),2) + powf((yDirection + leg2.yRot),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + leg3.xRot),2) + powf((yDirection + leg3.yRot),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + leg4.xRot),2) + powf((yDirection + leg4.yRot),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + leg5.xRot),2) + powf((yDirection + leg5.yRot),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + leg6.xRot),2) + powf((yDirection + leg6.yRot),2);
	stepMax = fmaxf(stepMax, stepTest);
	
	stepMax =sqrtf(stepMax);


	//Stegskalning f�r att inte alltid ta max l�ngd p� steg;
	stepScaling = fmaxf(speedf/100, fabsf(rotation));


	//Uppdatera vinklar f�r nya steget


	step_part1_calculator(&leg1);
	leg1.newPosx = basis_change_leg1x(newPosxWB,newPosyWB);
	leg1.newPosy = basis_change_leg1y(newPosxWB,newPosyWB);
	step_part2_calculator(&leg1);
	
	step_part1_calculator(&leg2);
	leg2.newPosx = basis_change_leg2x(newPosxWB);
	leg2.newPosy = basis_change_leg2y(newPosyWB);
	step_part2_calculator(&leg2);
		
	step_part1_calculator(&leg3);
	leg3.newPosx = basis_change_leg3x(newPosxWB,newPosyWB);
	leg3.newPosy = basis_change_leg3y(newPosxWB,newPosyWB);
	step_part2_calculator(&leg3);
			
	step_part1_calculator(&leg4);
	leg4.newPosx = basis_change_leg4x(newPosxWB,newPosyWB);
	leg4.newPosy = basis_change_leg4y(newPosxWB,newPosyWB);
	step_part2_calculator(&leg4);
				
	step_part1_calculator(&leg5);
	leg5.newPosx = basis_change_leg5x(newPosxWB);
	leg5.newPosy = basis_change_leg5y(newPosyWB);
	step_part2_calculator(&leg5);
					
	step_part1_calculator(&leg6);
	leg6.newPosx = basis_change_leg6x(newPosxWB,newPosyWB);
	leg6.newPosy = basis_change_leg6y(newPosxWB,newPosyWB);
	step_part2_calculator(&leg6);

	leg_motion();
	}
}

void leg_motion_init()
{
	leg1.temp2AngleAlpha = leg1.prevAngleAlpha;
	leg1.temp2AngleBeta = leg1.prevAngleBeta;
	leg1.temp2AngleGamma = leg1.prevAngleGamma;
	
	leg2.temp2AngleAlpha = leg2.prevAngleAlpha;
	leg2.temp2AngleBeta = leg2.prevAngleBeta;
	leg2.temp2AngleGamma = leg2.prevAngleGamma;
	
	leg3.temp2AngleAlpha = leg3.prevAngleAlpha;
	leg3.temp2AngleBeta = leg3.prevAngleBeta;
	leg3.temp2AngleGamma = leg3.prevAngleGamma;
	
	leg4.temp2AngleAlpha = leg4.prevAngleAlpha;
	leg4.temp2AngleBeta = leg4.prevAngleBeta;
	leg4.temp2AngleGamma = leg4.prevAngleGamma;
	
	leg5.temp2AngleAlpha = leg5.prevAngleAlpha;
	leg5.temp2AngleBeta = leg5.prevAngleBeta;
	leg5.temp2AngleGamma = leg5.prevAngleGamma;
	
	leg6.temp2AngleAlpha = leg6.prevAngleAlpha;
	leg6.temp2AngleBeta = leg6.prevAngleBeta;
	leg6.temp2AngleGamma = leg6.prevAngleGamma;
}

void move_leg(struct LegData* leg, float n)
{
	if(leg->lift == 1 && n != iterations+1)
	{
		tempz = z + 30;
	}
	else
	{
		tempz = z;
	}
	if(n != 0 && n != iterations+1)
	{
		leg->temp1AngleGamma = leg->temp2AngleGamma;
		leg->temp1AngleBeta = leg->temp2AngleBeta;
		leg->temp1AngleAlpha = leg->temp2AngleAlpha;
		tempx = leg->prevPosx + n*(leg->newPosx - leg->prevPosx)/iterations;
		tempy = leg->prevPosy + n*(leg->newPosy - leg->prevPosy)/iterations;
	}
	else if (n == iterations+1)
	{
		tempx = leg->newPosx;
		tempy = leg->newPosy;
	}
	else
	{
		tempx = leg->prevPosx;
		tempy = leg->prevPosy;
	}
	calc_d(tempx, tempy, tempz);
	leg->temp2AngleGamma = get_gamma(tempx, tempy);
	leg->temp2AngleBeta = get_beta();
	leg->temp2AngleAlpha = get_alpha(tempz);
	speedAlpha = fabsf(leg->temp1AngleAlpha - leg->temp2AngleAlpha)*speedMultiplier;
	speedBeta = fabsf(leg->temp1AngleBeta - leg->temp2AngleBeta)*speedMultiplier;
	speedGamma = fabsf(leg->temp1AngleGamma - leg->temp2AngleGamma)*speedMultiplier;
	servoBufferPosition(leg->servoAlpha, leg->side *(leg->temp2AngleAlpha + femurAngleAddition),(int)speedAlpha);
	servoBufferPosition(leg->servoBeta,leg->side*(-leg->temp2AngleBeta + tibiaAngleAddition),(int)speedBeta);
	servoBufferPosition(leg->servoGamma, leg->temp2AngleGamma,(int)speedGamma);
}
void leg_motion()
{
	leg_motion_init();
	for(int i = 0; i <= (int)iterations+1; ++i)
	{
		move_leg(&leg1,i);
		move_leg(&leg2,i);
		move_leg(&leg3,i);
		move_leg(&leg4,i);
		move_leg(&leg5,i);
		move_leg(&leg6,i);
		servoAction();
		_delay_ms(40);
	}
}

void move_to_std()
{
	leg1.lift = -leg1.lift;
	leg2.lift = -leg2.lift;
	leg3.lift = -leg3.lift;
	leg4.lift = -leg4.lift;
	leg5.lift = -leg5.lift;
	leg6.lift = -leg6.lift;
	
	step_start(&leg1);
	step_start(&leg2);
	step_start(&leg3);
	step_start(&leg4);
	step_start(&leg5);
	step_start(&leg6);
	
	if(leg1.lift == -1)
	{
		leg1.newPosx = x0;
		leg1.newPosy = y0;
		step_part2_calculator(&leg1);
	}
	
	if(leg2.lift == -1)
	{
		leg2.newPosx = x0;
		leg2.newPosy = y0;
		step_part2_calculator(&leg2);
	}
	
	if(leg3.lift == -1)
	{
		leg3.newPosx = x0;
		leg3.newPosy = y0;
		step_part2_calculator(&leg3);
	}
	
	if(leg4.lift == -1)
	{
		leg4.newPosx = x0;
		leg4.newPosy = y0;
		step_part2_calculator(&leg4);
	}
	
	if(leg5.lift == -1)
	{
		leg5.newPosx = x0;
		leg5.newPosy = y0;
		step_part2_calculator(&leg5);
	}
	
	if(leg6.lift == -1)
	{
		leg6.newPosx = x0;
		leg6.newPosy = y0;
		step_part2_calculator(&leg6);
	}
	
	leg_motion();

	
}

