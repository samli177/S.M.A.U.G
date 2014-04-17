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

int speed;
float iterations;

struct LegData 
{
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
//Statusvariabler

//1 om benet ska lyftas, -1 om det ska vara i marken
float leg1Lift;
float leg2Lift;
float leg3Lift;
float leg4Lift;
float leg5Lift;
float leg6Lift;

/*
#define Gamma0 (float) 0
#define Beta0 (float) 1.875
#define Alpha0 (float) 0.1406
*/

float z = z0;

float direction;
float rotation;
float speedf;

//Position där föregående steg slutade
float leg1PrevPosx;
float leg1PrevPosy;

float leg2PrevPosx;
float leg2PrevPosy;

float leg3PrevPosx;
float leg3PrevPosy;

float leg4PrevPosx;
float leg4PrevPosy;

float leg5PrevPosx;
float leg5PrevPosy;

float leg6PrevPosx;
float leg6PrevPosy;


//Position där nuvarande steg ska sluta
float leg1NewPosx;
float leg1NewPosy;

float leg2NewPosx;
float leg2NewPosy;

float leg3NewPosx;
float leg3NewPosy;

float leg4NewPosx;
float leg4NewPosy;

float leg5NewPosx;
float leg5NewPosy;

float leg6NewPosx;
float leg6NewPosy;

float xDirection;
float yDirection;


	
float xRot1;
float yRot1;

float xRot2;
float yRot2;

float xRot3;
float yRot3;

float xRot4;
float yRot4;

float xRot5;
float yRot5;

float xRot6;
float yRot6;

float stepMax;
float stepTest;
float stepScaling;




float leg1NewPosxWB;
float leg1NewPosyWB;
float leg1NewPosz;
float leg1NewPosx;
float leg1NewPosy;
float leg1PrevPosx;
float leg1PrevPosy;

float leg2NewPosxWB;
float leg2NewPosyWB;
float leg2NewPosz;
float leg2NewPosx;
float leg2NewPosy;
float leg2PrevPosx;
float leg2PrevPosy;

float leg3NewPosxWB;
float leg3NewPosyWB;
float leg3NewPosz;
float leg3NewPosx;
float leg3NewPosy;
float leg3PrevPosx;
float leg3PrevPosy;

float leg4NewPosxWB;
float leg4NewPosyWB;
float leg4NewPosz;
float leg4NewPosx;
float leg4NewPosy;
float leg4PrevPosx;
float leg4PrevPosy;

float leg5NewPosxWB;
float leg5NewPosyWB;
float leg5NewPosz;
float leg5NewPosx;
float leg5NewPosy;
float leg5PrevPosx;
float leg5PrevPosy;

float leg6NewPosxWB;
float leg6NewPosyWB;
float leg6NewPosz;
float leg6NewPosx;
float leg6NewPosy;
float leg6PrevPosx;
float leg6PrevPosy;

	//Vinklar där föregående steg slutade
	float leg1PrevAngleGamma;
	float leg1PrevAngleBeta;
	float leg1PrevAngleAlpha;

	float leg2PrevAngleGamma;
	float leg2PrevAngleBeta;
	float leg2PrevAngleAlpha;
	
	float leg3PrevAngleGamma;
	float leg3PrevAngleBeta;
	float leg3PrevAngleAlpha;
	
	float leg4PrevAngleGamma;
	float leg4PrevAngleBeta;
	float leg4PrevAngleAlpha;
	
	float leg5PrevAngleGamma;
	float leg5PrevAngleBeta;
	float leg5PrevAngleAlpha;
	
	float leg6PrevAngleGamma;
	float leg6PrevAngleBeta;
	float leg6PrevAngleAlpha;
	
	//Benkoordinater som nuvarande steget ska gå till
	float leg1NewAngleGamma;
	float leg1NewAngleBeta;
	float leg1NewAngleAlpha;
	
	float leg2NewAngleGamma;
	float leg2NewAngleBeta;
	float leg2NewAngleAlpha;
	
	float leg3NewAngleGamma;
	float leg3NewAngleBeta;
	float leg3NewAngleAlpha;
	
	float leg4NewAngleGamma;
	float leg4NewAngleBeta;
	float leg4NewAngleAlpha;
	
	float leg5NewAngleGamma;
	float leg5NewAngleBeta;
	float leg5NewAngleAlpha;
	
	float leg6NewAngleGamma;
	float leg6NewAngleBeta;
	float leg6NewAngleAlpha;


//Längd från origo till standardposition för ben 1,3,4,6
float stdLength;


//Max möjliga steglängd från grundpositionen
float maxStepLength;

//Middle-postioner
float leg1NewMiddlePosx;
float leg1NewMiddlePosy;

float leg2NewMiddlePosx;
float leg2NewMiddlePosy;

float leg3NewMiddlePosx;
float leg3NewMiddlePosy;

float leg4NewMiddlePosx;
float leg4NewMiddlePosy;

float leg5NewMiddlePosx;
float leg5NewMiddlePosy;

float leg6NewMiddlePosx;
float leg6NewMiddlePosy;


//Middle-vinklar
float leg1NewMiddleAngleGamma;
float leg1NewMiddleAngleBeta;
float leg1NewMiddleAngleAlpha;

float leg2NewMiddleAngleGamma;
float leg2NewMiddleAngleBeta;
float leg2NewMiddleAngleAlpha;

float leg3NewMiddleAngleGamma;
float leg3NewMiddleAngleBeta;
float leg3NewMiddleAngleAlpha;

float leg4NewMiddleAngleGamma;
float leg4NewMiddleAngleBeta;
float leg4NewMiddleAngleAlpha;

float leg5NewMiddleAngleGamma;
float leg5NewMiddleAngleBeta;
float leg5NewMiddleAngleAlpha;

float leg6NewMiddleAngleGamma;
float leg6NewMiddleAngleBeta;
float leg6NewMiddleAngleAlpha;

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
	speed = 300;
	iterations = 5;
	maxStepLength = 100;
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


//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
void move_robot(int dir, int rot, int spd)
{
	direction = (float)dir*2;
	rotation = (float)rot / 50 -1;
	speedf = (float)spd;
	if (speedf != 0 || rotation != 50)
	{
	step_start(&leg1);
	step_start(&leg2);
	step_start(&leg3);
	step_start(&leg4);
	step_start(&leg5);
	step_start(&leg6);

	//x och y riktning för förflyttning, skalad med hastigheten
	xDirection = -sinf(direction * pi / 90) * speedf / 100;
	yDirection = cosf(direction * pi / 90) * speedf / 100;

	

	//x och y riktning för rotation, skalad med rotationshastighet
	
	
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


	//Addera förflyttningarna och sätter Step_max till längsta stegets längd
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


	//Stegskalning för att inte alltid ta max längd på steg;
	stepScaling = fmaxf(speedf/100, fabsf(rotation/50-1));


	//Uppdatera vinklar för nya steget


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

	}
}

void move_leg(struct LegData* leg, float n)
{
	tempx = leg->prevPosx + n*(leg->newPosx - leg->prevPosx)/iterations;
	tempy = leg->prevPosy + n*(leg->newPosy - leg->prevPosy)/iterations;
	tempz = z;
	calc_d(tempx, tempy, tempz);
	
	
}
void leg_motion()
{
	
	for(i = 0; i < (int)iterations; i++)
	{
		
	}
}

void Leg_motion()
{
	speed = 150;
	if (leg1Lift == -1)
	{
		servoBufferPosition(8,leg1NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(10,side1*(leg1NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(12,side1*(-leg1NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(8,leg1PrevAngleGamma + (leg1NewAngleGamma-leg1PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(10,side1*(leg1PrevAngleAlpha + (leg1NewAngleAlpha-leg1PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(12,side1*(-leg1PrevAngleBeta - (leg1NewAngleBeta-leg1PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
	_delay_ms(1);

	if (leg2Lift == -1)
	{
		servoBufferPosition(14,leg2NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(16,side2*(leg2NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(18,side2*(-leg2NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(14,leg2PrevAngleGamma + (leg2NewAngleGamma-leg2PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(16,side2*(leg2PrevAngleAlpha + (leg2NewAngleAlpha-leg2PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(18,side2*(-leg2PrevAngleBeta - (leg2NewAngleBeta-leg2PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
	_delay_ms(1);

	if (leg3Lift == -1)
	{
		servoBufferPosition(2,leg3NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(4,side3*(leg3NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(6,side3*(-leg3NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(2,leg3PrevAngleGamma + (leg3NewAngleGamma-leg3PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(4,side3*(leg3PrevAngleAlpha + (leg3NewAngleAlpha-leg3PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(6,side3*(-leg3PrevAngleBeta - (leg3NewAngleBeta-leg3PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
	_delay_ms(1);

	if (leg4Lift == -1)
	{
		servoBufferPosition(1,leg4NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(3,side4*(leg4NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(5,side4*(-leg4NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(1,leg4PrevAngleGamma + (leg4NewAngleGamma-leg4PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(3,side4*(leg4PrevAngleAlpha + (leg4NewAngleAlpha-leg4PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(5,side4*(-leg4PrevAngleBeta - (leg4NewAngleBeta-leg4PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
	_delay_ms(1);

	if (leg5Lift == -1)
	{
		servoBufferPosition(13,leg5NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(15,side5*(leg5NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(17,side5*(-leg5NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(13,leg5PrevAngleGamma + (leg5NewAngleGamma-leg5PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(15,side5*(leg5PrevAngleAlpha + (leg5NewAngleAlpha-leg5PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(17,side5*(-leg5PrevAngleBeta - (leg5NewAngleBeta-leg5PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
_delay_ms(1);

	if (leg6Lift == -1)
	{
		servoBufferPosition(7,leg6NewMiddleAngleGamma,speed);
		_delay_us(100);
		servoBufferPosition(9,side6*(leg6NewMiddleAngleAlpha + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(11,side6*(-leg6NewMiddleAngleBeta + tibiaAngleAddition),speed);
	}
	else
	{
		servoBufferPosition(7,leg6PrevAngleGamma + (leg6NewAngleGamma-leg6PrevAngleGamma)/2 ,speed);
		_delay_us(100);
		servoBufferPosition(9,side6*(leg6PrevAngleAlpha + (leg6NewAngleAlpha-leg6PrevAngleAlpha)/2 + 0.5 + femurAngleAddition),speed);
		_delay_us(100);
		servoBufferPosition(11,side6*(-leg6PrevAngleBeta - (leg6NewAngleBeta-leg6PrevAngleBeta)/2 + tibiaAngleAddition),speed);
	}
servoAction();
	_delay_ms(200);

	servoBufferPosition(8,leg1NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(10,side1*(leg1NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(12,side1*(-leg1NewAngleBeta + tibiaAngleAddition),speed);

	_delay_ms(1);

	servoBufferPosition(14,leg2NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(16,side2*(leg2NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(18,side2*(-leg2NewAngleBeta + tibiaAngleAddition),speed);

	_delay_ms(1);

	servoBufferPosition(2,leg3NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(4,side3*(leg3NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(6,side3*(-leg3NewAngleBeta + tibiaAngleAddition),speed);

	_delay_ms(1);

	servoBufferPosition(1,leg4NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(3,side4*(leg4NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(5,side4*(-leg4NewAngleBeta + tibiaAngleAddition),speed);

	_delay_ms(1);

	servoBufferPosition(13,leg5NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(15,side5*(leg5NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(17,side5*(-leg5NewAngleBeta + tibiaAngleAddition),speed);

	_delay_ms(1);

	servoBufferPosition(7,leg6NewAngleGamma,speed);
	_delay_us(100);
	servoBufferPosition(9,side6*(leg6NewAngleAlpha + femurAngleAddition),speed);
	_delay_us(100);
	servoBufferPosition(11,side6*(-leg6NewAngleBeta + tibiaAngleAddition),speed);
servoAction();
	_delay_ms(200);
}


int main(void)
{
	DDRD |= (1<<PORTD5); //init LED
	//servoTx;
	
	servoRetrunLevel(BROADCASTING_ID, 1); //turns off return packets
	
	sei();
	initServoSerial(); //Init servos
	USART_init();
	initvar();
	

	moveLeg1too(x0_1, y0_1, z0, speed);
	moveLeg2too(x0_2, y0_2, z0, speed);
	moveLeg3too(x0_3, y0_3, z0, speed);
	moveLeg4too(x0_4, y0_4, z0, speed);
	moveLeg5too(x0_5, y0_5, z0, speed);
	moveLeg6too(x0_6, y0_6, z0, speed);
	
	
	_delay_ms(5000);
	
	
    while(1)
    {
		//moveRobot(USART_getDirection(),USART_getRotation(),USART_getSpeed());
		move_robot(0,50,100);
		leg1Lift = -leg1Lift;
		leg2Lift = -leg2Lift;
		leg3Lift = -leg3Lift;
		leg4Lift = -leg4Lift;
		leg5Lift = -leg5Lift;
		leg6Lift = -leg6Lift;
		
		
		//USART_DecodeRxFIFO();

    }
}