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

/*
#define pi (float) 3.14159265
#define sqrt2 (float) 1.41421356

#define x0_1 (float) (-100/sqrt2-61.85) //standard x pos for leg 1
#define y0_1 (float) (100/sqrt2+120) //standard y pos for leg 1
#define x0_2 (float) (-100-100) //standard x pos for leg 2
#define y0_2 (float) 0 //standard y pos for leg 2
#define x0_3 (float) (-100/sqrt2-61.85) //standard x pos for leg 3
#define y0_3 (float) (-100/sqrt2-120) //standard y pos for leg 3
#define x0_4 (float) (100/sqrt2+61.85) //standard x pos for leg 4
#define y0_4 (float) (-100/sqrt2-120) //standard y pos for leg 4
#define x0_5 (float) (100+100) //standard x pos for leg 5
#define y0_5 (float) 0 //standard y pos for leg 5
#define x0_6 (float) (100/sqrt2+61.85) //standard x pos for leg 6
#define y0_6 (float) (100/sqrt2+120) //standard y pos for leg 6
#define x0 (float) 100
#define y0 (float) 0
#define z0 (float) -120
*/
int speed;

/*
#define side1 (float)1
#define side2 (float)1
#define side3 (float)1
#define side4 (float)(-1)
#define side5 (float)(-1)
#define side6 (float)(-1)
*/

//#define femurAngleAddition (float)0.231 //0.2426
//#define tibiaAngleAddition (float)0.812 //(-3.1415/6)



/*
#define frontLegDistanfce (float)200
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85
*/

//Jonas function for robot movement

/*



float z0 = -120; //120 fungerar
int speed = 200;

void moveRobot(float direction,float distanfce, float rotation, float z, int servoSpeed, float rotationX, float rotationY)
{
	float sinfrotation = sinf(rotation);
	float cosfrotation = cosf(rotation);
	float sinfdirection = sinf(direction);
	float cosfdirection = cosf(direction);
	float sinfaroundx = 0;//sinf(rotationX);
	float sinfaroundy = 0;//sinf(rotationY);
	
	//First state-------------------
	servoBufferPosition(4, 3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoBufferPosition(10, 3.1415/2, servoSpeed);
	_delay_ms(5);
	servoBufferPosition(15, -3.1415/2, servoSpeed);*/
	/*
	moveLeg1too(x0_1*cosfrotation-y0_1*sinfrotation-sinfdirection*distanfce, y0_1*cosfrotation-x0_1*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY-40), servoSpeed);
	moveLeg3too(x0_3*cosfrotation-y0_3*sinfrotation-sinfdirection*distanfce, y0_3*cosfrotation-x0_3*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY-40), servoSpeed);
	moveLeg5too(x0_5*cosfrotation-y0_5*sinfrotation-sinfdirection*distanfce, y0_5*cosfrotation+x0_5*sinfrotation+cosfdirection*distanfce, -(z-centerToSideLegs*sinfaroundy-40), servoSpeed);
	_delay_ms(200);
	moveLeg1too(x0_1*cosfrotation-y0_1*sinfrotation-sinfdirection*distanfce, y0_1*cosfrotation-x0_1*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg3too(x0_3*cosfrotation-y0_3*sinfrotation-sinfdirection*distanfce, y0_3*cosfrotation-x0_3*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg5too(x0_5*cosfrotation-y0_5*sinfrotation-sinfdirection*distanfce, y0_5*cosfrotation+x0_5*sinfrotation+cosfdirection*distanfce, -(z-centerToSideLegs*sinfaroundy), servoSpeed);
	moveLeg6too(x0_6, y0_6, -(z-sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg4too(x0_4, y0_4, -(z-sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg2too(x0_2, y0_2, -(z+centerToSideLegs*sinfaroundy), servoSpeed);	
	_delay_ms(300);
		*/
	//Second state-------------------
	/*servoBufferPosition(3, -3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoBufferPosition(9, -3.1415/2, servoSpeed);
	_delay_ms(5);
	servoBufferPosition(16, 3.1415/2, servoSpeed);*/
	/*
	moveLeg2too(x0_2*cosfrotation-y0_2*sinfrotation-sinfdirection*distanfce, y0_2*cosfrotation+x0_2*sinfrotation+cosfdirection*distanfce, -(z+centerToSideLegs*sinfaroundy-40), servoSpeed);
	moveLeg4too(x0_4*cosfrotation-y0_4*sinfrotation-sinfdirection*distanfce, y0_4*cosfrotation-x0_4*sinfrotation+cosfdirection*distanfce, -(z-sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY-40), servoSpeed);
	moveLeg6too(x0_6*cosfrotation-y0_6*sinfrotation-sinfdirection*distanfce, y0_6*cosfrotation-x0_6*sinfrotation+cosfdirection*distanfce, -(z-sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY-40), servoSpeed);
	_delay_ms(200);
	moveLeg2too(x0_2*cosfrotation-y0_2*sinfrotation-sinfdirection*distanfce, y0_2*cosfrotation+x0_2*sinfrotation+cosfdirection*distanfce, -(z+centerToSideLegs*sinfaroundy), servoSpeed);
	moveLeg4too(x0_4*cosfrotation-y0_4*sinfrotation-sinfdirection*distanfce, y0_4*cosfrotation-x0_4*sinfrotation+cosfdirection*distanfce, -(z-sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg6too(x0_6*cosfrotation-y0_6*sinfrotation-sinfdirection*distanfce, y0_6*cosfrotation-x0_6*sinfrotation+cosfdirection*distanfce, -(z-sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg1too(x0_1, y0_1, -(z+sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg3too(x0_3, y0_3, -(z+sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg5too(x0_5, y0_5, -(z-centerToSideLegs*sinfaroundy), servoSpeed);
	_delay_ms(300);

	//Third state-------------------

	
	
}

*/
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
float y_rot3;

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

void initvar()
{
	speed = 300;
	maxStepLength = 100;
	stdLength = sqrtf(x0_1*x0_1 + y0_1*y0_1);
	
	leg1Lift = 1;
	leg2Lift = -1;
	leg3Lift = 1;
	leg4Lift = -1;	
	leg5Lift = 1;
	leg6Lift = -1;
	
	//Vinklar där föregående steg slutade
	leg1PrevAngleGamma = Gamma0;
	leg1PrevAngleBeta = Beta0;
	leg1PrevAngleAlpha = Alpha0;

	leg2PrevAngleGamma = Gamma0;
	leg2PrevAngleBeta = Beta0;
	leg2PrevAngleAlpha = Alpha0;
	
	leg3PrevAngleGamma = Gamma0;
	leg3PrevAngleBeta = Beta0;
	leg3PrevAngleAlpha = Alpha0;
	
	leg4PrevAngleGamma = Gamma0;
	leg4PrevAngleBeta = Beta0;
	leg4PrevAngleAlpha = Alpha0;
	
	leg5PrevAngleGamma = Gamma0;
	leg5PrevAngleBeta = Beta0;
	leg5PrevAngleAlpha = Alpha0;
	
	leg6PrevAngleGamma = Gamma0;
	leg6PrevAngleBeta = Beta0;
	leg6PrevAngleAlpha = Alpha0;
	
	//Benkoordinater som nuvarande steget ska gå till
	leg1NewAngleGamma = Gamma0;
	leg1NewAngleBeta = Beta0;
	leg1NewAngleAlpha = Alpha0;
	
	leg2NewAngleGamma = Gamma0;
	leg2NewAngleBeta = Beta0;
	leg2NewAngleAlpha = Alpha0;
	
	leg3NewAngleGamma = Gamma0;
	leg3NewAngleBeta = Beta0;
	leg3NewAngleAlpha = Alpha0;
	
	leg4NewAngleGamma = Gamma0;
	leg4NewAngleBeta = Beta0;
	leg4NewAngleAlpha = Alpha0;
	
	leg5NewAngleGamma = Gamma0;
	leg5NewAngleBeta = Beta0;
	leg5NewAngleAlpha = Alpha0;
	
	leg6NewAngleGamma = Gamma0;
	leg6NewAngleBeta = Beta0;
	leg6NewAngleAlpha = Alpha0;
	
	leg1NewPosx = x0;
	leg1NewPosy = y0;
	leg1NewPosz = z0;
	
	leg2NewPosx = x0;
	leg2NewPosy = y0;
	leg2NewPosz = z0;
	
	leg3NewPosx = x0;
	leg3NewPosy = y0;
	leg3NewPosz = z0;
	
	leg4NewPosx = x0;
	leg4NewPosy = y0;
	leg4NewPosz = z0;
	
	leg5NewPosx = x0;
	leg5NewPosy = y0;
	leg5NewPosz = z0;
	
	leg6NewPosx = x0;
	leg6NewPosy = y0;
	leg6NewPosz = z0;
}
//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
void move_robot(int dir, int rot, int spd)
{
	direction = (float)dir*2;
	rotation = (float)rot;
	speedf = (float)spd;
	if (speedf != 0 || rotation != 50)
	{
		

	leg1PrevPosx = leg1NewPosx;
	leg1PrevPosy = leg1NewPosy;
	
	leg2PrevPosx = leg2NewPosx;
	leg2PrevPosy = leg2NewPosy;
	
	leg3PrevPosx = leg3NewPosx;
	leg3PrevPosy = leg3NewPosy;
	
	leg4PrevPosx = leg4NewPosx;
	leg4PrevPosy = leg4NewPosy;
	
	leg5PrevPosx = leg5NewPosx;
	leg5PrevPosy = leg5NewPosy;
	
	leg6PrevPosx = leg6NewPosx;
	leg6PrevPosy = leg6NewPosy;
	
	
	//Nytt steg ska räknas ut. Nya stegets vinklar sätts till föregående stegets vinklar.
	leg1PrevAngleGamma = leg1NewAngleGamma;
	leg1PrevAngleBeta = leg1NewAngleBeta;
	leg1PrevAngleAlpha = leg1NewAngleAlpha;

	leg2PrevAngleGamma = leg2NewAngleGamma;
	leg2PrevAngleBeta = leg2NewAngleBeta;
	leg2PrevAngleAlpha = leg2NewAngleAlpha;

	leg3PrevAngleGamma = leg3NewAngleGamma;
	leg3PrevAngleBeta = leg3NewAngleBeta;
	leg3PrevAngleAlpha = leg3NewAngleAlpha;

	leg4PrevAngleGamma = leg4NewAngleGamma;
	leg4PrevAngleBeta = leg4NewAngleBeta;
	leg4PrevAngleAlpha = leg4NewAngleAlpha;

	leg5PrevAngleGamma = leg5NewAngleGamma;
	leg5PrevAngleBeta = leg5NewAngleBeta;
	leg5PrevAngleAlpha = leg5NewAngleAlpha;

	leg6PrevAngleGamma = leg6NewAngleGamma;
	leg6PrevAngleBeta = leg6NewAngleBeta;
	leg6PrevAngleAlpha = leg6NewAngleAlpha;


	//x och y riktning för förflyttning, skalad med hastigheten
	xDirection = -sinf(direction * pi / 90) * speedf / 100;
	yDirection = cosf(direction * pi / 90) * speedf / 100;


	//x och y riktning för rotation, skalad med rotationshastighet
	xRot1 = (rotation / 50 -1) * y0_1/ stdLength;
	yRot1 = (rotation / 50 -1) * x0_1 / stdLength;

	xRot2 = 0;
	yRot2 = rotation / 50 - 1;

	xRot3 = (rotation / 50 -1) * (- y0_1 / stdLength);
	y_rot3 = (rotation / 50 -1) * x0_1 / stdLength;

	xRot4 = (rotation / 50 -1) * (- y0_1 / stdLength);
	yRot4 = (rotation / 50 -1) * (- x0_1 / stdLength);

	xRot5 = 0;
	yRot5 = -rotation / 50 + 1;

	xRot6 = (rotation / 50 -1) * y0_1 / stdLength;
	yRot6 = (rotation / 50 -1) * (- x0_1 / stdLength);


	//Addera förflyttningarna och sätter Step_max till längsta stegets längd
	stepMax = powf((xDirection + xRot1),2) + powf((yDirection + yRot1),2);
	stepTest = powf((xDirection + xRot2),2) + powf((yDirection + yRot2),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + xRot3),2) + powf((yDirection + y_rot3),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + xRot4),2) + powf((yDirection + yRot4),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + xRot5),2) + powf((yDirection + yRot5),2);
	stepMax = fmaxf(stepMax, stepTest);

	stepTest = powf((xDirection + xRot6),2) + powf((yDirection + yRot6),2);
	stepMax = fmaxf(stepMax, stepTest);
	
	stepMax =sqrtf(stepMax);


	//Stegskalning för att inte alltid ta max längd på steg;
	stepScaling = fmaxf(speedf/100, fabsf(rotation/50-1));


	//Uppdatera vinklar för nya steget


	leg1NewPosxWB = leg1Lift * maxStepLength / stepMax * (xDirection + xRot1) * stepScaling;
	leg1NewPosyWB = leg1Lift * maxStepLength / stepMax * (yDirection + yRot1) * stepScaling;
	leg1NewPosz = z;
	leg1NewPosx = basis_change_leg1x(leg1NewPosxWB,leg1NewPosyWB);
	leg1NewPosy = basis_change_leg1y(leg1NewPosxWB,leg1NewPosyWB);
	calc_d(leg1NewPosx, leg1NewPosy, leg1NewPosz);
	leg1NewAngleGamma = get_gamma(leg1NewPosx, leg1NewPosy);
	leg1NewAngleBeta = get_beta();
	leg1NewAngleAlpha = get_alpha(leg1NewPosz);


	leg2NewPosxWB = leg2Lift * maxStepLength / stepMax * (xDirection + xRot2) * stepScaling;
	leg2NewPosyWB = leg2Lift * maxStepLength / stepMax * (yDirection + yRot2) * stepScaling;
	leg2NewPosz = z;
	leg2NewPosx = basis_change_leg2x(leg2NewPosxWB);
	leg2NewPosy = basis_change_leg2y(leg2NewPosyWB);
	calc_d(leg2NewPosx, leg2NewPosy, leg2NewPosz);
	leg2NewAngleGamma = get_gamma(leg2NewPosx, leg2NewPosy);
	leg2NewAngleBeta = get_beta();
	leg2NewAngleAlpha = get_alpha(leg2NewPosz);


	leg3NewPosxWB = leg3Lift * maxStepLength / stepMax * (xDirection + xRot3) * stepScaling;
	leg3NewPosyWB = leg3Lift * maxStepLength / stepMax * (yDirection + y_rot3) * stepScaling;
	leg3NewPosz = z;
	leg3NewPosx = basis_change_leg3x(leg3NewPosxWB,leg3NewPosyWB);
	leg3NewPosy = basis_change_leg3y(leg3NewPosxWB,leg3NewPosyWB);
	calc_d(leg3NewPosx, leg3NewPosy, leg3NewPosz);
	leg3NewAngleGamma = get_gamma(leg3NewPosx, leg3NewPosy);
	leg3NewAngleBeta = get_beta();
	leg3NewAngleAlpha = get_alpha(leg3NewPosz);


	leg4NewPosxWB = leg4Lift * maxStepLength / stepMax * (xDirection + xRot4) * stepScaling;
	leg4NewPosyWB = leg4Lift * maxStepLength / stepMax * (yDirection + yRot4) * stepScaling;
	leg4NewPosz = z;
	leg4NewPosx = basis_change_leg4x(leg4NewPosxWB,leg4NewPosyWB);
	leg4NewPosy = basis_change_leg4y(leg4NewPosxWB,leg4NewPosyWB);
	calc_d(leg4NewPosx, leg4NewPosy, leg4NewPosz);
	leg4NewAngleGamma = get_gamma(leg4NewPosx, leg4NewPosy);
	leg4NewAngleBeta = get_beta();
	leg4NewAngleAlpha = get_alpha(leg4NewPosz);


	leg5NewPosxWB = leg5Lift * maxStepLength / stepMax * (xDirection + xRot5) * stepScaling;
	leg5NewPosyWB = leg5Lift * maxStepLength / stepMax * (yDirection + yRot5) * stepScaling;
	leg5NewPosz = z;
	leg5NewPosx = basis_change_leg5x(leg5NewPosxWB);
	leg5NewPosy = basis_change_leg5y(leg5NewPosyWB);
	calc_d(leg5NewPosx, leg5NewPosy, leg5NewPosz);
	leg5NewAngleGamma = get_gamma(leg5NewPosx, leg5NewPosy);
	leg5NewAngleBeta = get_beta();
	leg5NewAngleAlpha = get_alpha(leg5NewPosz);


	leg6NewPosxWB = leg6Lift * maxStepLength / stepMax * (xDirection + xRot6) * stepScaling;
	leg6NewPosyWB = leg6Lift * maxStepLength / stepMax * (yDirection + yRot6) * stepScaling;
	leg6NewPosz = z;
	leg6NewPosx = basis_change_leg6x(leg6NewPosxWB,leg6NewPosyWB);
	leg6NewPosy = basis_change_leg6y(leg6NewPosxWB,leg6NewPosyWB);
	calc_d(leg6NewPosx, leg6NewPosy, leg6NewPosz);
	leg6NewAngleGamma = get_gamma(leg6NewPosx, leg6NewPosy);
	leg6NewAngleBeta = get_beta();
	leg6NewAngleAlpha = get_alpha(leg6NewPosz);


	if(leg1Lift == -1)
	{
		leg1NewMiddlePosx = leg1PrevPosx + (leg1NewPosx - leg1PrevPosx)/2;
		leg1NewMiddlePosy = leg1PrevPosy + (leg1NewPosy - leg1PrevPosy)/2;
		calc_d(leg1NewMiddlePosx, leg1NewMiddlePosy, z);
		leg1NewMiddleAngleGamma = get_gamma(leg1NewMiddlePosx, leg1NewMiddlePosy);
		leg1NewMiddleAngleBeta = get_beta();
		leg1NewMiddleAngleAlpha = get_alpha(leg1NewPosz);
	}
	
	if(leg2Lift == -1)
	{
		leg2NewMiddlePosx = leg2PrevPosx + (leg2NewPosx - leg2PrevPosx)/2;
		leg2NewMiddlePosy = leg2PrevPosy + (leg2NewPosy - leg2PrevPosy)/2;
		calc_d(leg2NewMiddlePosx, leg2NewMiddlePosy, z);
		leg2NewMiddleAngleGamma = get_gamma(leg2NewMiddlePosx, leg2NewMiddlePosy);
		leg2NewMiddleAngleBeta = get_beta();
		leg2NewMiddleAngleAlpha = get_alpha(leg2NewPosz);
	}
	
	if(leg3Lift == -1)
	{
		leg3NewMiddlePosx = leg3PrevPosx + (leg3NewPosx - leg3PrevPosx)/2;
		leg3NewMiddlePosy = leg3PrevPosy + (leg3NewPosy - leg3PrevPosy)/2;
		calc_d(leg3NewMiddlePosx, leg3NewMiddlePosy,z);
		leg3NewMiddleAngleGamma = get_gamma(leg3NewMiddlePosx, leg3NewMiddlePosy);
		leg3NewMiddleAngleBeta = get_beta();
		leg3NewMiddleAngleAlpha = get_alpha(leg3NewPosz);
	}
	
	if(leg4Lift == -1)
	{
		leg4NewMiddlePosx = leg4PrevPosx + (leg4NewPosx - leg4PrevPosx)/2;
		leg4NewMiddlePosy = leg4PrevPosy + (leg4NewPosy - leg4PrevPosy)/2;
		calc_d(leg4NewMiddlePosx, leg4NewMiddlePosy, z);
		leg4NewMiddleAngleGamma = get_gamma(leg4NewMiddlePosx, leg4NewMiddlePosy);
		leg4NewMiddleAngleBeta = get_beta();
		leg4NewMiddleAngleAlpha = get_alpha(leg4NewPosz);
	}
	
	if(leg5Lift == -1)
	{
		leg5NewMiddlePosx = leg5PrevPosx + (leg5NewPosx - leg5PrevPosx)/2;
		leg5NewMiddlePosy = leg5PrevPosy + (leg5NewPosy - leg5PrevPosy)/2;
		calc_d(leg5NewMiddlePosx, leg5NewMiddlePosy,z);
		leg5NewMiddleAngleGamma = get_gamma(leg5NewMiddlePosx, leg5NewMiddlePosy);
		leg5NewMiddleAngleBeta = get_beta();
		leg5NewMiddleAngleAlpha = get_alpha(leg5NewPosz);
	}
	
	if(leg6Lift == -1)
	{
		leg6NewMiddlePosx = leg6PrevPosx + (leg6NewPosx - leg6PrevPosx)/2;
		leg6NewMiddlePosy = leg6PrevPosy + (leg6NewPosy - leg6PrevPosy)/2;
		calc_d(leg6NewMiddlePosx, leg6NewMiddlePosy,z);
		leg6NewMiddleAngleGamma = get_gamma(leg6NewMiddlePosx, leg6NewMiddlePosy);
		leg6NewMiddleAngleBeta = get_beta();
		leg6NewMiddleAngleAlpha = get_alpha(leg6NewPosz);
	}
	


	Leg_motion();
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
	
	/*
	float alpha = 3.1415/4;
	float beta = 3.1415/2.2;
	int speed = 180;
	
	//dummy code that puts robot in stanfdard position
	
	servoBufferPosition(1, 0, speed);
	servoBufferPosition(2, 0, speed);
	servoBufferPosition(7, 0, speed);
	servoBufferPosition(8, 0, speed);
	servoBufferPosition(13, 0, speed);
	servoBufferPosition(14, 0, speed);
	
	_delay_ms(100);
	
	servoBufferPosition(15, -alpha, speed);
	servoBufferPosition(3, -alpha, speed);
	servoBufferPosition(9, -alpha, speed);
	servoBufferPosition(16, alpha, speed);
	servoBufferPosition(4, alpha, speed);
	servoBufferPosition(10, alpha, speed);
	
	_delay_ms(100);
	
	servoBufferPosition(12, -beta, speed);
	servoBufferPosition(18, -beta, speed);
	servoBufferPosition(6, -beta, speed);
	servoBufferPosition(5, beta, speed);
	servoBufferPosition(11, beta, speed);
	servoBufferPosition(17, beta, speed);
	
	_delay_ms(5000);
	*/
	moveLeg1too(x0_1, y0_1, z0, speed);
	moveLeg2too(x0_2, y0_2, z0, speed);
	moveLeg3too(x0_3, y0_3, z0, speed);
	moveLeg4too(x0_4, y0_4, z0, speed);
	moveLeg5too(x0_5, y0_5, z0, speed);
	moveLeg6too(x0_6, y0_6, z0, speed);
	
	
	_delay_ms(5000);
	
	//moveRobotTob(0,50,100);
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

		//moveRobot((float)0,(float)40,(float)0,(float)120,(int)100,(float)0,(float)0);


        //TODO:: Please write your application code 
    }
}