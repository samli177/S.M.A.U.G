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
#include "MpuInit.h"

int speed;
float iterations;
float height;
int height_flag;



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

float z;
float zInput;

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
	
	leg->newPosx = get_x0();
	leg->newPosy = get_y0();
	leg->newPosz = get_z0();
}

void initvar()
{
	speedMultiplier = 1500;
	speed = 300;
	iterations = 5;
	maxStepLength = 55;
	stdLength = sqrtf(get_x0_1()*get_x0_1() + get_y0_1()*get_y0_1());
	z = -120;
	//z = z0;
	//zInput = z0;
	
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

void update_leg_info(struct LegData* leg)
{
	SERVO_update_data(leg->servoAlpha);
	leg->currPosAlpha = SERVO_get_pos();
	leg->currLoadAlpha = SERVO_get_load();
	leg->currSpeedAlpha = SERVO_get_speed();
	leg->currTempAlpha = SERVO_get_temperature();
	leg->currVoltAlpha = SERVO_get_voltage();
	
	SERVO_update_data(leg->servoBeta);
	leg->currPosBeta = SERVO_get_pos();
	leg->currLoadBeta = SERVO_get_load();
	leg->currSpeedBeta = SERVO_get_speed();
	leg->currTempBeta = SERVO_get_temperature();
	leg->currVoltBeta = SERVO_get_voltage();
	
	SERVO_update_data(leg->servoGamma);
	leg->currPosGamma = SERVO_get_pos();
	leg->currLoadGamma = SERVO_get_load();
	leg->currSpeedGamma = SERVO_get_speed();
	leg->currTempGamma = SERVO_get_temperature();
	leg->currVoltGamma = SERVO_get_voltage();
	
}

uint8_t close_enough(struct LegData* leg, uint8_t tolerance)
{
	int tempAlpha, tempBeta, tempGamma;
	tempGamma = fabsf(angle_to_servo_pos(leg->goalAngleGamma) - leg->currPosGamma);
	tempBeta = fabsf(angle_to_servo_pos(leg->goalAngleBeta) - leg->currPosBeta);
	tempAlpha = fabsf(angle_to_servo_pos(leg->goalAngleAlpha) - leg->currPosAlpha);
	/*
	USART_SendValue(tempAlpha);
	USART_SendValue(tempBeta);
	USART_SendValue(tempGamma);
	*/
	
	if(tempGamma >= tolerance)
	{
		return 0;
	} else if(tempBeta >= tolerance)
	{
		//return 0;
	} else if(tempAlpha >= tolerance)
	{
		return 0;
	}
	
	
	
	return 1;
}

uint16_t angle_to_servo_pos(float angle)
{
	angle = 150 + angle * 150/3.1415;
	
	// limit inputs to between 0 and 300 degrees
	if (angle > 300)
	{
		angle = 300;
	} else if (angle < 0)
	{
		angle = 0;
	}
	
	angle = (angle * 0x3ff / 300);
	return (uint16_t)angle; //this will probably truncate correctly...or not....
}

void step_start(struct LegData* leg)
{
	leg->prevPosx = leg->newPosx;
	leg->prevPosy = leg->newPosy;
	//leg->prevPosz = leg->newPosz;
	leg->prevAngleAlpha = leg->newAngleAlpha;
	leg->prevAngleBeta = leg->newAngleBeta;
	leg->prevAngleGamma = leg->newAngleGamma;
}

void step_part1_calculator(struct LegData* leg)
{
	newPosxWB = leg->lift * maxStepLength / stepMax * (xDirection + leg->xRot) * stepScaling;
	newPosyWB = leg->lift * maxStepLength / stepMax * (yDirection + leg->yRot) * stepScaling;
	//leg->newPosz = z;
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
	if(direction < 0 || direction > 90)
	{
		direction = 0;
	}
	
	rotation = (float)rot / 50 -1;
	if(rotation < -1)
	{
		rotation = -1;
	} else if(rotation > 1)
	{
		rotation = 1;
	}
	
	speedf = (float)spd;
	if(speedf < 0)
	{
		speedf = 0;
	} else if(speedf > 100)
	{
		speedf = 100;
	}
	
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
	
	
	
	leg1.xRot = rotation * get_y0_1()/ stdLength;
	leg1.yRot = rotation * get_x0_1() / stdLength;

	leg2.xRot = 0;
	leg2.yRot = rotation;

	leg3.xRot = rotation * (- get_y0_1() / stdLength);
	leg3.yRot = rotation * get_x0_1() / stdLength;

	leg4.xRot = rotation * (- get_y0_1() / stdLength);
	leg4.yRot = rotation * (- get_x0_1() / stdLength);

	leg5.xRot = 0;
	leg5.yRot = -rotation;

	leg6.xRot = rotation  * get_y0_1() / stdLength;
	leg6.yRot = rotation  * (- get_x0_1() / stdLength);


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
		tempz = leg->newPosz + 30;
	}
	else
	{
		tempz = leg->newPosz;
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
		leg->climbing = 0;
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
	leg->goalAngleAlpha = leg->side *(leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side*(-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;
	SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,(int)speedAlpha);
	SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,(int)speedBeta);
	SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,(int)speedGamma);
}
void leg_motion()
{
	leg_motion_init();
	for(int i = 0; i <= (int)iterations+1; ++i)
	{
		if(leg1.climbing == 1)
		{
			leg_climb(&leg1);
		}
		/*else if(leg1.climbing == 2)
		{
			leg_climb_down(&leg1);
		}*/

		move_leg(&leg1,i);

		if(leg2.climbing == 1)
		{
			leg_climb(&leg2);
		}/*
		else if(leg2.climbing == 2)
		{
			leg_climb_down(&leg2);
		}*/

		move_leg(&leg2,i);

		if(leg3.climbing == 1)
		{
			leg_climb(&leg3);
		}/*
		else if(leg3.climbing == 2)
		{
			leg_climb_down(&leg3);
		}*/

		move_leg(&leg3,i);

		if(leg4.climbing == 1)
		{
			leg_climb(&leg4);
		}/*
		else if(leg4.climbing == 2)
		{
			leg_climb_down(&leg4);
		}*/

		move_leg(&leg4,i);

		if(leg5.climbing == 1)
		{
			leg_climb(&leg5);
		}/*
		else if(leg5.climbing == 2)
		{
			leg_climb_down(&leg5);
		}*/

		move_leg(&leg5,i);

		if(leg6.climbing == 1)
		{
			leg_climb(&leg6);
		}/*
		else if(leg6.climbing == 2)
		{
			leg_climb_down(&leg6);
		}*/

		move_leg(&leg6,i);

		SERVO_action();
		wait(30);
		
		//change_z(USART_get_z());
		

	}
}

void change_z(float input)
{
	leg1.newPosz = input;
	leg2.newPosz = input;
	leg3.newPosz = input;
	leg4.newPosz = input;
	leg5.newPosz = input;
	leg6.newPosz = input;

	height_change_all(input);
}

void move_to_std()
{
	
	//change_z(USART_get_z());
	
	leg1.lift = -1;
	leg2.lift = 1;
	leg3.lift = -1;
	leg4.lift = 1;
	leg5.lift = -1;
	leg6.lift = 1;

	step_start(&leg1);
	step_start(&leg2);
	step_start(&leg3);
	step_start(&leg4);
	step_start(&leg5);
	step_start(&leg6);

	leg2.newPosx = get_x0();
	leg2.newPosy = get_y0();
	step_part2_calculator(&leg2);

	leg4.newPosx = get_x0();
	leg4.newPosy = get_y0();
	step_part2_calculator(&leg4);

	leg6.newPosx = get_x0();
	leg6.newPosy = get_y0();
	step_part2_calculator(&leg6);

	leg_motion();

	leg1.lift = 1;
	leg2.lift = -1;
	leg3.lift = 1;
	leg4.lift = -1;
	leg5.lift = 1;
	leg6.lift = -1;

	step_start(&leg1);
	step_start(&leg2);
	step_start(&leg3);
	step_start(&leg4);
	step_start(&leg5);
	step_start(&leg6);

	leg1.newPosx = get_x0();
	leg1.newPosy = get_y0();
	step_part2_calculator(&leg1);

	leg3.newPosx = get_x0();
	leg3.newPosy = get_y0();
	step_part2_calculator(&leg3);

	leg5.newPosx = get_x0();
	leg5.newPosy = get_y0();
	step_part2_calculator(&leg5);

	leg_motion();
	
	
}

void climb()
{

	z = -150;
	change_z(z);

	move_to_std();

	leg1.lift = -1;
	leg2.lift = 1;
	leg3.lift = -1;
	leg4.lift = 1;
	leg5.lift = -1;
	leg6.lift = 1;

	height_flag = 0;
	height = 60;

	height_change_leg1(-70);

	leg1.climbing = 1;
	move_robot(0,50,100);
	leg1.climbing = 0;
	leg6.climbing = 1;
	move_robot(0,50,100);
	leg6.climbing = 0;
	leg5.climbing = 1;
	move_robot(0,50,100);
	leg5.climbing = 0;
	leg2.climbing = 1;
	move_robot(0,50,100);
	leg2.climbing = 0;
	leg3.climbing = 1;
	move_robot(0,50,100);
	leg3.climbing = 0;
	leg4.climbing = 1;
	move_robot(0,50,100);
	leg4.climbing = 0;

	_delay_ms(5000);
}

void leg_climb(struct LegData* leg)
{

	tempz = z + height + 30;

	tempx = leg->newPosx;
	tempy = leg->newPosy;

	calc_d(tempx, tempy, tempz);
	leg->temp2AngleGamma = get_gamma(tempx, tempy);
	leg->temp2AngleBeta = get_beta();
	leg->temp2AngleAlpha = get_alpha(tempz);
	leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;

	SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
	SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);

	_delay_ms(800);

	SERVO_goto(leg->servoGamma, leg->goalAngleGamma, 100);

	 _delay_ms(400);
	 if(height_flag)
	 {
		 tempz = height+z;
		 /*
		 tempz = height - 120;
		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
		 
		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
		 */

		 switch(leg->servoGamma)
		 {
			 case 1:
			 height_change_leg1(tempz);
			 break;

			 case 2:
			 height_change_leg4(tempz);
			 break;

			 case 7:
			 height_change_leg5(tempz);
			 break;

			 case 8:
			 height_change_leg6(tempz);
			 break;

			 case 13:
			 height_change_leg2(tempz);
			 break;

			 case 14:
			 height_change_leg3(tempz);
			 break;

			 default:
			 break;
		 }
		// _delay_ms(1000);
	 }
	 else
	 {
		 tempz = z + height + 30;

		 switch(leg->servoGamma)
		 {
			 case 1:
			 height_change_leg1(tempz);
			 break;

			 case 2:
			 height_change_leg4(tempz);
			 break;

			 case 7:
			 height_change_leg5(tempz);
			 break;

			 case 8:
			 height_change_leg6(tempz);
			 break;

			 case 13:
			 height_change_leg2(tempz);
			 break;

			 case 14:
			 height_change_leg3(tempz);
			 break;

			 default:
			 break;
		 }

		 tempx = leg->newPosx;
		 tempy = leg->newPosy;


		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);

		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);

		 _delay_ms(400);

	 do 
	 {


		 tempz -= 10;



		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);	

		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100); 
		 _delay_ms(100);
		 update_leg_info(leg);

	 } while (!(leg->currLoadAlpha > 235));

	 tempz += 20;

	 calc_d(tempx, tempy, tempz);
	 leg->temp2AngleBeta = get_beta();
	 leg->temp2AngleAlpha = get_alpha(tempz);
	 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
	 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);

	 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
	 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
	 _delay_ms(400);

	 height_flag = 1;
	 height = tempz - z;

	 }

	 leg->climbing = 2;
	 step_start(leg);
	 leg->newPosz = tempz;

	 /*
	 switch(leg->servoGamma)
	 {
		 case 1:
		 height_change_leg4(tempz);
		 break;
		 
		 case 2:
		 height_change_leg3(tempz);
		 break;
		 
		 case 7:
		 height_change_leg6(tempz);
		 break;
		 
		 case 8:
		 height_change_leg1(tempz);
		 break;
		 
		 case 13:
		 height_change_leg5(tempz);
		 break;
		 
		 case 14:
		 height_change_leg2(tempz);
		 break;
		 
		 default:
		 break;
	 }
	*/

}






/*

void climb_down()
{
	z = -90;
	change_z(z);
	
	move_to_std();
	
	leg1.lift = -1;
	leg2.lift = 1;
	leg3.lift = -1;
	leg4.lift = 1;
	leg5.lift = -1;
	leg6.lift = 1;
	
	height_flag = 0;
	height = 60;
	
	height_change_leg1(-140);
	
	leg1.climbing = 2;
	move_robot(0,50,100);
	leg1.climbing = 0;
	leg6.climbing = 2;
	move_robot(0,50,100);
	leg6.climbing = 0;
	leg5.climbing = 2;
	move_robot(0,50,100);
	leg5.climbing = 0;
	leg2.climbing = 2;
	move_robot(0,50,100);
	leg2.climbing = 0;
	leg3.climbing = 2;
	move_robot(0,50,100);
	leg3.climbing = 0;
	leg4.climbing = 2;
	move_robot(0,50,100);
	leg4.climbing = 0;
	
	_delay_ms(5000);
}

void leg_climb_down(struct LegData* leg) // To descend from the high pass obstacle
{
	
	tempz = z - height + 30;
	
	tempx = leg->newPosx;
	tempy = leg->newPosy;

	calc_d(tempx, tempy, tempz);
	leg->temp2AngleGamma = get_gamma(tempx, tempy);
	leg->temp2AngleBeta = get_beta();
	leg->temp2AngleAlpha = get_alpha(tempz);
	leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;
	
	SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
	SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
	
	_delay_ms(400);
	
	SERVO_goto(leg->servoGamma, leg->goalAngleGamma, 100);
	
	 _delay_ms(400);
	 if(height_flag)
	 {
		 tempz = -height + z;
		 */
		 /* Detta ska vara botrkommenterat...
		 tempz = height - 120;
		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
		 
		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
		 */
		 /*
		 switch(leg->servoGamma)
		 {
			 case 1:
			 height_change_leg1(tempz);
			 break;
			 
			 case 2:
			 height_change_leg4(tempz);
			 break;
			 
			 case 7:
			 height_change_leg5(tempz);
			 break;
			 
			 case 8:
			 height_change_leg6(tempz);
			 break;
			 
			 case 13:
			 height_change_leg2(tempz);
			 break;
			 
			 case 14:
			 height_change_leg3(tempz);
			 break;
			 
			 default:
			 break;
		 }
		// _delay_ms(1000);
	 }
	 else
	 {
		 tempz = z - height + 30;
		 
		 switch(leg->servoGamma)
		 {
			 case 1:
			 height_change_leg1(tempz);
			 break;
			 
			 case 2:
			 height_change_leg4(tempz);
			 break;
			 
			 case 7:
			 height_change_leg5(tempz);
			 break;
			 
			 case 8:
			 height_change_leg6(tempz);
			 break;
			 
			 case 13:
			 height_change_leg2(tempz);
			 break;
			 
			 case 14:
			 height_change_leg3(tempz);
			 break;
			 
			 default:
			 break;
		 }
		 
		 tempx = leg->newPosx;
		 tempy = leg->newPosy;
		 
		 
		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
		 
		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
		 
		 _delay_ms(400);
	 
	 do 
	 {
		 
		 
		 tempz -= 10;
		 
		 
		 
		 calc_d(tempx, tempy, tempz);
		 leg->temp2AngleBeta = get_beta();
		 leg->temp2AngleAlpha = get_alpha(tempz);
		 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);	
		 
		 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
		 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100); 
		 _delay_ms(100);
		 update_leg_info(leg);
		 
	 } while (!(leg->currLoadAlpha > 235));
	 
	 tempz += 20;
	 
	 calc_d(tempx, tempy, tempz);
	 leg->temp2AngleBeta = get_beta();
	 leg->temp2AngleAlpha = get_alpha(tempz);
	 leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
	 leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
	 
	 SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,100);
	 SERVO_goto(leg->servoBeta, leg->goalAngleBeta,100);
	 _delay_ms(400);
	 
	 height_flag = 1;
	 height = tempz - z;
	 
	 }
	 
	 leg->climbing = 2;
	 step_start(leg);
	 leg->newPosz = tempz;
	 */
	 /* Detta ska vara bortkommenterat...
	 switch(leg->servoGamma)
	 {
		 case 1:
		 height_change_leg4(tempz);
		 break;
		 
		 case 2:
		 height_change_leg3(tempz);
		 break;
		 
		 case 7:
		 height_change_leg6(tempz);
		 break;
		 
		 case 8:
		 height_change_leg1(tempz);
		 break;
		 
		 case 13:
		 height_change_leg5(tempz);
		 break;
		 
		 case 14:
		 height_change_leg2(tempz);
		 break;
		 
		 default:
		 break;
	 }
	*/
/*
}*/

void turn_degrees(uint16_t degrees, int8_t dir)
{
	float radians = degrees * M_PI/180;
	float tolerance = 2.0 * M_PI/180;
	wait(10);
	float startAngle = MPU_get_y();
	float newAngle;
	float angleLeft;
	USART_SendValue(startAngle*180/M_PI);
	float diff;
	
	do
	{
		wait(10);
		newAngle = MPU_get_y();
		
		diff = newAngle - startAngle;
		if(fabs(diff) <= M_PI)
		{
			angleLeft = radians - fabs(diff);
		} else if(diff < 0) {
			angleLeft = radians - M_PI - fmod(diff, M_PI);
		} else {
			angleLeft = radians - fmod(diff, M_PI);
		}
		USART_SendValue(angleLeft*180/M_PI);
		
		move_robot(0, 50 + dir * 50 * angleLeft*8.0/M_PI, 0);
	} while(fabs(angleLeft) > tolerance);
	
	/*wait(10);
	float startAngle = MPU_get_y() * 180/pi;
	float totalDegrees = 0;
	float lastDegree = startAngle;
	float degreesLeft = degrees;
	
	while (fabs(degreesLeft) > 2)
	{
		PORTD ^= (1 << PORTD5);
		wait(10);
		float newDegree = MPU_get_y() * 180/pi;
		float change = fmod(newDegree - lastDegree, 180);
		lastDegree = newDegree;
		totalDegrees += change;
		degreesLeft = (float) degrees - totalDegrees;
		
		USART_SendValue(degreesLeft);
		
		if(change < fabs(degreesLeft))
		{
			move_robot(0,50+dir*50,0);
		}
		else
		{
			move_robot(0,50+dir*50*fabs(degreesLeft/change),0);
		}
		
		
	} */
}
