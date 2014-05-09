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
float height;
int climb_step;




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

int height_flag;
int climbing_flag;
int legsNotDown;

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

//Längd från origo till standardposition för ben 1,3,4,6
float stdLength;

//Max möjliga steglängd från grundpositionen
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
	leg1.loadLimitTot = 590; 
	leg1.loadLimitAlpha = 440;
	leg1.loadLimitBeta = 240;
	
	leg2.servoAlpha = 16;
	leg2.servoBeta = 18;
	leg2.servoGamma = 14;
	leg2.loadLimitTot = 550; 
	leg2.loadLimitAlpha = 350;
	leg2.loadLimitBeta = 290;
	
	leg3.servoAlpha = 4;
	leg3.servoBeta = 6;
	leg3.servoGamma = 2;
	leg3.loadLimitTot = 620; 
	leg3.loadLimitAlpha = 420;
	leg3.loadLimitBeta = 220;
	
	leg4.servoAlpha = 3;
	leg4.servoBeta = 5;
	leg4.servoGamma = 1;
	leg4.loadLimitTot = 583; 
	leg4.loadLimitAlpha = 480;
	leg4.loadLimitBeta = 330;
	
	leg5.servoAlpha = 15;
	leg5.servoBeta = 17;
	leg5.servoGamma = 13;
	leg5.loadLimitTot = 600; 
	leg5.loadLimitAlpha = 400;
	leg5.loadLimitBeta = 150;
	
	leg6.servoAlpha = 9;
	leg6.servoBeta = 11;
	leg6.servoGamma = 7;
	leg6.loadLimitTot = 443;
	leg6.loadLimitAlpha = 330;
	leg6.loadLimitBeta = 145;
	
	// To count x and y from a new z:
	// sign*(120 + 0.5 *(new_z - z0))/divider + term
	/*
	leg1.signX = -1;
	leg1.dividerX = sqrt2;
	leg1.termX = -61.85;
	
	leg1.signY = 1;
	leg1.dividerY = sqrt2;
	leg1.termY = 120;
	
	leg2.signX = -1;
	leg2.dividerX = 1;
	leg2.termX = -100;
	
	leg2.signY = 0;
	leg2.dividerY = 1;
	leg2.termY = get_y0_2();
	
	leg3.signX = -1;
	leg3.dividerX = sqrt2;
	leg3.termX = -61.85;
	
	leg3.signY = -1;
	leg3.dividerY = sqrt2;
	leg3.termY = -120;
	
	leg4.signX = 1;
	leg4.dividerX = sqrt2;
	leg4.termX = 61.85;

	leg4.signY = -1;
	leg4.dividerY = sqrt2;
	leg4.termY = -120;
	
	leg5.signX = 1;
	leg5.dividerX = 1;
	leg5.termX = 100;
	
	leg5.signY = 0;
	leg5.dividerY = 1;
	leg5.termY = get_y0_5();
	
	leg6.signX = 1;
	leg6.dividerX = sqrt2;
	leg6.termX = 61.85;

	leg6.signY = 1;
	leg6.dividerY = sqrt2;
	leg6.termY = 120;
	*/
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
	//leg->newPosz = z;
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


//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
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

	//x och y riktning för förflyttning, skalad med hastigheten
	xDirection = -sinf(direction * pi / 45) * speedf / 100;
	yDirection = cosf(direction * pi / 45) * speedf / 100;

	

	//x och y riktning för rotation, skalad med rotationshastighet
	
	
	
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
	stepScaling = fmaxf(speedf/100, fabsf(rotation));


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
	//speedAlpha = fabsf(leg->temp1AngleAlpha - leg->temp2AngleAlpha)*speedMultiplier;
	//speedBeta = fabsf(leg->temp1AngleBeta - leg->temp2AngleBeta)*speedMultiplier;
	//speedGamma = fabsf(leg->temp1AngleGamma - leg->temp2AngleGamma)*speedMultiplier;
	leg->goalAngleAlpha = leg->side *(leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side*(-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;
	SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,200);
	SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,200);
	SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,200);
}

void climb_all_one_leg()
{
	climbing_flag = 1;
	height = 70;
	
	z = -120;
	change_z(z);
	
	move_to_std();
	
	while(climbing_flag)
	{
		leg1.lift = -1;
		leg2.lift = 1;
		leg3.lift = -1;
		leg4.lift = 1;
		leg5.lift = -1;
		leg6.lift = 1;
		height_change_leg1(z + height);
		leg1.climbing = 1;
		leg1.newPosz = z + height;
		climb_one_leg(&leg1);
		move_robot(0,50,100);
		
		_delay_ms(1000);
		
		leg1.lift = 1;
		leg2.lift = -1;
		leg3.lift = 1;
		leg4.lift = -1;
		leg5.lift = 1;
		leg6.lift = -1;
		height_change_leg2(z + height);
		leg2.climbing = 1;
		leg2.newPosz = z + height;
		climb_one_leg(&leg2);
		move_robot(0,50,100);
		
		_delay_ms(1000);
		
		leg1.lift = -1;
		leg2.lift = 1;
		leg3.lift = -1;
		leg4.lift = 1;
		leg5.lift = -1;
		leg6.lift = 1;
		height_change_leg3(z + height);
		leg3.climbing = 1;
		leg3.newPosz = z + height;
		climb_one_leg(&leg3);
		move_robot(0,50,100);
		
		_delay_ms(1000);
		
		leg1.lift = 1;
		leg2.lift = -1;
		leg3.lift = 1;
		leg4.lift = -1;
		leg5.lift = 1;
		leg6.lift = -1;
		height_change_leg4(z + height);
		leg4.newPosz = z + height;
		leg4.climbing = 1;
		climb_one_leg(&leg4);
		move_robot(0,50,100);
		
		_delay_ms(1000);
		
		leg1.lift = -1;
		leg2.lift = 1;
		leg3.lift = -1;
		leg4.lift = 1;
		leg5.lift = -1;
		leg6.lift = 1;
		height_change_leg5(z + height);
		leg5.climbing = 1;
		leg5.newPosz = z + height;
		climb_one_leg(&leg5);
		move_robot(0,50,100);
		
		_delay_ms(1000);
		
		leg1.lift = 1;
		leg2.lift = -1;
		leg3.lift = 1;
		leg4.lift = -1;
		leg5.lift = 1;
		leg6.lift = -1;
		height_change_leg6(z + height);
		leg6.climbing = 1;
		leg6.newPosz = z + height;
		climb_one_leg(&leg6);
		move_robot(0,50,100);
		
		_delay_ms(5000);
	}
}

void climb_one_leg(struct LegData* leg)
{
	for(int i = 0; i <= (int)iterations+1; ++i)
	{
		move_climb(leg,i);
		
		SERVO_action();
		_delay_ms(300);
	}
	
	if(climbing_flag)
	{
		legsNotDown = 1;
		while(legsNotDown)
		{
			legsNotDown = 0;
			
			if(leg->lift == 1 && leg->climbing == 1)
			{
				leg_move_down(leg);
			}
			SERVO_action();
			_delay_ms(200);
			
			if(leg->lift == 1 && leg->climbing == 1)
			{
				leg_check_down(leg);
			}
			SERVO_action();
			_delay_ms(200);
		}
	}
}

void leg_motion()
{
	leg_motion_init();
	for(int i = 0; i <= (int)iterations+1; ++i)
	{
		if(leg1.climbing == 1)
		{
			move_climb(&leg1, i);
		}
		/*else if(leg1.climbing == 2)
		{
			leg_climb_down(&leg1);
		}*/
		else
		{
			move_leg(&leg1,i);
		}
		if(leg2.climbing == 1)
		{
			move_climb(&leg2, i);
		}/*
		else if(leg2.climbing == 2)
		{
			leg_climb_down(&leg2);
		}*/
		else
		{
			move_leg(&leg2,i);
		}
		
		if(leg3.climbing == 1)
		{
			move_climb(&leg3, i);
		}/*
		else if(leg3.climbing == 2)
		{
			leg_climb_down(&leg3);
		}*/
		else
		{
			move_leg(&leg3,i);
		}
		if(leg4.climbing == 1)
		{
			move_climb(&leg4, i);
		}/*
		else if(leg4.climbing == 2)
		{
			leg_climb_down(&leg4);
		}*/
		else
		{
			move_leg(&leg4,i);
		}
		if(leg5.climbing == 1)
		{
			move_climb(&leg5, i);
		}/*
		else if(leg5.climbing == 2)
		{
			leg_climb_down(&leg5);
		}*/
		else
		{
			move_leg(&leg5,i);
		}
		if(leg6.climbing == 1)
		{
			move_climb(&leg6, i);
		}/*
		else if(leg6.climbing == 2)
		{
			leg_climb_down(&leg6);
		}*/
		else
		{
			move_leg(&leg6,i);
		}

		SERVO_action();
		_delay_ms(300);
		
		
		//change_z(USART_get_z());
	}
	
	if(climbing_flag)
	{
		legsNotDown = 1;
		while(legsNotDown)
		{
			legsNotDown = 0;
			
			if(leg1.lift == 1 && leg1.climbing == 1)
			{
				leg_move_down(&leg1);
			}
			if(leg2.lift == 1 && leg2.climbing == 1)
			{
				leg_move_down(&leg2);
			}
			if(leg3.lift == 1 && leg3.climbing == 1)
			{
				leg_move_down(&leg3);
			}
			if(leg4.lift == 1 && leg4.climbing == 1)
			{
				leg_move_down(&leg4);
			}
			if(leg5.lift == 1 && leg5.climbing == 1)
			{
				leg_move_down(&leg5);
			}
			if(leg6.lift == 1 && leg6.climbing == 1)
			{
				leg_move_down(&leg6);
			}
			
			SERVO_action();
			_delay_ms(200);
			
			if(leg1.lift == 1 && leg1.climbing == 1)
			{
				leg_check_down(&leg1);
			}
			if(leg2.lift == 1 && leg2.climbing == 1)
			{
				leg_check_down(&leg2);
			}
			if(leg3.lift == 1 && leg3.climbing == 1)
			{
				leg_check_down(&leg3);
			}
			if(leg4.lift == 1 && leg4.climbing == 1)
			{
				leg_check_down(&leg4);
			}
			if(leg5.lift == 1 && leg5.climbing == 1)
			{
				leg_check_down(&leg5);
			}
			if(leg6.lift == 1 && leg6.climbing == 1)
			{
				leg_check_down(&leg6);
			}
			
			SERVO_action();
			_delay_ms(200);
		}
	}
}

void leg_move_down(struct LegData* leg)
{
	leg->newPosz -=10;
	tempz = leg->newPosz;
	// To count new x and y positions: sign*(120 + 0.5 *(new_z - z0))/divider + term
	leg->newPosx -= 5;
	tempx = leg->newPosx;
	tempy = leg->newPosy;
	
	calc_d(tempx, tempy, tempz);
	leg->temp2AngleGamma = get_gamma(tempx, tempy);
	leg->temp2AngleBeta = get_beta();
	leg->temp2AngleAlpha = get_alpha(tempz);
	leg->goalAngleAlpha = leg->side *(leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side*(-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;
	SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,200);
	SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,200);
	SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,200);
}

void leg_check_down(struct LegData* leg)
{
	update_leg_info(leg);
	if(leg->currLoadAlpha + leg->currLoadBeta > leg->loadLimitTot || 
		leg->currLoadAlpha > leg->loadLimitAlpha ||
		leg->currLoadBeta > leg->loadLimitBeta)
	{
		
		//leg->newPosz += 20;
		//leg->newPosx -= 10;
		tempz = leg->newPosz;
		tempx = leg->newPosx;
		tempy = leg->newPosy;
		
		calc_d(tempx, tempy, tempz);
		leg->temp2AngleBeta = get_beta();
		leg->temp2AngleAlpha = get_alpha(tempz);
		leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
	 
		SERVO_buffer_position(leg->servoAlpha,leg->goalAngleAlpha,100);
		SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,100);
		
		switch(leg->servoGamma)
		{
			case 8:
			height_change_leg1(tempz);
			break;
			
			case 14:
			height_change_leg2(tempz);
			break;
			
			case 2:
			height_change_leg3(tempz);
			break;
			
			case 7:
			height_change_leg4(tempz);
			break;
			
			case 13:
			height_change_leg5(tempz);
			break;
			
			case 1:
			height_change_leg6(tempz);
			break;
			
			default:
			break;
		}
		
		 
		
		leg->climbing = 0;
	}
	
	if(legsNotDown == 0 && leg->climbing == 1)
	{
		legsNotDown = 1;
	}
}

void climb()
{
	climbing_flag = 1;
	height = 70;
	climb_step = 0;
	
	z = -120;
	change_z(z);
	
	move_to_std();
	
	while(climbing_flag)
	{
		if(leg1.lift && climb_step == 0)
		{
			height_change_leg6(z + height);
			leg6.climbing = 1;
			leg6.newPosz = z + height;
		}
		else if (!(leg1.lift) && climb_step == 0)
		{
			height_change_leg1(leg1.newPosz + height);
			leg1.climbing = 1;
			leg1.newPosz = z + height;
		}
		else if (leg1.lift && climb_step == 1)
		{
			height_change_leg2(z + height);
			leg2.climbing = 1;
			leg2.newPosz = z + height;
		}
		else if (!(leg1.lift) && climb_step == 1)
		{
			height_change_leg5(z + height);
			leg5.climbing = 1;
			leg5.newPosz = z + height;
		}
		else if (leg1.lift && climb_step == 2)
		{
			height_change_leg4(z + height);
			leg4.climbing = 1;
			leg4.newPosz = z + height;
		}
		else if (!(leg1.lift) && climb_step == 2)
		{
			height_change_leg3(z + height);
			leg3.climbing = 1;
			leg3.newPosz = z + height;
		}
		
		move_robot(0,50,100);
		_delay_ms(2000);
	}
	
}

void move_climb(struct LegData* leg, float n)
{
	
	tempz = z + height + 30;
	
	if(leg->lift == 1 && n == 0)
	{		
		tempx = leg->newPosx;
		tempy = leg->newPosy;

		calc_d(tempx, tempy, tempz);
		leg->temp2AngleGamma = get_gamma(tempx, tempy);
		leg->temp2AngleBeta = get_beta();
		leg->temp2AngleAlpha = get_alpha(tempz);
		leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
		leg->goalAngleGamma = leg->temp2AngleGamma;
			
		SERVO_buffer_position(leg->servoAlpha,leg->goalAngleAlpha,200);
		SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,200);
	}
	else if(n != iterations + 1)
	{
		leg->temp1AngleGamma = leg->temp2AngleGamma;
		leg->temp1AngleBeta = leg->temp2AngleBeta;
		leg->temp1AngleAlpha = leg->temp2AngleAlpha;

		tempx = leg->prevPosx + n*(leg->newPosx - leg->prevPosx)/iterations;
		tempy = leg->prevPosy + n*(leg->newPosy - leg->prevPosy)/iterations;
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
		//SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,200);
		//SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,200);
		SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,200);
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
/*
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
	
	_delay_ms(400);
	
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
/*
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