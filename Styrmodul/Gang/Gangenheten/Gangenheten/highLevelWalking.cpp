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
#include "LED.h"

float TURN_WALK_SPEED_MIN = 60;
float TURN_WALK_SPEED_MAX = 75;
float TURN_TOLERANCE = 5;
float TURN_MIN_SPEED = 15;
float TURN_SCALE_FACTOR = 75 * M_PI / 180;
float TURN_WALK_SCALE_FACTOR = 45 * M_PI / 180;
float ITERATIONS;
int SERVO_SPEED = 0x300;
uint16_t ITERATIONS_DELAY_START = 50;
uint16_t ITERATIONS_DELAY = 25;
uint8_t STEP_HEIGHT = 20;
uint8_t STEP_HEIGHT_CLIMB = 90;
float AUTO_ELEVATION = -120;

int speed;
float height;
float climb_step;
float climb_step2;
float obstacle_height;
float z_in_use;
float climb_start_control;
float climb_start_slope_r;
float climb_start_slope_p;
float climb_check_down_r;
float climb_check_down_p;
uint8_t rotation_flag = 0;
float MPUPMean;
float MPURMean;
uint8_t climb_counter;
uint8_t climb_rotation_counter;

float costemp;
float sintemp;
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
float MAX_STEP_LENGTH; //abcd


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
	ITERATIONS = 4;
	MAX_STEP_LENGTH = 55;
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
	
	leg1.onObstacle = 0;
	leg2.onObstacle = 0;
	leg3.onObstacle = 0;
	leg4.onObstacle = 0;
	leg5.onObstacle = 0;
	leg6.onObstacle = 0;
	
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
	
	if(tempGamma >= tolerance)
	{
		return 0;
	} 
	else if(tempBeta >= tolerance)
	{
		//return 0;
	} 
	else if(tempAlpha >= tolerance)
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
	} 
	else if (angle < 0)
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
	newPosxWB = leg->lift * MAX_STEP_LENGTH / stepMax * (xDirection + leg->xRot) * stepScaling;
	newPosyWB = leg->lift * MAX_STEP_LENGTH / stepMax * (yDirection + leg->yRot) * stepScaling;
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
	if(dir < 0 || dir >= 90){
		dir = 0;
	}
	direction = (float)dir;
	if(rot < 0)
	{
		rot = 0;
	} 
	else if(rot > 100)
	{
		rot = 100;
	}
	rotation = (float)rot / 50 -1;
	if(spd < 0)
	{
		spd = 0;
	} 
	else if(spd > 100)
	{
		spd = 100;
	}
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
		tempx = (1-cosf(MAX_STEP_LENGTH*rotation/stdLength))*stdLength/MAX_STEP_LENGTH;
		tempy = sinf(MAX_STEP_LENGTH*rotation/stdLength)*stdLength/MAX_STEP_LENGTH;
		costemp = -get_x0_1() / stdLength;
		sintemp = -get_y0_1() / stdLength;
		
		//x och y riktning för rotation, skalad med rotationshastighet
		leg1.xRot = tempx*costemp-tempy*sintemp;
		leg1.yRot = tempx*sintemp+tempy*costemp;

		leg2.xRot = tempx;
		leg2.yRot = tempy;

		leg3.xRot = tempx*costemp+tempy*sintemp;
		leg3.yRot = -tempx*sintemp+tempy*costemp;

		leg4.xRot = -tempx*costemp+tempy*sintemp;
		leg4.yRot = -tempx*sintemp-tempy*costemp;
		
		leg5.xRot = -tempx;
		leg5.yRot = -tempy;

		leg6.xRot = -tempx*costemp-tempy*sintemp;
		leg6.yRot = tempx*sintemp-tempy*costemp;

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

		// Preforming the step.
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
	if(leg->lift == 1 && n != ITERATIONS+1 && climbing_flag == 1)
	{
		/*
		if (leg->climbing == 2)
		{
			tempz = obstacle_height + 50;
		}
		*/
	
		tempz = leg->newPosz + STEP_HEIGHT_CLIMB;
	}
	else if (leg->lift == 1 && n != ITERATIONS+1)
	{
	tempz = leg->newPosz + STEP_HEIGHT; 	
		
	}
	else
	{
		tempz = leg->newPosz;
	}
	
	if(n != 0 && n != ITERATIONS+1)
	{
		leg->temp1AngleGamma = leg->temp2AngleGamma;
		leg->temp1AngleBeta = leg->temp2AngleBeta;
		leg->temp1AngleAlpha = leg->temp2AngleAlpha;

		tempx = leg->prevPosx + n*(leg->newPosx - leg->prevPosx)/ITERATIONS;
		tempy = leg->prevPosy + n*(leg->newPosy - leg->prevPosy)/ITERATIONS;
	}
	else if (n == ITERATIONS+1)
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
	
	if(leg->lift == 1 && n != ITERATIONS+1 && climbing_flag == 1)
	{
		tempx += 50;
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
	
	if(leg->climbing == 4)
	{
		SERVO_goto(leg->servoAlpha, leg->goalAngleAlpha,SERVO_SPEED);
		SERVO_goto(leg->servoBeta, leg->goalAngleBeta,SERVO_SPEED);
		SERVO_goto(leg->servoGamma, leg->goalAngleGamma,SERVO_SPEED);
		leg->climbing = 0;
		wait(500);
	}
	else
	{
		SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,SERVO_SPEED);
		SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,SERVO_SPEED);
		SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,SERVO_SPEED);
	}
}

void leg_motion()
{
	leg_motion_init();
	MPU_update();
// 	climb_check_down_r = MPU_get_r();
// 	climb_check_down_p = MPU_get_p();
	for(int i = 0; i <= (int)ITERATIONS+1; ++i)
	{
		/*
		if(leg1.climbing == 1)
		{
			move_climb(&leg1, i);
		}
		else
		{
			*/
			move_leg(&leg1,i);
			/*
		}
		
		if(leg2.climbing == 1)
		{
			move_climb(&leg2, i);
		}
		else
		{
			*/
			move_leg(&leg2,i);
			/*
		}
		
		if(leg3.climbing == 1)
		{
			move_climb(&leg3, i);
		}
		else
		{
			*/
			move_leg(&leg3,i);
			/*
		}
		
		if(leg4.climbing == 1)
		{
			move_climb(&leg4, i);
		}
		else
		{
			*/
			move_leg(&leg4,i);
			/*
		}
		
		if(leg5.climbing == 1)
		{
			move_climb(&leg5, i);
		}
		else
		{
			*/
			move_leg(&leg5,i);
			/*
		}
		
		if(leg6.climbing == 1)
		{
			move_climb(&leg6, i);
		}
		else
		{
			*/
			move_leg(&leg6,i);
			/*
		}
*/
		SERVO_action();
		if((i == 0 || i == ITERATIONS + 1) && climbing_flag)
		{
			wait(300);
		}
		else if (i == 0 || i == ITERATIONS + 1)
		{
			wait(ITERATIONS_DELAY_START); //abcd
		}
		else
		{
			wait(ITERATIONS_DELAY); //abcd
		}
		
		if(USART_elevation_flag())
		{
			change_z(USART_get_z());
		}
	}
	/*
	if(climbing_flag)
	{
		wait(300);
		MPU_update();
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
			wait(100);
			
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
			
			//SERVO_action();
			//wait(100);
		}
		
		// Updating which step in the climbing process it is and taking some
		// steps on the obstacle:
		if (leg3.newPosz > z_in_use && leg4.newPosz > z_in_use && climb_step < 3)
		{
			if(MPURMean - climb_start_slope_r < -0.10 && climb_step2 == 3)
			{
				leg3.newPosz = z_in_use;
				height_change_leg3(z_in_use);
				leg3.climbing = 0;
				leg4.newPosz = z_in_use;
				height_change_leg4(z_in_use);
				leg4.climbing = 0;
				
				move_robot(0, 50, 0);
			}
			else
			{
				climb_step = 3;
				climb_step2 = 1;
				
				climb_counter = 0;
				move_robot(0,50,100);
				wait(100);
				move_robot(0,50,100);
				wait(100);
				while(fabs(MPU_get_y() - climb_start_control) > 0.10)
				{
					MPU_update();
					if (MPU_get_y() - climb_start_control > 0)
					{
						move_robot(0, 40, 0);
					}
					else
					{
						move_robot(0, 60, 0);
					}
				}
				
				wait(100);
				climb_start_slope_r = MPU_get_r();
				climb_start_slope_p = MPU_get_p();
			}
		}
		else if (leg2.newPosz > z_in_use && leg5.newPosz > z_in_use && climb_step < 3)
		{
			climb_step = 2;
		}
		else if (leg1.newPosz > z_in_use && leg6.newPosz > z_in_use && climb_step < 3)
		{
			climb_step = 1;
		}
		
		
		if (leg1.newPosz < z_in_use + 20 && 
			leg6.newPosz < z_in_use + 20 && 
			climb_step2 == 1)
		{
			climb_step2 = 2;
			
			MPU_update();
		}
		else if (leg2.newPosz < z_in_use + 20 && 
			leg5.newPosz < z_in_use + 20 && 
			climb_step2 == 2)
		{
			climb_step2 = 3;
			climb_counter = 0;
			MPU_update();
		}
		else if (leg3.newPosz < z_in_use + 20 &&
			leg4.newPosz < z_in_use + 20 &&
			climb_step2 == 3)
		{
			climb_step = 0;
			climb_step2 = 0;
			climbing_flag = 0;
			servo_speed = 200;
		}
		
		if(fmod(leg1.onObstacle + leg2.onObstacle + leg3.onObstacle + 
			leg4.onObstacle + leg5.onObstacle + leg6.onObstacle, 2) == 0)
		{
			rotation_flag = 1;
		}
	}
	*/
}

/*
void leg_move_down(struct LegData* leg)
{
	leg->newPosz -=10;
	tempz = leg->newPosz;
	// To count new x and y positions: sign*(120 + 0.5 *(new_z - z0))/divider + term
	//leg->newPosx -= 5;
	//leg->newPosy += leg->side*5;
	tempx = leg->newPosx;
	tempy = leg->newPosy;
	
	calc_d(tempx, tempy, tempz);
	leg->temp2AngleGamma = get_gamma(tempx, tempy);
	leg->temp2AngleBeta = get_beta();
	leg->temp2AngleAlpha = get_alpha(tempz);
	leg->goalAngleAlpha = leg->side *(leg->temp2AngleAlpha + femurAngleAddition);
	leg->goalAngleBeta = leg->side*(-leg->temp2AngleBeta + tibiaAngleAddition);
	leg->goalAngleGamma = leg->temp2AngleGamma;
	SERVO_buffer_position(leg->servoAlpha, leg->goalAngleAlpha,servo_speed);
	SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,servo_speed);
	SERVO_buffer_position(leg->servoGamma, leg->goalAngleGamma,servo_speed);
}

void leg_check_down(struct LegData* leg)
{
	//update_leg_info(leg);
	wait(100);
	MPU_get_mean();
	if(fabsf(MPURMean - climb_start_slope_r) > 0.05 || 
		fabsf(MPUPMean - climb_start_slope_p) > 0.05 || 
		leg->newPosz < z_in_use - 10)
	{
		//leg->newPosz += 20;
		//leg->newPosx -= 10;
		
		// The leg is at the obstacle:
		if (leg->newPosz >= z_in_use - 10)
		{
			obstacle_height = z_in_use + 65; 
			tempz = obstacle_height;
			leg->climbing = 0;
			leg->newPosz = obstacle_height;
			leg->onObstacle = 1;
		}
		else // The leg has found the ground:
		{
			tempz = z_in_use;
			leg->newPosz = tempz;
			leg->climbing = 0;
			leg->onObstacle = 0;
		}
		
		tempx = leg->newPosx;
		tempy = leg->newPosy;
		
		calc_d(tempx, tempy, tempz);
		leg->temp2AngleBeta = get_beta();
		leg->temp2AngleAlpha = get_alpha(tempz);
		leg->goalAngleAlpha = leg->side * (leg->temp2AngleAlpha + femurAngleAddition);
		leg->goalAngleBeta = leg->side * (-leg->temp2AngleBeta + tibiaAngleAddition);
	 
		SERVO_buffer_position(leg->servoAlpha,leg->goalAngleAlpha,100);
		SERVO_buffer_position(leg->servoBeta, leg->goalAngleBeta,100);
		
		SERVO_action();
		wait(100);
		
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
			
			case 1:
			height_change_leg4(tempz);
			break;
			
			case 13:
			height_change_leg5(tempz);
			break;
			
			case 7:
			height_change_leg6(tempz);
			break;
			
			default:
			break;
		}
	}
	
	else //if(legsNotDown == 0 && leg->climbing == 1)
	{
		legsNotDown = 1;
	}
}

void climb()
{
	climbing_flag = 1;
	height = 70;
	climb_step = 0;
	climb_step2 = 0;
	climb_counter = 0;
	
	z_in_use = -150;
	obstacle_height = z_in_use;
	change_z(z_in_use);
	
	move_to_std();
	servo_speed = 100;
	
	MPU_update();
	climb_start_control = MPU_get_y();
	climb_start_slope_r = MPU_get_r();
	climb_start_slope_p = MPU_get_p();
	
	while(climbing_flag)
	{
		if(leg6.lift != 1 && climb_step == 0 && obstacle_height == z_in_use)
		{
			leg6.climbing = 1;
			leg6.newPosz = z_in_use + height;
			height_change_leg6(obstacle_height);
		}
		else if(leg6.lift != 1 && climb_step == 0) // If leg1 has found the obstacle.
		{
			height_change_leg6(obstacle_height);
			leg6.climbing = 4;
			leg6.newPosz = obstacle_height;
			leg6.onObstacle = 1;
		}
		else if (leg1.lift != 1 && climb_step == 0 && obstacle_height == z_in_use)
		{
			leg1.climbing = 1;
			leg1.newPosz = z_in_use + height;
			height_change_leg1(obstacle_height);
		}
		else if (leg1.lift != 1 && climb_step == 0) // If leg6 has found the obstacle.
		{
			height_change_leg1(obstacle_height);
			leg1.climbing = 4;
			leg1.newPosz = obstacle_height;
			leg1.onObstacle = 1;
		}
		else if (leg2.lift != 1 && climb_step == 1 && leg5.newPosz == obstacle_height)
		{
			height_change_leg2(obstacle_height);
			leg2.climbing = 4;
			leg2.newPosz = obstacle_height;
			leg2.onObstacle = 1;
		}
		else if (leg2.lift != 1 && climb_step == 1) // Leg5 has not found the obstacle.
		{
			leg2.climbing = 1;
			leg2.newPosz = obstacle_height;
		}
		else if (leg5.lift != 1 && climb_step == 1 && leg2.newPosz ==obstacle_height)
		{
			height_change_leg5(obstacle_height);
			leg5.climbing = 4;
			leg5.newPosz = obstacle_height;
			leg5.onObstacle = 1;
		}
		else if (leg5.lift != 1 && climb_step == 1)
		{
			leg5.climbing = 1;
			leg5.newPosz = obstacle_height;
		}
		else if(leg4.newPosz > z_in_use + 40 && leg3.newPosz > z_in_use + 40)
		{
			climb_step = 3;
			
			height_change_leg4(obstacle_height);
			leg4.climbing = 0;
			leg4.newPosz = obstacle_height;
			
			height_change_leg3(obstacle_height);
			leg3.climbing = 0;
			leg3.newPosz = obstacle_height;
		}
		else if (leg4.lift != 1 && climb_step == 2 && leg3.newPosz == obstacle_height)
		{
			height_change_leg4(obstacle_height);
			leg4.climbing = 4;
			leg4.newPosz = obstacle_height;
			leg4.onObstacle = 1;
		}
		else if (leg4.lift != 1 && climb_step == 2)
		{
			height_change_leg4(obstacle_height);
			leg4.climbing = 1;
			leg4.newPosz = obstacle_height;
		}
		else if (leg3.lift != 1 && climb_step == 2 && leg4.newPosz == obstacle_height)
		{
			height_change_leg3(obstacle_height);
			leg3.climbing = 4;
			leg3.newPosz = obstacle_height;
			leg3.onObstacle = 1;
		}
		else if (leg3.lift != 1 && climb_step == 2)
		{
			height_change_leg3(obstacle_height);
			leg3.climbing = 1;
			leg3.newPosz = obstacle_height;
		}
		
		// Here the part about climbing down starts. 
		if (leg1.lift != 1 && climb_step2 == 1 && leg6.newPosz == z_in_use)
		{
			leg1.climbing = 2;
			leg1.newPosz = z_in_use;
			height_change_leg1(z_in_use);
			leg1.onObstacle = 0;
		}
		else if (leg1.lift != 1 && climb_step2 == 1)
		{
			leg1.climbing = 1;
			leg1.newPosz = obstacle_height;
		}
		else if (leg6.lift != 1 && climb_step2 == 1 && leg1.newPosz == z_in_use)
		{
			leg6.climbing = 2;
			leg6.newPosz = z_in_use;
			height_change_leg6(z_in_use);
			leg6.onObstacle = 0;
		}
		else if (leg6.lift != 1 && climb_step2 == 1)
		{
			leg6.climbing = 1;
			leg6.newPosz = obstacle_height;
		}
		else if (leg2.lift != 1 && climb_step2 == 2 && leg5.newPosz == z_in_use)
		{
			leg2.climbing = 2;
			leg2.newPosz = z_in_use;
			height_change_leg2(z_in_use);
			leg2.onObstacle = 0;
		}
		else if (leg2.lift != 1 && climb_step2 == 2)
		{
			leg2.climbing = 1;
			leg2.newPosz = obstacle_height;
		}
		else if (leg5.lift != 1 && climb_step2 == 2 && leg2.newPosz == z_in_use)
		{
			leg5.climbing = 2;
			leg5.newPosz = z_in_use;
			height_change_leg5(z_in_use);
			leg5.onObstacle = 0;
		}
		else if (leg5.lift != 1 && climb_step2 == 2)
		{
			leg5.climbing = 1;
			leg5.newPosz = obstacle_height;
		}
		else if (leg4.lift != 1 && climb_step2 == 3 && leg3.newPosz == z_in_use)
		{
			leg4.climbing = 2;
			leg4.newPosz = z_in_use;
			height_change_leg4(z_in_use);
			leg4.onObstacle = 0;
		}
		else if (leg4.lift != 1 && climb_step2 == 3)
		{
			leg4.climbing = 1;
			leg4.newPosz = obstacle_height;
		}
		else if (leg3.lift != 1 && climb_step2 == 3 && leg4.newPosz == z_in_use)
		{
			leg3.climbing = 2;
			leg3.newPosz = z_in_use;
			height_change_leg3(z_in_use);
			leg3.onObstacle = 0;
		}
		else if (leg3.lift != 1 && climb_step2 == 3)
		{
			leg3.climbing = 1;
			leg3.newPosz = obstacle_height;
		}
		
		//For safety when climbing down, if we fall.
		MPU_get_mean();
		if(MPURMean - climb_start_slope_r > 0.10 && climb_step == 1)
		{
			leg1.newPosz = z_in_use;
			height_change_leg1(z_in_use);
			leg6.newPosz = z_in_use;
			height_change_leg6(z_in_use);
			leg2.newPosz = z_in_use;
			height_change_leg2(z_in_use);
			leg2.climbing = 0;
			leg5.newPosz = z_in_use;
			height_change_leg5(z_in_use);
			leg5.climbing = 0;
			climb_step = 0;
			
		}
		else if(MPURMean - climb_start_slope_r > 0.10 && climb_step2 == 1)
		{
			leg1.newPosz = z_in_use;
			height_change_leg1(z_in_use);
			leg1.climbing = 0;
			leg6.newPosz = z_in_use;
			height_change_leg6(z_in_use);
			leg6.climbing = 0;
		}
		else if(climb_step2 == 2 || climb_step == 2)
		{
			climb_counter += 1;
			if(climb_counter > 7)
			{
				leg2.newPosz = z_in_use;
				height_change_leg2(z_in_use);
				leg2.climbing = 0;
				leg5.newPosz = z_in_use;
				height_change_leg5(z_in_use);
				leg5.climbing = 0;
				
				climb_counter = 0;
			}
		}
		else if(MPURMean - climb_start_slope_r < -0.10 && climb_step2 == 3)
		{
			leg3.newPosz = z_in_use;
			height_change_leg3(z_in_use);
			leg3.climbing = 0;
			leg4.newPosz = z_in_use;
			height_change_leg4(z_in_use);
			leg4.climbing = 0;
		}
		else if(climb_step2 == 3)
		{
			climb_counter += 1;
			if(climb_counter > 4)
			{
				leg3.newPosz = z_in_use;
				height_change_leg3(z_in_use);
				leg3.climbing = 0;
				leg4.newPosz = z_in_use;
				height_change_leg4(z_in_use);
				leg4.climbing = 0;
				
				climb_counter = 0;
			}
		}
		
		move_robot(0,50,100);
		wait(200);
		
		climb_rotation_counter = 0;
		while(fabs(MPU_get_y() - climb_start_control) > 0.10 && rotation_flag)
		{
			MPU_update();
			if (MPU_get_y() - climb_start_control > 0)
			{
				move_robot(0, 40, 0);
			}
			else
			{
				move_robot(0, 60, 0);
			}
			climb_rotation_counter += 1;
			if(climb_rotation_counter > 8)
			{
				leg2.newPosz = z_in_use;
				height_change_leg2(z_in_use);
				leg2.climbing = 0;
				leg5.newPosz = z_in_use;
				height_change_leg5(z_in_use);
				leg5.climbing = 0;
				leg3.climbing = 0;
				leg4.climbing = 0;
				
				rotation_flag = 0;
			}
		}
		rotation_flag = 0;
	}
	//take another two steps for good measure and change height to the normal one.
	move_robot(0,50,100);
	wait(30);
	z_in_use = -120;
	change_z(z_in_use);
	move_robot(0,50,100);
	USART_send_climb_done();
}

void move_climb(struct LegData* leg, float n)
{
	tempz = z_in_use + height + 30;
	
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
			
		SERVO_goto(leg->servoAlpha,leg->goalAngleAlpha,200);
		SERVO_goto(leg->servoBeta, leg->goalAngleBeta,200);
		
		wait(200);
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
*/

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
	if(USART_elevation_flag())
	{
		change_z(USART_get_z());
	}
	
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

void turn_degrees(uint16_t degrees, int8_t dir)
{
	float walkSpeedMax = 0;
	float walkSpeedMin = 0;
	if(degrees < 135)
	{
		walkSpeedMax = TURN_WALK_SPEED_MAX;
		walkSpeedMin = TURN_WALK_SPEED_MIN;
	}
	float radians = degrees * M_PI/180;
	float tolerance = TURN_TOLERANCE * M_PI/180;
	wait(10);
	float startAngle = MPU_get_y();
	float newAngle;
	float angleLeft, turnRotation;
	float diff, realDiff;
	
	do
	{
		reset_counter_1();
		wait(10);
		newAngle = MPU_get_y();
		
		diff = newAngle - startAngle;
		
		if(fabs(diff) <= M_PI)
		{
			realDiff = fabs(diff);
		} 
		else if(diff < 0)
		{
			realDiff = M_PI + fmod(diff, M_PI);
		} 
		else 
		{
			realDiff = M_PI - fmod(diff, M_PI);
		}
		angleLeft = radians - realDiff;
		
		
		if(fabs(realDiff) >= M_PI_2)
		{
			radians -= realDiff;
			startAngle = newAngle;
			
			if(radians < 0)
			{
				radians *= -1;
				dir *= -1;
				angleLeft *= -1;
			}
		}
		if(degrees < 135)
		{
			turnRotation = 50 + dir * 50 * angleLeft / TURN_SCALE_FACTOR;
		} else {
			turnRotation = 50 + dir * 50 * angleLeft / M_PI_2;
		}
		
		if(turnRotation > 50 - TURN_MIN_SPEED && turnRotation < 50 + TURN_MIN_SPEED)
		{
			turnRotation = 50 + TURN_MIN_SPEED * dir * angleLeft / (fabs(angleLeft) * 2);
		}
		uint8_t speed = walkSpeedMax * angleLeft / TURN_WALK_SCALE_FACTOR;
		if(speed > walkSpeedMax)
		{
			speed = walkSpeedMax;
		} else if(speed < walkSpeedMin)
		{
			speed = walkSpeedMin;	
		}
		move_robot(0, turnRotation, speed);
	} 
	while(fabs(angleLeft) > tolerance);
	
	USART_send_turn_done();
}


void MPU_get_mean()
{
	wait(10);
	MPU_update();
	float xp = MPU_get_p();
	float xr = MPU_get_r();
	wait(10);
	MPU_update();
	float yp = MPU_get_p();
	float yr = MPU_get_r();
	wait(10);
	MPU_update();
	MPUPMean = (xp+yp+ MPU_get_p())/3;	
	MPURMean = (xr+yr+ MPU_get_r())/3;
}

union Union_floatcast
{
	float f;
	char s[sizeof(float)];
};

float floatCast(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
	union Union_floatcast foo;
	foo.s[0] = b1;
	foo.s[1] = b2;
	foo.s[2] = b3;
	foo.s[3] = b4;
	return foo.f;
}

void update_parameters(uint8_t params[33])
{
	ITERATIONS = params[0];
	SERVO_SPEED = (((uint16_t) params[1]) << 8) | params[2];
	MAX_STEP_LENGTH = floatCast(params[3], params[4], params[5], params[6]) * 10;
	ITERATIONS_DELAY_START = (((uint16_t) params[7]) << 8) | params[8];
	ITERATIONS_DELAY = (((uint16_t) params[9]) << 8) | params[10];
	STEP_HEIGHT = params[11] * 10;
	STEP_HEIGHT_CLIMB = params[12] * 10;
	TURN_WALK_SPEED_MAX = params[13];
	TURN_WALK_SPEED_MIN = params[14];
	TURN_TOLERANCE = floatCast(params[15], params[16], params[17], params[18]);
	TURN_SCALE_FACTOR = floatCast(params[19], params[20], params[21], params[22]) * M_PI / 180;
	TURN_WALK_SCALE_FACTOR = floatCast(params[23], params[24], params[25], params[26]) * M_PI / 180;
	TURN_MIN_SPEED = floatCast(params[27], params[28], params[29], params[30]);
	AUTO_ELEVATION = -((float) ((((uint16_t) params[31]) << 8) | params[32]));
	change_z(AUTO_ELEVATION);
	
	USART_send_value(MAX_STEP_LENGTH);
}

//Ny hårdkodad climb
void climb()
{
	float tempStepLength = MAX_STEP_LENGTH;
	MAX_STEP_LENGTH = 55;
	climbing_flag = 1;
	float tempdir;
	MPU_update();
	climb_start_control = MPU_get_y();
	
	for(int i = 0;i<16;++i)
	{		
		MPU_update();	
		if(climb_start_control-MPU_get_y() <= 0)
		{
			tempdir = 90-300*(climb_start_control-MPU_get_y());		
		}
		else
		{
			tempdir = 300*climb_start_control-MPU_get_y();	
		}
		move_robot(tempdir,50+150*(climb_start_control-MPU_get_y()),100);
	}
	climbing_flag = 0;
	MAX_STEP_LENGTH = tempStepLength;
	USART_send_climb_done();	
}