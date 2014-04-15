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
#include "usart.h"


#define pi (double) 3.14159265
#define sqrt2 (double) 1.41421356

#define x0_1 (double) -150/sqrt2-61.85 //standard x pos for leg 1
#define y0_1 (double) 150/sqrt2+120 //standard y pos for leg 1
#define x0_2 (double) -150-61.85 //standard x pos for leg 2
#define y0_2 (double) 0 //standard y pos for leg 2
#define x0_3 (double) -150/sqrt2-61.85 //standard x pos for leg 3
#define y0_3 (double) -150/sqrt2-120 //standard y pos for leg 3
#define x0_4 (double) 150/sqrt2+61.85 //standard x pos for leg 4
#define y0_4 (double) -150/sqrt2-120 //standard y pos for leg 4
#define x0_5 (double) 150+61.15 //standard x pos for leg 5
#define y0_5 (double) 0 //standard y pos for leg 5
#define x0_6 (double) 150/sqrt2+61.85 //standard x pos for leg 6
#define y0_6 (double) 150/sqrt2+120 //standard y pos for leg 6
#define z0 (double) -80

int speed = 200;

#define side1 1
#define side2 1
#define side3 1
#define side4 -1
#define side5 -1
#define side6 -1

#define femurAngleAddition (double)0.231 //0.2426
#define tibiaAngleAddition (double)0.812 //(-3.1415/6)




#define frontLegDistanfce (double)200
#define centerToFrontLegsY (double)120
#define centerToSideLegs (double)100
#define centerToFrontLegs (double)135
#define centerToFrontLegsX (double)61.85

//Jonas function for robot movement
/*
double z0 = -120;
int speed = 200;

void moveRobot(double direction,double distanfce, double rotation, double z, int servoSpeed, double rotationX, double rotationY)
{
	double sinfrotation = 0;//sinf(rotation);
	double cosfrotation = 1;//cosf(rotation);
	double sinfdirection = 0;//sinf(direction);
	double cosfdirection = 1;//cosf(direction);
	double sinfaroundx = 0;//sinf(rotationX);
	double sinfaroundy = 0;//cosf(rotationY);
	
	//First state-------------------
	servoGoto(4, 3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(10, 3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(15, -3.1415/2, servoSpeed);
	_delay_ms(200);
	moveLeg1too(x0_1*cosfrotation-y0_1*sinfrotation-sinfdirection*distanfce, y0_1*cosfrotation-x0_1*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg3too(x0_3*cosfrotation-y0_3*sinfrotation-sinfdirection*distanfce, y0_3*cosfrotation-x0_3*sinfrotation+cosfdirection*distanfce, -(z+sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg5too(x0_5*cosfrotation-y0_5*sinfrotation-sinfdirection*distanfce, y0_5*cosfrotation+x0_5*sinfrotation+cosfdirection*distanfce, -(z-centerToSideLegs*sinfaroundy), servoSpeed);
	moveLeg6too(x0_6, y0_6, -(z-sinfaroundy*centerToFrontLegsX+sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg4too(x0_4, y0_4, -(z-sinfaroundy*centerToFrontLegsX-sinfaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg2too(x0_2, y0_2, -(z+centerToSideLegs*sinfaroundy), servoSpeed);	
	_delay_ms(300);
		
	//Second state-------------------
	servoGoto(3, -3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(9, -3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(16, 3.1415/2, servoSpeed);
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
int Leg1_lift = 1;
int Leg2_lift = -1;
int Leg3_lift = 1;
int Leg4_lift = -1;
int Leg5_lift = 1;
int Leg6_lift = -1;

#define Gamma0 (double) 0
#define Beta0 (double) 1.9425
#define Alpha0 (double) 0.7159

double z = z0;

//Position där föregående steg slutade
double Leg1_prev_posx;
double Leg1_prev_posy;

double Leg2_prev_posx;
double Leg2_prev_posy;

double Leg3_prev_posx;
double Leg3_prev_posy;

double Leg4_prev_posx;
double Leg4_prev_posy;

double Leg5_prev_posx;
double Leg5_prev_posy;

double Leg6_prev_posx;
double Leg6_prev_posy;


//Position där nuvarande steg ska sluta
double Leg1_new_posx;
double Leg1_new_posy;

double Leg2_new_posx;
double Leg2_new_posy;

double Leg3_new_posx;
double Leg3_new_posy;

double Leg4_new_posx;
double Leg4_new_posy;

double Leg5_new_posx;
double Leg5_new_posy;

double Leg6_new_posx;
double Leg6_new_posy;


//Vinklar där föregående steg slutade
double Leg1_prev_angleGamma = Gamma0;
double Leg1_prev_angleBeta = Beta0;
double Leg1_prev_angleAlpha = Alpha0;

double Leg2_prev_angleGamma = Gamma0;
double Leg2_prev_angleBeta = Beta0;
double Leg2_prev_angleAlpha = Alpha0;

double Leg3_prev_angleGamma = Gamma0;
double Leg3_prev_angleBeta = Beta0;
double Leg3_prev_angleAlpha = Alpha0;

double Leg4_prev_angleGamma = Gamma0;
double Leg4_prev_angleBeta = Beta0;
double Leg4_prev_angleAlpha = Alpha0;

double Leg5_prev_angleGamma = Gamma0;
double Leg5_prev_angleBeta = Beta0;
double Leg5_prev_angleAlpha = Alpha0;

double Leg6_prev_angleGamma = Gamma0;
double Leg6_prev_angleBeta = Beta0;
double Leg6_prev_angleAlpha = Alpha0;

//Benkoordinater som nuvarande steget ska gå till
double Leg1_new_angleGamma = Gamma0;
double Leg1_new_angleBeta = Beta0;
double Leg1_new_angleAlpha = Alpha0;

double Leg2_new_angleGamma = Gamma0;
double Leg2_new_angleBeta = Beta0;
double Leg2_new_angleAlpha = Alpha0;

double Leg3_new_angleGamma = Gamma0;
double Leg3_new_angleBeta = Beta0;
double Leg3_new_angleAlpha = Alpha0;

double Leg4_new_angleGamma = Gamma0;
double Leg4_new_angleBeta = Beta0;
double Leg4_new_angleAlpha = Alpha0;

double Leg5_new_angleGamma = Gamma0;
double Leg5_new_angleBeta = Beta0;
double Leg5_new_angleAlpha = Alpha0;

double Leg6_new_angleGamma = Gamma0;
double Leg6_new_angleBeta = Beta0;
double Leg6_new_angleAlpha = Alpha0;


//Längd från origo till standardposition för ben 1,3,4,6
double std_lenght = sqrtf(x0_1*x0_1 + y0_1*y0_1);

//Max möjliga steglängd från grundpositionen
double max_step_lenght = 70;

//Middle-postioner
double Leg1_new_middle_posx;
double Leg1_new_middle_posy;

double Leg2_new_middle_posx;
double Leg2_new_middle_posy;

double Leg3_new_middle_posx;
double Leg3_new_middle_posy;

double Leg4_new_middle_posx;
double Leg4_new_middle_posy;

double Leg5_new_middle_posx;
double Leg5_new_middle_posy;

double Leg6_new_middle_posx;
double Leg6_new_middle_posy;


//Middle-vinklar
double Leg1_new_middle_angleGamma;
double Leg1_new_middle_angleBeta;
double Leg1_new_middle_angleAlpha;

double Leg2_new_middle_angleGamma;
double Leg2_new_middle_angleBeta;
double Leg2_new_middle_angleAlpha;

double Leg3_new_middle_angleGamma;
double Leg3_new_middle_angleBeta;
double Leg3_new_middle_angleAlpha;

double Leg4_new_middle_angleGamma;
double Leg4_new_middle_angleBeta;
double Leg4_new_middle_angleAlpha;

double Leg5_new_middle_angleGamma;
double Leg5_new_middle_angleBeta;
double Leg5_new_middle_angleAlpha;

double Leg6_new_middle_angleGamma;
double Leg6_new_middle_angleBeta;
double Leg6_new_middle_angleAlpha;


//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
void moveRobot(int direction, int rotation, int speed)
{

	//Nytt steg ska räknas ut. Nya stegets vinklar sätts till föregående stegets vinklar.
	Leg1_prev_angleGamma = Leg1_new_angleGamma;
	Leg1_prev_angleBeta = Leg1_new_angleBeta;
	Leg1_prev_angleAlpha = Leg1_new_angleAlpha;

	Leg2_prev_angleGamma = Leg2_new_angleGamma;
	Leg2_prev_angleBeta = Leg2_new_angleBeta;
	Leg2_prev_angleAlpha = Leg2_new_angleAlpha;

	Leg3_prev_angleGamma = Leg3_new_angleGamma;
	Leg3_prev_angleBeta = Leg3_new_angleBeta;
	Leg3_prev_angleAlpha = Leg3_new_angleAlpha;

	Leg4_prev_angleGamma = Leg4_new_angleGamma;
	Leg4_prev_angleBeta = Leg4_new_angleBeta;
	Leg4_prev_angleAlpha = Leg4_new_angleAlpha;

	Leg5_prev_angleGamma = Leg5_new_angleGamma;
	Leg5_prev_angleBeta = Leg5_new_angleBeta;
	Leg5_prev_angleAlpha = Leg5_new_angleAlpha;

	Leg6_prev_angleGamma = Leg6_new_angleGamma;
	Leg6_prev_angleBeta = Leg6_new_angleBeta;
	Leg6_prev_angleAlpha = Leg6_new_angleAlpha;


	//x och y riktning för förflyttning, skalad med hastigheten
	double x_direction = -sinf((double)direction * pi / 90) * (double)speed / 100;
	double y_direction = cosf((double)direction * pi / 90) * (double)speed / 100;


	//x och y riktning för rotation, skalad med rotationshastighet
	double x_rot1 = ((double)rotation / 50 -1) * y0_1 / std_lenght;
	double y_rot1 = ((double)rotation / 50 -1) * x0_1 / std_lenght;

	double x_rot2 = 0;
	double y_rot2 = (double)rotation / 50 - 1;

	double x_rot3 = ((double)rotation / 50 -1) * (- y0_1 / std_lenght);
	double y_rot3 = ((double)rotation / 50 -1) * x0_1 / std_lenght;

	double x_rot4 = ((double)rotation / 50 -1) * (- y0_1 / std_lenght);
	double y_rot4 = ((double)rotation / 50 -1) * (- x0_1 / std_lenght);

	double x_rot5 = 0;
	double y_rot5 = -(double)rotation / 50 + 1;

	double x_rot6 = ((double)rotation / 50 -1) * y0_1 / std_lenght;
	double y_rot6 = ((double)rotation / 50 -1) * (- x0_1 / std_lenght);


	//Addera förflyttningarna och sätter Step_max till längsta stegets längd
	double Step_max = sqrtf(powf((x_direction + x_rot1),(double)2) + powf((y_direction + y_rot1),(double)2));
	double Step_test = sqrtf(powf((x_direction + x_rot2),(double)2) + powf((y_direction + y_rot2),(double)2));
	Step_max = fmaxf(Step_max, Step_test);

	Step_test = sqrtf(powf((x_direction + x_rot3),(double)2) + powf((y_direction + y_rot3),(double)2));
	Step_max = fmaxf(Step_max, Step_test);

	Step_test = sqrtf(powf((x_direction + x_rot4),2) + powf((y_direction + y_rot4),2));
	Step_max = fmaxf(Step_max, Step_test);

	Step_test = sqrtf(powf((x_direction + x_rot5),2) + powf((y_direction + y_rot5),2));
	Step_max = fmaxf(Step_max, Step_test);

	Step_test = sqrtf(powf((x_direction + x_rot6),2) + powf((y_direction + y_rot6),2));
	Step_max = fmaxf(Step_max, Step_test);


	//Stegskalning för att inte alltid ta max längd på steg;
	double Step_scaling = fmaxf(speed/100, fabsf(rotation/50-1));


	//Uppdatera vinklar för nya steget


	double Leg1_new_posx_wb = (double)Leg1_lift * max_step_lenght / Step_max * (x_direction + x_rot1) * Step_scaling;
	double Leg1_new_posy_wb = (double)Leg1_lift * max_step_lenght / Step_max * (y_direction + y_rot1) * Step_scaling;
	double Leg1_new_posz = z;
	double Leg1_new_posx = basis_change_Leg1x(Leg1_new_posx_wb,Leg1_new_posy_wb);
	double Leg1_new_posy = basis_change_Leg1y(Leg1_new_posx_wb,Leg1_new_posy_wb);
	Calc_d(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	Leg1_new_angleGamma = Calc_gamma(Leg1_new_posx, Leg1_new_posy);
	Leg1_new_angleBeta = Calc_Beta();
	Leg1_new_angleAlpha = Calc_Alpha(Leg1_new_posz);


	double Leg2_new_posx_wb = Leg2_lift * max_step_lenght / Step_max * (x_direction + x_rot2) * Step_scaling;
	double Leg2_new_posy_wb = Leg2_lift * max_step_lenght / Step_max * (y_direction + y_rot2) * Step_scaling;
	double Leg2_new_posz = z;
	double Leg2_new_posx = basis_change_Leg2x(Leg2_new_posx_wb);
	double Leg2_new_posy = basis_change_Leg2y(Leg2_new_posy_wb);
	Calc_d(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	Leg2_new_angleGamma = Calc_gamma(Leg2_new_posx, Leg2_new_posy);
	Leg2_new_angleBeta = Calc_Beta();
	Leg2_new_angleAlpha = Calc_Alpha(Leg2_new_posz);


	double Leg3_new_posx_wb = Leg3_lift * max_step_lenght / Step_max * (x_direction + x_rot3) * Step_scaling;
	double Leg3_new_posy_wb = Leg3_lift * max_step_lenght / Step_max * (y_direction + y_rot3) * Step_scaling;
	double Leg3_new_posz = z;
	double Leg3_new_posx = basis_change_Leg3x(Leg3_new_posx_wb,Leg3_new_posy_wb);
	double Leg3_new_posy = basis_change_Leg3y(Leg3_new_posx_wb,Leg3_new_posy_wb);
	Calc_d(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	Leg3_new_angleGamma = Calc_gamma(Leg3_new_posx, Leg3_new_posy);
	Leg3_new_angleBeta = Calc_Beta();
	Leg3_new_angleAlpha = Calc_Alpha(Leg3_new_posz);


	double Leg4_new_posx_wb = Leg4_lift * max_step_lenght / Step_max * (x_direction + x_rot4) * Step_scaling;
	double Leg4_new_posy_wb = Leg4_lift * max_step_lenght / Step_max * (y_direction + y_rot4) * Step_scaling;
	double Leg4_new_posz = z;
	double Leg4_new_posx = basis_change_Leg4x(Leg4_new_posx_wb,Leg4_new_posy_wb);
	double Leg4_new_posy = basis_change_Leg4y(Leg4_new_posx_wb,Leg4_new_posy_wb);
	Calc_d(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	Leg4_new_angleGamma = Calc_gamma(Leg4_new_posx, Leg4_new_posy);
	Leg4_new_angleBeta = Calc_Beta();
	Leg4_new_angleAlpha = Calc_Alpha(Leg4_new_posz);


	double Leg5_new_posx_wb = Leg5_lift * max_step_lenght / Step_max * (x_direction + x_rot5) * Step_scaling;
	double Leg5_new_posy_wb = Leg5_lift * max_step_lenght / Step_max * (y_direction + y_rot5) * Step_scaling;
	double Leg5_new_posz = z;
	double Leg5_new_posx = basis_change_Leg5x(Leg5_new_posx_wb);
	double Leg5_new_posy = basis_change_Leg5y(Leg5_new_posy_wb);
	Calc_d(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	Leg5_new_angleGamma = Calc_gamma(Leg5_new_posx, Leg5_new_posy);
	Leg5_new_angleBeta = Calc_Beta();
	Leg5_new_angleAlpha = Calc_Alpha(Leg5_new_posz);


	double Leg6_new_posx_wb = Leg6_lift * max_step_lenght / Step_max * (x_direction + x_rot6) * Step_scaling;
	double Leg6_new_posy_wb = Leg6_lift * max_step_lenght / Step_max * (y_direction + y_rot6) * Step_scaling;
	double Leg6_new_posz = z;
	double Leg6_new_posx = basis_change_Leg6x(Leg6_new_posx_wb,Leg6_new_posy_wb);
	double Leg6_new_posy = basis_change_Leg6y(Leg6_new_posx_wb,Leg6_new_posy_wb);
	Calc_d(Leg6_new_posx, Leg6_new_posy, Leg6_new_posz);
	Leg6_new_angleGamma = Calc_gamma(Leg6_new_posx, Leg6_new_posy);
	Leg6_new_angleBeta = Calc_Beta();
	Leg6_new_angleAlpha = Calc_Alpha(Leg6_new_posz);


	if(Leg1_lift == -1)
	{
		Leg1_new_middle_posx = Leg1_prev_posx + (Leg1_new_posx - Leg1_prev_posx)/2;
		Leg1_new_middle_posy = Leg1_prev_posy + (Leg1_new_posy - Leg1_prev_posy)/2;
		Calc_d(Leg1_new_middle_posx, Leg1_new_middle_posy, z);
		Leg1_new_middle_angleGamma = Calc_gamma(Leg1_new_middle_posx, Leg1_new_middle_posy);
		Leg1_new_middle_angleBeta = Calc_Beta();
		Leg1_new_middle_angleAlpha = Calc_Alpha(Leg1_new_posz);
	}
	
	if(Leg2_lift == -1)
	{
		Leg2_new_middle_posx = Leg2_prev_posx + (Leg2_new_posx - Leg2_prev_posx)/2;
		Leg2_new_middle_posy = Leg2_prev_posy + (Leg2_new_posy - Leg2_prev_posy)/2;
		Calc_d(Leg2_new_middle_posx, Leg2_new_middle_posy, z);
		Leg2_new_middle_angleGamma = Calc_gamma(Leg2_new_middle_posx, Leg2_new_middle_posy);
		Leg2_new_middle_angleBeta = Calc_Beta();
		Leg2_new_middle_angleAlpha = Calc_Alpha(Leg2_new_posz);
	}
	
	if(Leg3_lift == -1)
	{
		Leg3_new_middle_posx = Leg3_prev_posx + (Leg3_new_posx - Leg3_prev_posx)/2;
		Leg3_new_middle_posy = Leg3_prev_posy + (Leg3_new_posy - Leg3_prev_posy)/2;
		Calc_d(Leg3_new_middle_posx, Leg3_new_middle_posy,z);
		Leg3_new_middle_angleGamma = Calc_gamma(Leg3_new_middle_posx, Leg3_new_middle_posy);
		Leg3_new_middle_angleBeta = Calc_Beta();
		Leg3_new_middle_angleAlpha = Calc_Alpha(Leg3_new_posz);
	}
	
	if(Leg4_lift == -1)
	{
		Leg4_new_middle_posx = Leg4_prev_posx + (Leg4_new_posx - Leg4_prev_posx)/2;
		Leg4_new_middle_posy = Leg4_prev_posy + (Leg4_new_posy - Leg4_prev_posy)/2;
		Calc_d(Leg4_new_middle_posx, Leg4_new_middle_posy, z);
		Leg4_new_middle_angleGamma = Calc_gamma(Leg4_new_middle_posx, Leg4_new_middle_posy);
		Leg4_new_middle_angleBeta = Calc_Beta();
		Leg4_new_middle_angleAlpha = Calc_Alpha(Leg4_new_posz);
	}
	
	if(Leg5_lift == -1)
	{
		Leg5_new_middle_posx = Leg5_prev_posx + (Leg5_new_posx - Leg5_prev_posx)/2;
		Leg5_new_middle_posy = Leg5_prev_posy + (Leg5_new_posy - Leg5_prev_posy)/2;
		Calc_d(Leg5_new_middle_posx, Leg5_new_middle_posy,z);
		Leg5_new_middle_angleGamma = Calc_gamma(Leg5_new_middle_posx, Leg5_new_middle_posy);
		Leg5_new_middle_angleBeta = Calc_Beta();
		Leg5_new_middle_angleAlpha = Calc_Alpha(Leg5_new_posz);
	}
	
	if(Leg6_lift == -1)
	{
		Leg6_new_middle_posx = Leg6_prev_posx + (Leg6_new_posx - Leg6_prev_posx)/2;
		Leg6_new_middle_posy = Leg6_prev_posy + (Leg6_new_posy - Leg6_prev_posy)/2;
		Calc_d(Leg6_new_middle_posx, Leg6_new_middle_posy,z);
		Leg6_new_middle_angleGamma = Calc_gamma(Leg6_new_middle_posx, Leg6_new_middle_posy);
		Leg6_new_middle_angleBeta = Calc_Beta();
		Leg6_new_middle_angleAlpha = Calc_Alpha(Leg6_new_posz);
	}
	


	Leg_motion();

}


void Leg_motion()
{
	if (Leg1_lift == -1)
	{
		servoGoto(8,Leg1_new_middle_angleGamma,100);
		servoGoto(10,side1*(Leg1_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(12,side1*(-Leg1_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(8,Leg1_new_angleGamma + (Leg1_new_angleGamma-Leg1_prev_angleBeta)/2 ,100);
		servoGoto(10,side1*(Leg1_new_angleAlpha + (Leg1_new_angleAlpha-Leg1_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(12,side1*(-Leg1_prev_angleBeta - (Leg1_new_angleBeta-Leg1_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}
	_delay_ms(1);

	if (Leg2_lift == -1)
	{
		servoGoto(14,Leg2_new_middle_angleGamma,100);
		servoGoto(16,side2*(Leg2_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(18,side2*(-Leg2_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(14,Leg2_new_angleGamma + (Leg2_new_angleGamma-Leg2_prev_angleBeta)/2 ,100);
		servoGoto(16,side2*(Leg2_new_angleAlpha + (Leg2_new_angleAlpha-Leg2_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(18,side2*(-Leg2_prev_angleBeta - (Leg2_new_angleBeta-Leg2_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}
	_delay_ms(1);

	if (Leg3_lift == -1)
	{
		servoGoto(2,Leg3_new_middle_angleGamma,100);
		servoGoto(4,side3*(Leg3_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(6,side3*(-Leg3_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(2,Leg3_new_angleGamma + (Leg3_new_angleGamma-Leg3_prev_angleBeta)/2 ,100);
		servoGoto(4,side3*(Leg3_new_angleAlpha + (Leg3_new_angleAlpha-Leg3_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(6,side3*(-Leg3_prev_angleBeta - (Leg3_new_angleBeta-Leg3_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}
	_delay_ms(1);

	if (Leg4_lift == -1)
	{
		servoGoto(1,Leg4_new_middle_angleGamma,100);
		servoGoto(3,side4*(Leg4_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(5,side4*(-Leg4_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(1,Leg4_new_angleGamma + (Leg4_new_angleGamma-Leg4_prev_angleBeta)/2 ,100);
		servoGoto(3,side4*(Leg4_new_angleAlpha + (Leg4_new_angleAlpha-Leg4_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(5,side4*(-Leg4_prev_angleBeta - (Leg4_new_angleBeta-Leg4_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}
	_delay_ms(1);

	if (Leg5_lift == -1)
	{
		servoGoto(13,Leg5_new_middle_angleGamma,100);
		servoGoto(15,side5*(Leg5_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(17,side5*(-Leg5_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(13,Leg5_new_angleGamma + (Leg5_new_angleGamma-Leg5_prev_angleBeta)/2 ,100);
		servoGoto(15,side5*(Leg5_new_angleAlpha + (Leg5_new_angleAlpha-Leg5_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(17,side5*(-Leg5_prev_angleBeta - (Leg5_new_angleBeta-Leg5_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}
_delay_ms(1);

	if (Leg6_lift == -1)
	{
		servoGoto(7,Leg6_new_middle_angleGamma,100);
		servoGoto(9,side6*(Leg6_new_middle_angleAlpha + femurAngleAddition),100);
		servoGoto(11,side6*(-Leg6_new_middle_angleBeta + tibiaAngleAddition),100);
	}
	else
	{
		servoGoto(7,Leg6_new_angleGamma + (Leg6_new_angleGamma-Leg6_prev_angleBeta)/2 ,100);
		servoGoto(9,side6*(Leg6_new_angleAlpha + (Leg6_new_angleAlpha-Leg6_prev_angleAlpha)/2 + 0.5 + femurAngleAddition),100);
		servoGoto(11,side6*(-Leg6_prev_angleBeta - (Leg6_new_angleBeta-Leg6_prev_angleBeta)/2 + tibiaAngleAddition),100);
	}

	_delay_ms(2000);

	servoGoto(8,Leg1_new_angleGamma,100);
	servoGoto(10,side1*(Leg1_new_angleAlpha + femurAngleAddition),100);
	servoGoto(12,side1*(-Leg1_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(1);

	servoGoto(14,Leg2_new_angleGamma,100);
	servoGoto(16,side2*(Leg2_new_angleAlpha + femurAngleAddition),100);
	servoGoto(18,side2*(-Leg2_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(1);

	servoGoto(2,Leg3_new_angleGamma,100);
	servoGoto(4,side3*(Leg3_new_angleAlpha + femurAngleAddition),100);
	servoGoto(6,side3*(-Leg3_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(1);

	servoGoto(1,Leg4_new_angleGamma,100);
	servoGoto(3,side4*(Leg4_new_angleAlpha + femurAngleAddition),100);
	servoGoto(5,side4*(-Leg4_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(1);

	servoGoto(13,Leg5_new_angleGamma,100);
	servoGoto(15,side5*(Leg5_new_angleAlpha + femurAngleAddition),100);
	servoGoto(17,side5*(-Leg5_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(1);

	servoGoto(7,Leg6_new_angleGamma,100);
	servoGoto(9,side6*(Leg6_new_angleAlpha + femurAngleAddition),100);
	servoGoto(11,side6*(-Leg6_new_angleBeta + tibiaAngleAddition),100);

	_delay_ms(2000);
}


int main(void)
{
	DDRD |= (1<<PORTD5); //init LED
	//servoTx;
	
	servoRetrunLevel(BROADCASTING_ID, 1); //turns off return packets
	
	sei();
	initServoSerial(); //Init servos
	USART_init();
	//initvar();
	
	/*
	double alpha = 3.1415/4;
	double beta = 3.1415/2.2;
	int speed = 180;
	
	//dummy code that puts robot in stanfdard position
	
	servoGoto(1, 0, speed);
	servoGoto(2, 0, speed);
	servoGoto(7, 0, speed);
	servoGoto(8, 0, speed);
	servoGoto(13, 0, speed);
	servoGoto(14, 0, speed);
	
	_delay_ms(100);
	
	servoGoto(15, -alpha, speed);
	servoGoto(3, -alpha, speed);
	servoGoto(9, -alpha, speed);
	servoGoto(16, alpha, speed);
	servoGoto(4, alpha, speed);
	servoGoto(10, alpha, speed);
	
	_delay_ms(100);
	
	servoGoto(12, -beta, speed);
	servoGoto(18, -beta, speed);
	servoGoto(6, -beta, speed);
	servoGoto(5, beta, speed);
	servoGoto(11, beta, speed);
	servoGoto(17, beta, speed);
	
	_delay_ms(5000);
	*/
	moveLeg1too(x0_1, y0_1, z0, speed);
	moveLeg2too(x0_2, y0_2, z0, speed);
	moveLeg3too(x0_3, y0_3, z0, speed);
	moveLeg4too(x0_4, y0_4, z0, speed);
	moveLeg5too(x0_5, y0_5, z0, speed);
	moveLeg6too(x0_6, y0_6, z0, speed);
	
	
	//_delay_ms(5000);
	
	//moveRobotTob(0,50,100);
    while(1)
    {
		moveRobot(0,50,100);
		Leg1_lift = -Leg1_lift;
		Leg2_lift = -Leg2_lift;
		Leg3_lift = -Leg3_lift;
		Leg4_lift = -Leg4_lift;
		Leg5_lift = -Leg5_lift;
		Leg6_lift = -Leg6_lift;
		
		
		//USART_DecodeRxFIFO();

		//moveRobot((double)0,(double)40,(double)0,(double)120,(int)100,(double)0,(double)0);

        //TODO:: Please write your application code 
    }
}