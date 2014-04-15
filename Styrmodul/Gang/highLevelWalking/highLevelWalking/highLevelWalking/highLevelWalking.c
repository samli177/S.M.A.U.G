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


#define pi 3.14159265

#define x0_1 -140 //stanfdard x pos for leg 1
#define y0_1  180 //stanfdard y pos for leg 1
#define x0_2  -220 //stanfdard x pos for leg 2
#define y0_2  0 //stanfdard y pos for leg 2
#define x0_3  -140 //stanfdard x pos for leg 3
#define y0_3  -180 //stanfdard y pos for leg 3
#define x0_4 140 //stanfdard x pos for leg 4
#define y0_4  -180 //stanfdard y pos for leg 4
#define x0_5  220 //stanfdard x pos for leg 5
#define y0_5  0 //stanfdard y pos for leg 5
#define x0_6  140 //stanfdard x pos for leg 6
#define y0_6  180 //stanfdard y pos for leg 6





#define frontLegDistanfce (float)200
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85
/*
float x0_1 = -240; //stanfdard x pos for leg 1
float y0_1 = 240; //stanfdard y pos for leg 1
float x0_2 = -320; //stanfdard x pos for leg 2
float y0_2 = 0; //stanfdard y pos for leg 2
float x0_3 = -240; //stanfdard x pos for leg 3
float y0_3 = -240; //stanfdard y pos for leg 3
float x0_4 = 240; //stanfdard x pos for leg 4
float y0_4 = -240; //stanfdard y pos for leg 4
float x0_5 = 320; //stanfdard x pos for leg 5
float y0_5 = 0; //stanfdard y pos for leg 5
float x0_6 = 240; //stanfdard x pos for leg 6
<<<<<<< HEAD
float y0_6 = 250; //stanfdard y pos for leg 6
*/
/*
float Gamma0_1 = 0;
float Alpha0_1 = -3.1415/4;
float Beta0_1 = -3.1415/2.2;

float Gamma0_2 = 0;
float Alpha0_2 = -3.1415/4;
float Beta0_2 = -3.1415/2.2;

float Gamma0_3 = 0;
float Alpha0_3 = -3.1415/4;
float Beta0_3 = -3.1415/2.2;

float Gamma0_4 = 0;
float Alpha0_4 = -3.1415/4;
float Beta0_4 = -3.1415/2.2;

float Gamma0_5 = 0;
float Alpha0_5 = 3.1415/4;
float Beta0_5 = 3.1415/2.2;

float Gamma0_6 = 0;
float Alpha0_6 = 3.1415/4;
float Beta0_6 = 3.1415/2.2;



float std_lenght1;
//Max möjliga steglängd från grundpositionen
float max_step_lenght = 40;

float Leg1_prev_angleGamma;
float Leg1_prev_angleBeta;
float Leg1_prev_angleAlpha;

float Leg2_prev_angleGamma;
float Leg2_prev_angleBeta;
float Leg2_prev_angleAlpha;

float Leg3_prev_angleGamma;
float Leg3_prev_angleBeta;
float Leg3_prev_angleAlpha;

float Leg4_prev_angleGamma;
float Leg4_prev_angleBeta;
float Leg4_prev_angleAlpha;

float Leg5_prev_angleGamma;
float Leg5_prev_angleBeta;
float Leg5_prev_angleAlpha;

float Leg6_prev_angleGamma;
float Leg6_prev_angleBeta;
float Leg6_prev_angleAlpha;

//Benkoordinater som nuvarande steget ska gå till
float Leg1_new_angleGamma;
float Leg1_new_angleBeta;
float Leg1_new_angleAlpha;

float Leg2_new_angleGamma;
float Leg2_new_angleBeta;
float Leg2_new_angleAlpha;

float Leg3_new_angleGamma;
float Leg3_new_angleBeta;
float Leg3_new_angleAlpha;

float Leg4_new_angleGamma;
float Leg4_new_angleBeta;
float Leg4_new_angleAlpha;

float Leg5_new_angleGamma;
float Leg5_new_angleBeta;
float Leg5_new_angleAlpha;

float Leg6_new_angleGamma;
float Leg6_new_angleBeta;
float Leg6_new_angleAlpha;
float z0; 
*/
//Jonas function for robot movement

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
	/*servoGoto(4, 3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(10, 3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(15, -3.1415/2, servoSpeed);*/
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
		
	//Second state-------------------
	/*servoGoto(3, -3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(9, -3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(16, 3.1415/2, servoSpeed);*/
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


/*
//Statusvariabler
void initvar(){
//1 om benet ska lyftas, -1 om det ska vara i marken
int Leg1_lift = 1;
int Leg2_lift = -1;
int Leg3_lift = 1;
int Leg4_lift = -1;
int Leg5_lift = 1;
int Leg6_lift = -1;

std_lenght1 = sqrt((float)x0_1*x0_1 + (float)y0_1*y0_1);

//Vinklar där föregående steg slutade
Leg1_prev_angleGamma = Gamma0_1;
Leg1_prev_angleBeta = Beta0_1;
Leg1_prev_angleAlpha = Alpha0_1;

Leg2_prev_angleGamma = Gamma0_2;
Leg2_prev_angleBeta = Beta0_2;
Leg2_prev_angleAlpha = Alpha0_2;

Leg3_prev_angleGamma = Gamma0_3;
Leg3_prev_angleBeta = Beta0_3;
Leg3_prev_angleAlpha = Alpha0_3;

Leg4_prev_angleGamma = Gamma0_4;
Leg4_prev_angleBeta = Beta0_4;
Leg4_prev_angleAlpha = Alpha0_4;

Leg5_prev_angleGamma = Gamma0_5;
Leg5_prev_angleBeta = Beta0_5;
Leg5_prev_angleAlpha = Alpha0_5;

Leg6_prev_angleGamma = Gamma0_6;
Leg6_prev_angleBeta = Beta0_6;
Leg6_prev_angleAlpha = Alpha0_6;

//Benkoordinater som nuvarande steget ska gå till
Leg1_new_angleGamma = Gamma0_1;
Leg1_new_angleBeta = Beta0_1;
Leg1_new_angleAlpha = Alpha0_1;

Leg2_new_angleGamma = Gamma0_2;
Leg2_new_angleBeta = Beta0_2;
Leg2_new_angleAlpha = Alpha0_2;

Leg3_new_angleGamma = Gamma0_3;
Leg3_new_angleBeta = Beta0_3;
Leg3_new_angleAlpha = Alpha0_3;

Leg4_new_angleGamma = Gamma0_4;
Leg4_new_angleBeta = Beta0_4;
Leg4_new_angleAlpha = Alpha0_4;

Leg5_new_angleGamma = Gamma0_5;
Leg5_new_angleBeta = Beta0_5;
Leg5_new_angleAlpha = Alpha0_5;

Leg6_new_angleGamma = Gamma0_6;
Leg6_new_angleBeta = Beta0_6;
Leg6_new_angleAlpha = Alpha0_6;

//Längd från origo till stanfdardposition för ben 1,3,4,6
//float std_lenght = sqrt(x0_1*x0_1 + y0_1*y0_1);


}

//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
void moveRobotTob(int direction, int rotation, int speed)
{
	float z = -50;
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
	float x_direction = -sinf(direction * pi / 90) * speed / 100;
	float y_direction = cosf(direction * pi / 90) * speed / 100;


	//x och y riktning för rotation, skalad med rotationshastighet
	float x_rot1 = (rotation / 50 -1)*y0_1/std_lenght1;
	float y_rot1 = (rotation / 50 -1) * x0_1 / std_lenght1;

	float x_rot2 = 0;
	float y_rot2 = rotation / 50 - 1;

	float x_rot3 = (rotation / 50 -1) * (- y0_1 / std_lenght1);
	float y_rot3 = (rotation / 50 -1) * x0_1 / std_lenght1;

	float x_rot4 = (rotation / 50 -1) * (- y0_1 / std_lenght1);
	float y_rot4 = (rotation / 50 -1) * (- x0_1 / std_lenght1);

	float x_rot5 = 0;
	float y_rot5 = -rotation / 50 + 1;

	float x_rot6 = (rotation / 50 -1) * y0_1 / std_lenght1;
	float y_rot6 = (rotation / 50 -1) * (- x0_1 / std_lenght1);


	//Addera förflyttningarna och sätter Step_max till längsta stegets längd
	float Step_max = sqrt(pow(x_direction + x_rot1,2) + pow(y_direction + y_rot1,2));
	float Step_test = sqrt(pow(x_direction + x_rot2,2) + pow(y_direction + y_rot2,2));
	Step_max = fmax(Step_max, Step_test);

	Step_test = sqrt(pow(x_direction + x_rot3,2) + pow(y_direction + y_rot3,2));
	Step_max = fmax(Step_max, Step_test);

	Step_test = sqrt(pow(x_direction + x_rot4,2) + pow(y_direction + y_rot4,2));
	Step_max = fmax(Step_max, Step_test);

	Step_test = sqrt(pow(x_direction + x_rot5,2) + pow(y_direction + y_rot5,2));
	Step_max = fmax(Step_max, Step_test);

	Step_test = sqrt(pow(x_direction + x_rot6,2) + pow(y_direction + y_rot6,2));
	Step_max = fmax(Step_max, Step_test);


	//Stegskalning för att inte alltid ta max längd på steg;
	float Step_scaling = fmax(speed/100, fabs(rotation/50-1));


	//Uppdatera vinklar för nya steget


	float Leg1_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot1)*Step_scaling;
	float Leg1_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot1)*Step_scaling;
	float Leg1_new_posz = z;
	float Leg1_new_posx = basis_change_Leg1x(Leg1_new_posx_wb,Leg1_new_posy_wb,Leg1_new_posz);
	float Leg1_new_posy = basis_change_Leg1y(Leg1_new_posx_wb,Leg1_new_posy_wb,Leg1_new_posz);
	Calc_d(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	Leg1_new_angleGamma = Calc_gamma(Leg1_new_posx, Leg1_new_posy);
	Leg1_new_angleBeta = Calc_Beta(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	Leg1_new_angleAlpha = Calc_Alpha(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	servoGoto(8, Leg1_new_angleGamma, speed);
	servoGoto(10, -Leg1_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(12, -Leg1_new_angleBeta + tibiaAngleAddition, speed);


	float Leg2_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot2)*Step_scaling;
	float Leg2_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot2)*Step_scaling;
	float Leg2_new_posz = z;
	float Leg2_new_posx = basis_change_Leg2x(Leg2_new_posx_wb,Leg2_new_posy_wb,Leg2_new_posz);
	float Leg2_new_posy = basis_change_Leg2y(Leg2_new_posx_wb,Leg2_new_posy_wb,Leg2_new_posz);
	Calc_d(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	Leg2_new_angleGamma = Calc_gamma(Leg2_new_posx, Leg2_new_posy);
	Leg2_new_angleBeta = Calc_Beta(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	Leg2_new_angleAlpha = Calc_Alpha(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	servoGoto(14, Leg2_new_angleGamma, speed);
	servoGoto(16, -Leg2_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(18, -Leg2_new_angleBeta + tibiaAngleAddition, speed);
	


	float Leg3_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot3)*Step_scaling;
	float Leg3_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot3)*Step_scaling;
	float Leg3_new_posz = z;
	float Leg3_new_posx = basis_change_Leg3x(Leg3_new_posx_wb,Leg3_new_posy_wb,Leg3_new_posz);
	float Leg3_new_posy = basis_change_Leg3y(Leg3_new_posx_wb,Leg3_new_posy_wb,Leg3_new_posz);
	Calc_d(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	Leg3_new_angleGamma = Calc_gamma(Leg3_new_posx, Leg3_new_posy);
	Leg3_new_angleBeta = Calc_Beta(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	Leg3_new_angleAlpha = Calc_Alpha(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	servoGoto(2, Leg3_new_angleGamma, speed);
	servoGoto(4, -Leg3_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(6, -Leg3_new_angleBeta + tibiaAngleAddition, speed);


	float Leg4_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot4)*Step_scaling;
	float Leg4_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot4)*Step_scaling;
	float Leg4_new_posz = z;
	float Leg4_new_posx = basis_change_Leg4x(Leg4_new_posx_wb,Leg4_new_posy_wb,Leg4_new_posz);
	float Leg4_new_posy = basis_change_Leg4y(Leg4_new_posx_wb,Leg4_new_posy_wb,Leg4_new_posz);
	Calc_d(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	Leg4_new_angleGamma = Calc_gamma(Leg4_new_posx, Leg4_new_posy);
	Leg4_new_angleBeta = Calc_Beta(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	Leg4_new_angleAlpha = Calc_Alpha(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	servoGoto(1, Leg4_new_angleGamma, speed);
	servoGoto(3, Leg4_new_angleAlpha + femurAngleAddition, speed);
	servoGoto(5, Leg4_new_angleBeta - tibiaAngleAddition, speed);


	float Leg5_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot5)*Step_scaling;
	float Leg5_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot5)*Step_scaling;
	float Leg5_new_posz = z;
	float Leg5_new_posx = basis_change_Leg5x(Leg5_new_posx_wb,Leg5_new_posy_wb,Leg5_new_posz);
	float Leg5_new_posy = basis_change_Leg5y(Leg5_new_posx_wb,Leg5_new_posy_wb,Leg5_new_posz);
	Calc_d(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	Leg5_new_angleGamma = Calc_gamma(Leg5_new_posx, Leg5_new_posy);
	Leg5_new_angleBeta = Calc_Beta(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	Leg5_new_angleAlpha = Calc_Alpha(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	servoGoto(13, Leg5_new_angleGamma, speed);
	servoGoto(15, Leg5_new_angleAlpha + femurAngleAddition, speed);
	servoGoto(17, Leg5_new_angleBeta - tibiaAngleAddition, speed);


	float Leg6_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot6)*Step_scaling;
	float Leg6_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot6)*Step_scaling;
	float Leg6_new_posz = z;
	float Leg6_new_posx = basis_change_Leg6x(Leg6_new_posx_wb,Leg6_new_posy_wb,Leg6_new_posz);
	float Leg6_new_posy = basis_change_Leg6y(Leg6_new_posx_wb,Leg6_new_posy_wb,Leg6_new_posz);
	Calc_d(Leg6_new_posx, Leg6_new_posy, Leg6_new_posz);
	Leg6_new_angleGamma = Calc_gamma(Leg6_new_posx, Leg6_new_posy);
	Leg6_new_angleBeta = Calc_Beta(Leg6_new_posx, Leg6_new_posy, Leg6_new_posz);
	Leg6_new_angleAlpha = Calc_Alpha(Leg6_new_posx, Leg6_new_posy, Leg6_new_posz);
	servoGoto(7, Leg6_new_angleGamma, speed);
	servoGoto(9, Leg6_new_angleAlpha + femurAngleAddition, speed);
	servoGoto(11, Leg6_new_angleBeta - tibiaAngleAddition, speed);
}
*/

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
	float alpha = 3.1415/4;
	float beta = 3.1415/2.2;
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
	
	
	_delay_ms(5000);
	
	//moveRobotTob(0,50,100);
    while(1)
    {
		
		//USART_DecodeRxFIFO();

		moveRobot((float)1,(float)0,(float)0.3,(float)120,(int)100,(float)0,(float)0);

        //TODO:: Please write your application code 
    }
}