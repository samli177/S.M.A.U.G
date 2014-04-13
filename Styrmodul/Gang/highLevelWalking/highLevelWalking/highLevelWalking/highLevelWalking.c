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

#define x0_1 -215 //standard x pos for leg 1
#define y0_1  275 //standard y pos for leg 1
#define x0_2  -340 //standard x pos for leg 2
#define y0_2  0 //standard y pos for leg 2
#define x0_3  -215 //standard x pos for leg 3
#define y0_3  -275 //standard y pos for leg 3
#define x0_4 215 //standard x pos for leg 4
#define y0_4  -275 //standard y pos for leg 4
#define x0_5  340 //standard x pos for leg 5
#define y0_5  0 //standard y pos for leg 5
#define x0_6  215 //standard x pos for leg 6
#define y0_6  275 //standard y pos for leg 6


#define frontLegDistance 200
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85
/*
double x0_1 = -240; //standard x pos for leg 1
double y0_1 = 250; //standard y pos for leg 1
double x0_2 = -320; //standard x pos for leg 2
double y0_2 = 0; //standard y pos for leg 2
double x0_3 = -240; //standard x pos for leg 3
double y0_3 = -250; //standard y pos for leg 3
double x0_4 = 240; //standard x pos for leg 4
double y0_4 = -250; //standard y pos for leg 4
double x0_5 = 320; //standard x pos for leg 5
double y0_5 = 0; //standard y pos for leg 5
double x0_6 = 240; //standard x pos for leg 6
double y0_6 = 250; //standard y pos for leg 6
*/
/*
double Gamma0_1 = 0;
double Alpha0_1 = -3.1415/4;
double Beta0_1 = -3.1415/2.2;

double Gamma0_2 = 0;
double Alpha0_2 = -3.1415/4;
double Beta0_2 = -3.1415/2.2;

double Gamma0_3 = 0;
double Alpha0_3 = -3.1415/4;
double Beta0_3 = -3.1415/2.2;

double Gamma0_4 = 0;
double Alpha0_4 = -3.1415/4;
double Beta0_4 = -3.1415/2.2;

double Gamma0_5 = 0;
double Alpha0_5 = 3.1415/4;
double Beta0_5 = 3.1415/2.2;

double Gamma0_6 = 0;
double Alpha0_6 = 3.1415/4;
double Beta0_6 = 3.1415/2.2;



double std_lenght1;
//Max möjliga steglängd från grundpositionen
double max_step_lenght = 40;

double Leg1_prev_angleGamma;
double Leg1_prev_angleBeta;
double Leg1_prev_angleAlpha;

double Leg2_prev_angleGamma;
double Leg2_prev_angleBeta;
double Leg2_prev_angleAlpha;

double Leg3_prev_angleGamma;
double Leg3_prev_angleBeta;
double Leg3_prev_angleAlpha;

double Leg4_prev_angleGamma;
double Leg4_prev_angleBeta;
double Leg4_prev_angleAlpha;

double Leg5_prev_angleGamma;
double Leg5_prev_angleBeta;
double Leg5_prev_angleAlpha;

double Leg6_prev_angleGamma;
double Leg6_prev_angleBeta;
double Leg6_prev_angleAlpha;

//Benkoordinater som nuvarande steget ska gå till
double Leg1_new_angleGamma;
double Leg1_new_angleBeta;
double Leg1_new_angleAlpha;

double Leg2_new_angleGamma;
double Leg2_new_angleBeta;
double Leg2_new_angleAlpha;

double Leg3_new_angleGamma;
double Leg3_new_angleBeta;
double Leg3_new_angleAlpha;

double Leg4_new_angleGamma;
double Leg4_new_angleBeta;
double Leg4_new_angleAlpha;

double Leg5_new_angleGamma;
double Leg5_new_angleBeta;
double Leg5_new_angleAlpha;

double Leg6_new_angleGamma;
double Leg6_new_angleBeta;
double Leg6_new_angleAlpha;
double z0; 
*/
//Jonas function for robot movement

double z0 = -50;
int speed = 80;

void moveRobot(double direction,double distance, double rotation, double z, int servoSpeed, double rotationX, double rotationY)
{
	double sinrotation = sin(rotation);
	double cosrotation = cos(rotation);
	double sindirection = sin(direction);
	double cosdirection = cos(direction);
	double sinaroundx = sin(rotationX);
	double sinaroundy = cos(rotationY);
	
	//First state-------------------
	servoGoto(4, 3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(10, 3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(15, -3.1415/2, servoSpeed);
	_delay_ms(1000);
	moveLeg1too(x0_1*cosrotation-y0_1*sinrotation-sindirection*distance, y0_1*cosrotation-x0_1*sinrotation+cosdirection*distance, -(z+sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg3too(x0_3*cosrotation-y0_3*sinrotation-sindirection*distance, y0_3*cosrotation-x0_3*sinrotation+cosdirection*distance, -(z+sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg5too(x0_5*cosrotation-y0_5*sinrotation-sindirection*distance, y0_5*cosrotation+x0_5*sinrotation+cosdirection*distance, -(z-centerToSideLegs*sinaroundy), servoSpeed);
	_delay_ms(3000);
		
	//Second state-------------------
	servoGoto(3, -3.1415/2, servoSpeed); //raise legs
	_delay_ms(5);
	servoGoto(9, -3.1415/2, servoSpeed);
	_delay_ms(5);
	servoGoto(16, 3.1415/2, servoSpeed);
	_delay_ms(1000);
	moveLeg2too(x0_2*cosrotation-y0_2*sinrotation-sindirection*distance, y0_2*cosrotation+x0_2*sinrotation+cosdirection*distance, -(z+centerToSideLegs*sinaroundy), servoSpeed);
	moveLeg4too(x0_4*cosrotation-y0_4*sinrotation-sindirection*distance, y0_4*cosrotation-x0_4*sinrotation+cosdirection*distance, -(z-sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg6too(x0_6*cosrotation-y0_6*sinrotation-sindirection*distance, y0_6*cosrotation-x0_6*sinrotation+cosdirection*distance, -(z-sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY), servoSpeed);
	_delay_ms(3000);

	//Third state-------------------
	moveLeg1too(x0_1, y0_1, -(z+sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg2too(x0_2, y0_2, -(z+centerToSideLegs*sinaroundy), servoSpeed);
	moveLeg3too(x0_3, y0_3, -(z+sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg4too(x0_4, y0_4, -(z-sinaroundy*centerToFrontLegsX-sinaroundx*centerToFrontLegsY), servoSpeed);
	moveLeg5too(x0_5, y0_5, -(z-centerToSideLegs*sinaroundy), servoSpeed);
	moveLeg6too(x0_6, y0_6, -(z-sinaroundy*centerToFrontLegsX+sinaroundx*centerToFrontLegsY), servoSpeed);
	
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

std_lenght1 = sqrt((double)x0_1*x0_1 + (double)y0_1*y0_1);

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

//Längd från origo till standardposition för ben 1,3,4,6
//double std_lenght = sqrt(x0_1*x0_1 + y0_1*y0_1);


}

//Tar in styrkommandon (format á la Martin) och uppdaterar variabler för positionen
//av ben för förra steget och räknar ut position för nästa steg.
void moveRobotTob(int direction, int rotation, int speed)
{
	double z = -50;
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
	double x_direction = -sin(direction * pi / 90) * speed / 100;
	double y_direction = cos(direction * pi / 90) * speed / 100;


	//x och y riktning för rotation, skalad med rotationshastighet
	double x_rot1 = (rotation / 50 -1)*y0_1/std_lenght1;
	double y_rot1 = (rotation / 50 -1) * x0_1 / std_lenght1;

	double x_rot2 = 0;
	double y_rot2 = rotation / 50 - 1;

	double x_rot3 = (rotation / 50 -1) * (- y0_1 / std_lenght1);
	double y_rot3 = (rotation / 50 -1) * x0_1 / std_lenght1;

	double x_rot4 = (rotation / 50 -1) * (- y0_1 / std_lenght1);
	double y_rot4 = (rotation / 50 -1) * (- x0_1 / std_lenght1);

	double x_rot5 = 0;
	double y_rot5 = -rotation / 50 + 1;

	double x_rot6 = (rotation / 50 -1) * y0_1 / std_lenght1;
	double y_rot6 = (rotation / 50 -1) * (- x0_1 / std_lenght1);


	//Addera förflyttningarna och sätter Step_max till längsta stegets längd
	double Step_max = sqrt(pow(x_direction + x_rot1,2) + pow(y_direction + y_rot1,2));
	double Step_test = sqrt(pow(x_direction + x_rot2,2) + pow(y_direction + y_rot2,2));
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
	double Step_scaling = fmax(speed/100, fabs(rotation/50-1));


	//Uppdatera vinklar för nya steget


	double Leg1_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot1)*Step_scaling;
	double Leg1_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot1)*Step_scaling;
	double Leg1_new_posz = z;
	double Leg1_new_posx = basis_change_Leg1x(Leg1_new_posx_wb,Leg1_new_posy_wb,Leg1_new_posz);
	double Leg1_new_posy = basis_change_Leg1y(Leg1_new_posx_wb,Leg1_new_posy_wb,Leg1_new_posz);
	Calc_d(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	Leg1_new_angleGamma = Calc_gamma(Leg1_new_posx, Leg1_new_posy);
	Leg1_new_angleBeta = Calc_Beta(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	Leg1_new_angleAlpha = Calc_Alpha(Leg1_new_posx, Leg1_new_posy, Leg1_new_posz);
	servoGoto(8, Leg1_new_angleGamma, speed);
	servoGoto(10, -Leg1_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(12, -Leg1_new_angleBeta + tibiaAngleAddition, speed);


	double Leg2_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot2)*Step_scaling;
	double Leg2_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot2)*Step_scaling;
	double Leg2_new_posz = z;
	double Leg2_new_posx = basis_change_Leg2x(Leg2_new_posx_wb,Leg2_new_posy_wb,Leg2_new_posz);
	double Leg2_new_posy = basis_change_Leg2y(Leg2_new_posx_wb,Leg2_new_posy_wb,Leg2_new_posz);
	Calc_d(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	Leg2_new_angleGamma = Calc_gamma(Leg2_new_posx, Leg2_new_posy);
	Leg2_new_angleBeta = Calc_Beta(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	Leg2_new_angleAlpha = Calc_Alpha(Leg2_new_posx, Leg2_new_posy, Leg2_new_posz);
	servoGoto(14, Leg2_new_angleGamma, speed);
	servoGoto(16, -Leg2_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(18, -Leg2_new_angleBeta + tibiaAngleAddition, speed);
	


	double Leg3_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot3)*Step_scaling;
	double Leg3_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot3)*Step_scaling;
	double Leg3_new_posz = z;
	double Leg3_new_posx = basis_change_Leg3x(Leg3_new_posx_wb,Leg3_new_posy_wb,Leg3_new_posz);
	double Leg3_new_posy = basis_change_Leg3y(Leg3_new_posx_wb,Leg3_new_posy_wb,Leg3_new_posz);
	Calc_d(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	Leg3_new_angleGamma = Calc_gamma(Leg3_new_posx, Leg3_new_posy);
	Leg3_new_angleBeta = Calc_Beta(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	Leg3_new_angleAlpha = Calc_Alpha(Leg3_new_posx, Leg3_new_posy, Leg3_new_posz);
	servoGoto(2, Leg3_new_angleGamma, speed);
	servoGoto(4, -Leg3_new_angleAlpha - femurAngleAddition, speed);
	servoGoto(6, -Leg3_new_angleBeta + tibiaAngleAddition, speed);


	double Leg4_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot4)*Step_scaling;
	double Leg4_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot4)*Step_scaling;
	double Leg4_new_posz = z;
	double Leg4_new_posx = basis_change_Leg4x(Leg4_new_posx_wb,Leg4_new_posy_wb,Leg4_new_posz);
	double Leg4_new_posy = basis_change_Leg4y(Leg4_new_posx_wb,Leg4_new_posy_wb,Leg4_new_posz);
	Calc_d(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	Leg4_new_angleGamma = Calc_gamma(Leg4_new_posx, Leg4_new_posy);
	Leg4_new_angleBeta = Calc_Beta(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	Leg4_new_angleAlpha = Calc_Alpha(Leg4_new_posx, Leg4_new_posy, Leg4_new_posz);
	servoGoto(1, Leg4_new_angleGamma, speed);
	servoGoto(3, Leg4_new_angleAlpha + femurAngleAddition, speed);
	servoGoto(5, Leg4_new_angleBeta - tibiaAngleAddition, speed);


	double Leg5_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot5)*Step_scaling;
	double Leg5_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot5)*Step_scaling;
	double Leg5_new_posz = z;
	double Leg5_new_posx = basis_change_Leg5x(Leg5_new_posx_wb,Leg5_new_posy_wb,Leg5_new_posz);
	double Leg5_new_posy = basis_change_Leg5y(Leg5_new_posx_wb,Leg5_new_posy_wb,Leg5_new_posz);
	Calc_d(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	Leg5_new_angleGamma = Calc_gamma(Leg5_new_posx, Leg5_new_posy);
	Leg5_new_angleBeta = Calc_Beta(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	Leg5_new_angleAlpha = Calc_Alpha(Leg5_new_posx, Leg5_new_posy, Leg5_new_posz);
	servoGoto(13, Leg5_new_angleGamma, speed);
	servoGoto(15, Leg5_new_angleAlpha + femurAngleAddition, speed);
	servoGoto(17, Leg5_new_angleBeta - tibiaAngleAddition, speed);


	double Leg6_new_posx_wb = max_step_lenght/Step_max*(x_direction + x_rot6)*Step_scaling;
	double Leg6_new_posy_wb = max_step_lenght/Step_max*(y_direction + y_rot6)*Step_scaling;
	double Leg6_new_posz = z;
	double Leg6_new_posx = basis_change_Leg6x(Leg6_new_posx_wb,Leg6_new_posy_wb,Leg6_new_posz);
	double Leg6_new_posy = basis_change_Leg6y(Leg6_new_posx_wb,Leg6_new_posy_wb,Leg6_new_posz);
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
	double alpha = 3.1415/4;
	double beta = 3.1415/2.2;
	int speed = 180;
	
	//dummy code that puts robot in standard position
	
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
		//_delay_ms(1000);
		moveRobot((double)0,(double)25,(double)0,(double)70,(int)100,(double)0,(double)0);
        //TODO:: Please write your application code 
    }
}