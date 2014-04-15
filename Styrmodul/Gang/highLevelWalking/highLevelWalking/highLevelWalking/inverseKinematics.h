/*
 * inverseKinematics.h
 *
 * Created: 4/10/2014 1:34:05 PM
 *  Author: samli177
 */ 


#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#define coxa (float)56
#define femur (float)66
#define tibia (float)131
#define femurAngleAddition (float)0.2426
#define tibiaAngleAddition (float)(-3.1415/6)
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85


void LegOneGoto(float x,float y,float z, int servospeed);

void LegGoto(float x,float y, int z, int servospeed, int side, int servo1, int servo2, int servo3);
void moveLeg1too(float x, float y, float z, int servospeed);
void moveLeg2too(float x, float y, float z, int servospeed);
void moveLeg3too(float x, float y, float z, int servospeed);
void moveLeg4too(float x, float y, float z, int servospeed);
void moveLeg5too(float x, float y, float z, int servospeed);
void moveLeg6too(float x, float y, float z, int servospeed);

//New functions by Tobias
void Calc_d(float x,float y,float z);
float Calc_gamma(float x,float y);
float Calc_Beta();
float Calc_Alpha(float z);

float basis_change_Leg1x(float x, float y);
float basis_change_Leg1y(float x, float y);
float basis_change_Leg2x(float x);
float basis_change_Leg2y(float y);
float basis_change_Leg3x(float x, float y);
float basis_change_Leg3y(float x, float y); 
float basis_change_Leg4x(float x, float y); 
float basis_change_Leg4y(float x, float y); 
float basis_change_Leg5x(float x); 
float basis_change_Leg5y(float y); 
float basis_change_Leg6x(float x, float y); 
float basis_change_Leg6y(float x, float y); 
#endif /* INVERSEKINEMATICS_H_ */