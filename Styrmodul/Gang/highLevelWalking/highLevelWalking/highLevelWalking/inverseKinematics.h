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
#define femurAngleAddition (float)0.231 //0.2426
#define tibiaAngleAddition (float)0.812 //(-3.1415/6)
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85

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

#define side1 (float)1
#define side2 (float)1
#define side3 (float)1
#define side4 (float)(-1)
#define side5 (float)(-1)
#define side6 (float)(-1)

#define Gamma0 (float) 0
#define Beta0 (float) 1.875
#define Alpha0 (float) 0.1406


void LegOneGoto(float x,float y,float z, int servospeed);

void LegGoto(float x,float y, int z, int servospeed, int side, int servo1, int servo2, int servo3);
void moveLeg1too(float x, float y, float z, int servospeed);
void moveLeg2too(float x, float y, float z, int servospeed);
void moveLeg3too(float x, float y, float z, int servospeed);
void moveLeg4too(float x, float y, float z, int servospeed);
void moveLeg5too(float x, float y, float z, int servospeed);
void moveLeg6too(float x, float y, float z, int servospeed);

//New functions by Tobias
void calc_d(float x,float y,float z);
float get_gamma(float x,float y);
float get_beta();
float get_alpha(float z);

float basis_change_leg1x(float x, float y);
float basis_change_leg1y(float x, float y);
float basis_change_leg2x(float x);
float basis_change_leg2y(float y);
float basis_change_leg3x(float x, float y);
float basis_change_leg3y(float x, float y); 
float basis_change_leg4x(float x, float y); 
float basis_change_leg4y(float x, float y); 
float basis_change_leg5x(float x); 
float basis_change_leg5y(float y); 
float basis_change_leg6x(float x, float y); 
float basis_change_leg6y(float x, float y); 
#endif /* INVERSEKINEMATICS_H_ */