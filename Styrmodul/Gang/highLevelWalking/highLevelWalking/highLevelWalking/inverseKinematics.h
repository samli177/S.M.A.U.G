/*
 * inverseKinematics.h
 *
 * Created: 4/10/2014 1:34:05 PM
 *  Author: samli177
 */ 


#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#define coxa 56
#define femur 66
#define tibia 131
#define femurAngleAddition 0.2426
#define tibiaAngleAddition -3.1415/6
#define centerToFrontLegsY 120
#define centerToSideLegs 100
#define centerToFrontLegs 135
#define centerToFrontLegsX 61.85


void LegOneGoto(double x,double y,double z, int servospeed);

void LegGoto(double x,double y,double z, int servospeed, int side, int servo1, int servo2, int servo3);
void moveLeg1too(double x, double y, double z, int servospeed);
void moveLeg2too(double x, double y, double z, int servospeed);
void moveLeg3too(double x, double y, double z, int servospeed);
void moveLeg4too(double x, double y, double z, int servospeed);
void moveLeg5too(double x, double y, double z, int servospeed);
void moveLeg6too(double x, double y, double z, int servospeed);
#endif /* INVERSEKINEMATICS_H_ */