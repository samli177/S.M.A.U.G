﻿/*
 * inverseKinematics.h
 *
 * Created: 4/10/2014 1:34:05 PM
 *  Author: samli177
 */ 


#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#define coxa (double)56
#define femur (double)66
#define tibia (double)131
#define femurAngleAddition (double)0.2426
#define tibiaAngleAddition (double)(-3.1415/6)
#define centerToFrontLegsY (double)120
#define centerToSideLegs (double)100
#define centerToFrontLegs (double)135
#define centerToFrontLegsX (double)61.85


void LegOneGoto(double x,double y,double z, int servospeed);

void LegGoto(double x,double y, int z, int servospeed, int side, int servo1, int servo2, int servo3);
void moveLeg1too(double x, double y, double z, int servospeed);
void moveLeg2too(double x, double y, double z, int servospeed);
void moveLeg3too(double x, double y, double z, int servospeed);
void moveLeg4too(double x, double y, double z, int servospeed);
void moveLeg5too(double x, double y, double z, int servospeed);
void moveLeg6too(double x, double y, double z, int servospeed);

//New functions by Tobias
void Calc_d(double x,double y,double z);
double Calc_gamma(double x,double y);
double Calc_Beta(double x,double y,double z);
double Calc_Alpha(double x,double y,double z);

double basis_change_Leg1x(double x, double y, double z) ;
double basis_change_Leg1y(double x, double y, double z);
double basis_change_Leg2x(double x, double y, double z);
double basis_change_Leg2y(double x, double y, double z);
double basis_change_Leg3x(double x, double y, double z);
double basis_change_Leg3y(double x, double y, double z); 
double basis_change_Leg4x(double x, double y, double z); 
double basis_change_Leg4y(double x, double y, double z); 
double basis_change_Leg5x(double x, double y, double z); 
double basis_change_Leg5y(double x, double y, double z); 
double basis_change_Leg6x(double x, double y, double z); 
double basis_change_Leg6y(double x, double y, double z); 
#endif /* INVERSEKINEMATICS_H_ */