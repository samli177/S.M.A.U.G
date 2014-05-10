/*
 * MpuInit.h
 *
 * Created: 5/6/2014 8:52:52 AM
 *  Author: Samuel
 */ 


#ifndef MPUINIT_H_
#define MPUINIT_H_


float MPU_get_y();
float MPU_get_p();
float MPU_get_r();

void MPU_init(void);
void MPU_update();


#endif /* MPUINIT_H_ */