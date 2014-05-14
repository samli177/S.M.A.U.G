 /*
 * inverseKinematics.h
 *
 * Created: 4/10/2014 1:34:05 PM
 *  Author: samli177
 */ 


#ifndef INVERSEKINEMATICS_H_
#define INVERSEKINEMATICS_H_

#define coxa (float)54
#define femur (float)67
#define tibia (float)132
#define femurAngleAddition (float)(0.231-0.1) //0.2426
#define tibiaAngleAddition (float)(0.812-0.25) //(-3.1415/6)
#define centerToFrontLegsY (float)120
#define centerToSideLegs (float)100
#define centerToFrontLegs (float)135
#define centerToFrontLegsX (float)61.85

#define pi (float) 3.14159265
#define sqrt2 (float) 1.41421356



#define Gamma0 (float) 0
#define Beta0 (float) 1.875
#define Alpha0 (float) 0.1406

/**
 * \brief 
 * Calculates length of d for a leg. d is the length 
 * between servo two on the leg and the foot.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \param float z
 * New z position in leg basis.
 *
 * \return void
 */
void calc_d(float x,float y,float z);

/**
 * \brief 
 * Calculates angle gamma.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of gamma.
 */
float get_gamma(float x,float y);

/**
 * \brief 
 * Calculates angle beta.
 *
 * \return float
 * Returns calculated value of beta.
 */
float get_beta();

/**
 * \brief 
 * Calculates angle alpha.
 *
 * \param float z
 * New z position in leg basis.
 *
 * \return float
 * Returns calculated value of alpha.
 */
float get_alpha(float z);

/**
 * \brief 
 * Changes basis from robot basis to leg 1 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg1x(float x, float y);

/**
 * \brief 
 * Changes basis from robot basis to leg 1 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg1y(float x, float y);

/**
 * \brief 
 * Changes basis from robot basis to leg 2 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg2x(float x);

/**
 * \brief 
 * Changes basis from robot basis to leg 2 basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg2y(float y);

/**
 * \brief 
 * Changes basis from robot basis to leg 3 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg3x(float x, float y);

/**
 * \brief 
 * Changes basis from robot basis to leg 3 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg3y(float x, float y); 

/**
 * \brief 
 * Changes basis from robot basis to leg 4 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg4x(float x, float y); 

/**
 * \brief 
 * Changes basis from robot basis to leg 4 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg4y(float x, float y); 

/**
 * \brief 
 * Changes basis from robot basis to leg 5 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg5x(float x); 

/**
 * \brief 
 * Changes basis from robot basis to leg 5 basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg5y(float y); 

/**
 * \brief 
 * Changes basis from robot basis to leg 6 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of x.
 */
float basis_change_leg6x(float x, float y); 

/**
 * \brief 
 * Changes basis from robot basis to leg 6 basis.
 *
 * \param float x
 * New x position in leg basis.
 *
 * \param float y
 * New y position in leg basis.
 *
 * \return float
 * Returns calculated value of y.
 */
float basis_change_leg6y(float x, float y); 

void height_change_leg1(float new_z);
void height_change_leg2(float new_z);
void height_change_leg3(float new_z);
void height_change_leg4(float new_z);
void height_change_leg5(float new_z);
void height_change_leg6(float new_z);
void height_change_all(float new_z);

float get_x0_1();
float get_y0_1();

float get_x0_2();
float get_y0_2();

float get_x0_3();
float get_y0_3();

float get_x0_4();
float get_y0_4();

float get_x0_5();
float get_y0_5();

float get_x0_6();
float get_y0_6();

float get_x0();
float get_y0();
float get_z0();
#endif /* INVERSEKINEMATICS_H_ */