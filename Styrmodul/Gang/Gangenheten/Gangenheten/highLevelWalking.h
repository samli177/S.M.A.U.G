/*
 * highLevelWalking.h
 *
 * Created: 4/10/2014 1:35:13 PM
 *  Author: samli177
 */ 


#ifndef HIGHLEVELWALKING_H_
#define HIGHLEVELWALKING_H_

struct LegData
{
	float side;
	float lift;
	float xRot;
	float yRot;
	float newPosz;
	float newPosx;
	float newPosy;
	float prevPosx;
	float prevPosy;
	float prevPosz;
	float prevAngleGamma;
	float prevAngleBeta;
	float prevAngleAlpha;
	float newAngleGamma;
	float newAngleBeta;
	float newAngleAlpha;
	float temp1AngleGamma;
	float temp1AngleBeta;
	float temp1AngleAlpha;
	float temp2AngleGamma;
	float temp2AngleBeta;
	float temp2AngleAlpha;
	int servoGamma;
	int servoBeta;
	int servoAlpha;
	int climbing;
	uint16_t currPosAlpha;
	uint16_t currPosBeta;
	uint16_t currPosGamma;
	uint16_t currSpeedAlpha;
	uint16_t currSpeedBeta;
	uint16_t currSpeedGamma;
	uint16_t currLoadAlpha;
	uint16_t currLoadBeta;
	uint16_t currLoadGamma;
	uint16_t currVoltAlpha;
	uint16_t currVoltBeta;
	uint16_t currVoltGamma;
	uint16_t currTempAlpha;
	uint16_t currTempBeta;
	uint16_t currTempGamma;

	float goalAngleAlpha;
	float goalAngleBeta;
	float goalAngleGamma;

};

//void moveRobotTob(int direction, int rotation, int speed);

/**
 * \brief 
 * Main function for initiating movement of robot
 *
 * \param dir
 * The direction of walking as an int. 0=<direction=<90.
 * 0 is forward. +1 gives +4 degrees to walking direction.
 *
 * \param rot
 * The amount of rotation as an int. 0=<rotation=<100.
 * 0 is maximum rotation CCW, 50 is no rotation,
 * 100 is maximum rotation CW.
 *
 * \param spd
 * The length of the direction step as an int.
 * 0 gives no direction step length and 100 gives
 * maximum direction step length.
 *
 * \return void
 */
void move_robot(int dir, int rot, int spd);

/**
 * \brief 
 * Handles leg motion with already calculated parameters. 
 *
 * \return void
 */
void leg_motion();											// This one is at two places!

/**
 * \brief 
 * Initializes all variables of a leg with starting values.
 *
 * \param struct LegData*
 * Struct containing all variables of a leg.
 *
 * \return void
 */
void init_struct(struct LegData* leg);

/**
 * \brief 
 * Initializes variables with starting values.
 *
 * \return void
 */
void initvar();

/**
 * \brief 
 * Updates variables of leg to start new step.
 *
 * \param struct LegData*
 * Struct containing all variables of a leg.
 *
 * \return void
 */
void step_start(struct LegData* leg);

/**
 * \brief 
 * Calculates new position for leg in basis where origin
 * is standard position for each leg. Positive x is to the
 * right, positive y is forward and positive z is upwards.
 *
 * \param struct LegData*
 * Struct containing all variables of a leg.
 *
 * \return void
 */
void step_part1_calculator(struct LegData* leg);

/**
 * \brief 
 * Calculates angles for each servo on leg.
 *
 * \param struct LegData*
 * Struct containing all variables of a leg.
 *
 * \return void
 */
void step_part2_calculator(struct LegData* leg);


/**
 * \brief 
 * Updates variables for new step.
 *
 * \return void
 */
void leg_motion_init();

/**
 * \brief 
 * Calculates the path of a leg and updates servos.
 *
 * \param float n
 * Number of iterations for each step.
 *
 * \param struct LegData*
 * Struct containing all variables of a leg.
 *
 * \return void
 */
void move_leg(struct LegData* leg, float n);

/**
 * \brief 
 * Moves all legs one step.
 *
 * \return void
 */
void leg_motion();										// This one is at two places!

/**
 * \brief 
 * Makes the robot move to standard position
 *
 * \return void
 */
void move_to_std();

void climb();
// void climb_down();
void leg_climb(struct LegData* leg);
// void leg_climb_down(struct LegData* leg);
void change_z(float input);

void update_leg_info(struct LegData* leg);
uint16_t angle_to_servo_pos(float angle);
uint8_t close_enough(struct LegData* leg, uint8_t tolerance);

#endif /* HIGHLEVELWALKING_H_ */