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
	int onObstacle;
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
 * Initializes variables with starting values and start values
 * in all the structs for the legs.
 *
 * \return void
 */
void initvar();

/**
 * \brief 
 * Reads from the servos and updates the variables in the leg.
 * 
 * \param leg
 * The leg for which the servos is to be read. 
 * 
 * \return void
 */
void update_leg_info(struct LegData* leg);

/**
 * \brief 
 * Checks if the servos in the leg has reached a tolerable 
 * angle in comparison of the desired angles for the leg. 
 * 
 * \param leg
 * The leg to be checked.
 * \param tolerance
 * How accurate it must be. 
 * 
 * \return uint8_t
 */
uint8_t close_enough(struct LegData* leg, uint8_t tolerance);	// Is this correct?

/**
 * \brief 
 * Translates an angle to the right position for the servos.
 * 
 * \param angle
 * The desired angle. 
 * 
 * \return uint16_t
 */
uint16_t angle_to_servo_pos(float angle);						// Is this correct?

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
 * Updates angle variables for new step.
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
 * Handles leg motion with already calculated parameters. 
 *
 * \return void
 */
void leg_motion();										// This one is at two places! Which describes it best?

/**
 * \brief 
 * Moves the leg 10 mm down. This function is used to 
 * find the ground when climbing. 
 * 
 * \param leg
 * The leg which is to be lowered, the climbing leg. 
 * 
 * \return void
 */
void leg_move_down(struct LegData* leg);

/**
 * \brief
 * Checks if the leg has found the ground through 
 * checking if the robot is leaning. This function 
 * is used in combination with leg_move_down when 
 * climbing.
 * 
 * \param leg
 * The climbing leg which might has reached something
 * to stand on. 
 * 
 * \return void
 */
void leg_check_down(struct LegData* leg);

/**
 * \brief 
 * Handles climbing over obstacles of height 60 mm. 
 * 
 * \return void
 */
void climb();

/**
 * \brief 
 * Calculates new angles for each part of a step 
 * when climbing except for putting it down which 
 * is handled by leg_move_down and leg_check_down. 
 * 
 * \param leg
 * The leg which is to climb forward.
 *
 * \param n
 * The part of the step to be preformed. 
 * 
 * \return void
 */
void move_climb(struct LegData* leg, float n);

/**
 * \brief 
 * Changes the new z position, aka the height 
 * on which to put the feet in the next step. 
 * The function uses height_change_all which 
 * changes the standard positions for each leg. 
 * 
 * \param input
 * The desired new height. 
 * 
 * \return void
 */
void change_z(float input);

/**
 * \brief 
 * Makes the robot move to standard position
 * at the current height. 
 *
 * \return void
 */
void move_to_std();

/**
 * \brief 
 * Tells the robot to turn a specific number 
 * of degrees.
 * 
 * \param degrees
 * The number of degrees to turn.
 * \param dir
 * The turning direction. -1 => CCW, 1 => CW. 
 * 
 * \return void
 */
void turn_degrees(uint16_t degrees, int8_t dir);		// Is this correct?

/**
 * \brief 
 * Takes a mean of the last three values in 
 * r and p direction and puts the results in
 * the variables MPUPMean and MPURMean. 
 * 
 * \return void
 */
void MPU_get_mean();


#endif /* HIGHLEVELWALKING_H_ */