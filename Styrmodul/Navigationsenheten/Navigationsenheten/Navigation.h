#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <avr/io.h>

#define DISTANCE_FRONT_TO_BACK 14.5   // cm
#define DISTANCE_MIDDLE_TO_SIDE 7.5   // cm
#define CORRIDOR_WIDTH 80             // cm
#define ACCEPTABLE_OFFSET_ANGLE 0.05   // radians
#define ACCEPTABLE_DISTANCE_OFFSET 2  // cm
#define HEIGHT_LIMIT 100              // cm
#define PI 3.14159265

/**
 * \brief 
 * Returns the Kp regulation variable gKp as an unsigned
 * integer
 * 
 * \return uint8_t
 * An unsigned integer.
 */
uint8_t navigation_get_Kp();

/**
 * \brief 
 * A function to set the regulation parameter gKp
 *
 * \param Kp
 * An unsigned integer, preferably not to large.
 * 
 * \return void
 */
void navigation_set_Kp(uint8_t Kp);

/**
 * \brief 
 * Checks if the robot is using a left side algorithm.
 * Reads the variable gAlgorithm.
 * 
 * \return uint8_t
 * 1 if using a left side algorithm, 0 if using a right
 * side algorithm.
 */
uint8_t navigation_left_algorithm();

/**
 * \brief 
 * A function to set the algorithm used. Sets the variable 
 * gAlgorithm.
 * 
 * \param alg
 * A 1 means use a left hand algorithm and a 0 means
 * use a right hand algorithm.
 * 
 * \return void
 */
void navigation_set_algorithm(uint8_t alg);

/**
 * \brief 
 * Checks if the robot is in autonomous mode.
 * 
 * \return uint8_t
 * A 1 if the robot is in autonomous mode, a 0 otherwise.
 * Returns the gAutonomousWalk variable.
 */
uint8_t navigation_autonomous_walk();

/**
 * \brief 
 * Enables/disables autonomous mode for the robot. Sets
 * the gAutonomousWalk variable.
 * 
 * \param walk
 * A 1 means autonomous mode will be enabled, a 0 means that
 * it will be disabled.
 * 
 * \return void
 */
void navigation_set_autonomous_walk(uint8_t walk);

/**
 * \brief 
 * Returns the offset angle of the robot during normal
 * normal walking in a corridor.
 *
 * \param sensors[4]
 * The distances from the front and back side sensors
 * in centimeters as unsigned integers. Sensor 4 is the
 * front sensor.
 *
 * \return float
 * The returned angle measured in radians between
 * -pi/2 and pi/2.
 */
float navigation_angle_offset();

/**
 * \brief 
 * Returns the angle that the robot needs to walk in
 * to get to the middle of the corridor
 * 
 * \param sensors[4]
 * The distances from the front and back side sensors
 * in centimeters as unsigned integers. Sensor 4 is the
 * front sensor.
 *
 * \param angleOffset
 * The angle generated by navigation_angle_offset measured in
 * radians as a float.
 * 
 * \return float
 * The returned angle measured in radians between 
 * -pi/2 and pi/2.
 */
float navigation_direction_regulation(float angleOffset);

/**
 * \brief 
 * Checks if the robot can take a left turn
 *
 * \param frontLeftSensor
 * The distance from the front left sensor in centimeters as an unsigned integer.
 *
 * \param backLeftSensor
 * The distance from the back left sensor in centimeters as an unsigned integer.
 * 
 * \return uint8_t
  * Return 2 if the robot can take a left turn, 1 if there is a left
  * turn coming up and 0 otherwise.
 */
uint8_t navigation_check_left_turn();

/**
 * \brief 
 * Checks if the robot can take a right turn
 *
 * \param frontRightSensor
 * The distance from the front right sensor in centimeters as an unsigned integer.
 *
 * \param backRightSensor
 * The distance from the back right sensor in centimeters as an unsigned integer.
 * 
 * \return uint8_t
 * Return 2 if the robot can take a right turn, 1 if there is a right
 * turn coming up and 0 otherwise.
 */
uint8_t navigation_check_right_turn();

/**
 * \brief 
 * Detects if there is an obstacle above
 * 
 * \param ultraSoundSensor
 * The reading from the ultra sound sensor measured in centimeters as
 * an unsigned integer.
 * 
 * \return uint8_t
 * Return 1 if an obstacle is found, 0 otherwise.
 */
uint8_t navigation_detect_low_pass_obsticle();

/**
 * \brief 
 * A function to determine if the robot is in a dead end.
 * 
 * \param sensor0
 * The distance from the front left sensor in centimeters as an unsigned integer.
 *
 * \param sensor1
 * The distance from the front right sensor in centimeters as an unsigned integer.
 *
 * \param sensor4
 * The distance from the front middle sensor in centimeters as an unsigned integer.
 * 
 * \param angleOffset 
 * The angle generated by navigation_angle_offset measured in
 * radians as a float.
 *
 * \return uint8_t
 * Returns a 1 if the robot is in a dead end.
 */
uint8_t navigation_dead_end(float angleOffset);

/**
 * \brief 
 * A function that adds the latest sensor data to the
 * sensor buffer.
 * 
 * \return void
 */
void navigation_fill_buffer();

/**
 * \brief 
 * A function that retrieves the desired sensor. The data returned 
 * will be a median of the last x sensor samples, where x is the 
 * variable "sensorBufferSize".
 *
 * \param sensorNr
 * The sensor that you require data from, a number between 0 and 7.
 * 
 * \return uint8_t
 * The value from the sensor measured in centimeters given as an
 * uint8_t.
 */
uint8_t navigation_get_sensor(int sensorNr);

#endif