/*
 * autonomouswalk.h
 *
 * Created: 2014-04-18 14:46:32
 *  Author: Jonas
 */ 


#ifndef AUTONOMOUSWALK_H
#define AUTONOMOUSWALK_H

#include <avr/io.h>

/**
 * \brief 
 * Sets the speed to be used in autonomous walking,
 * sets the variable gSpeed.
 *
 * \param speed
 * The speed to be used in autonomous walking, as an
 * uint8_t.
 *
 * \return void
 */
void autonomouswalk_set_speed(uint8_t speed);

/**
 * \brief 
 * Returns the speed currently being used in autonomous
 * walking.
 * 
 * \return uint8_t
 * Returns the speed as an uint8_t.
 */
uint8_t autonomouswalk_get_speed();

/**
 * \brief 
 * Enables or disables status messages sent to the PC from
 * the navigation.
 *
 * \param status
 * 0 means that status messages will be disabled.
 * 1 means that status messages will be enabled.
 * 
 * \return void
 */
void autonomouswalk_set_return_status(uint8_t status);

/**
 * \brief 
 * Checks if status messages to the PC is enabled.
 * 
 * \return uint8_t
 * 0 means that status messages are disabled.
 * 1 means that status messages are enabled.
 */
uint8_t autonomouswalk_get_return_status();

/**
 * \brief 
 * Makes the robot turn 90 deg. to the left.
 * \return void
 */
void turn_left();

/**
 * \brief 
 * Makes the robot turn 90 deg. to the right. 
 * \return void
 */
void turn_right();

/**
 * \brief 
 * Makes the robot turn around 180 deg. To be used if the robot is
 * a dead end.
 * 
 * \param frontSensor
 * The distance from the front sensor measured in centimeters as an
 * uint8_t.
 * 
 * \return void
 */
void turn_around();

/**
 * \brief 
 * Makes the robot walk forward in a corridor while regulating
 * angles to the walls and distance to the walls.
 *
 * \param sensors
 * The distances from the front and back side sensors
 * in centimeters as unsigned integers. Sensor 4 is the
 * front sensor, sensor 5 is
 * the rear sensor. Needed to regulate.
 * 
 * \return void
 */
void walk_forward();

/**
 * \brief 
 * Cause the robot to make one movement according to a priority.
 *
 * \param sensors
 * All the sensor data is needed, given in a vector, measured in
 * centimeters and given as uint8_t.
 * 
 * \return void
 */
void autonomouswalk_walk();

#endif 