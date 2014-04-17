#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

/**
 * \brief 
 * Initiates ADC, mux, and ultra sonic sensor.
 * Creates tables for converting IR-data to length.
 * \return void
 */
void sensors_init();
/**
 * \brief 
 * Starts a series of ADCs looping over all IR-sensors and ends with the US-sensor.
 * To check if the conversion is done, use sensors_sampling_done().
 * \return void
 */
void sensors_start_sample();
/**
 * \brief 
 * Get flag for ADC completion.
 * \return bool
 * True if sampling is done, otherwise false.
 */
bool sensors_sampling_done();
/**
 * \brief 
 * Prints all sensor data on the display.
 * \return void
 */
void sensors_display_data();
/**
 * \brief 
 * Resets the flag for ADC completion.
 * \return void
 */
void sensors_reset_flag();

/**
 * \brief 
 * Returns an array containing the latest sensor samples.
 * \return uint8_t[8]
 * Array with 0-6th being IR, and 7th being US.
 */
uint8_t* sensors_get_data();

#endif