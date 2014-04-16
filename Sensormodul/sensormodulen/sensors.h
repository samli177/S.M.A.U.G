#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>

void sensors_init();
void sensors_start_sample();
bool sensors_sampling_done();
void sensors_display_data();
void sensors_reset_flag();

#endif