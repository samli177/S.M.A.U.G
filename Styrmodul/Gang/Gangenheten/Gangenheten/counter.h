/*
 * counter.h
 *
 * Created: 4/13/2014
 *  Author: perjo018
 */


#ifndef COUNTER_H_
#define COUNTER_H_

void init_counters();
void set_counter_1(uint16_t delay);
void set_counter_3(uint16_t delay);
void reset_counter_1();
void reset_counter_3();

void wait(uint16_t delaytime);

#endif