/*
 * counter.h
 *
 * Created: 4/13/2014
 *  Author: perjo018
 */


#ifndef COUNTER_H_
#define COUNTER_H_

void init_counters();
void enable_counter_0(uint8_t enable);
void set_counter_0(uint8_t delay);
void set_counter_1(uint16_t delay);
void set_counter_2(uint16_t delay);

#endif