/*
 * counter.c
 *
 * Created: 4/13/2014
 *  Author: perjo018
 */


#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"
#include "usart.h"
#include "counter.h"
#include "MpuInit.h"

volatile uint8_t gWaitFlag = 0;

//---------------------------------------COUNTERS/TIMERS-----------
void init_counters()
{
	// WGMn3:0 = 4 (OCRnA) or 12 (OCRn), where top value is read from.
	TCCR1B |= 0b00000101; // clock 1, prescaler 1024
	TCCR3B |= 0b00000101; // clock 2, prescaler 1024
	
	// standardvalue for interrupt is 1000ms
	set_counter_1(1000);
	set_counter_3(1000);
	
	TIMSK1 |= 0b00000010; // Enable interrupts when OCF1A, in TIFR1, is set.
	TIMSK3 |= 0b00000010; // Enable interrupts when OCF2A, in TIFR2, is set.
	// OCF1A (or ICFn) Flag, in TIFR1, can be used to generate interrupts.
	TCNT1 = 0;
	TCNT3 = 0;
}

void set_counter_1(uint16_t delay)
{
	delay = 15.625 * delay;
	OCR1A = delay;
}

void set_counter_3(uint16_t delay)
{
	delay = 15.625 * delay;
	OCR3A = delay;
}

void reset_counter_1()
{
	TCNT1 = 0;
}

void reset_counter_3()
{
	TCNT3 = 0;
}

//---------------------------------------------------------------------------------------
void wait(uint16_t delaytime)
{
	gWaitFlag = 0;
	set_counter_3(delaytime);
	while(gWaitFlag == 0)
	{
		MPU_update();
	}
}


ISR(TIMER3_COMPA_vect)
{
	gWaitFlag |= 1;
	TCNT3 = 0;
}