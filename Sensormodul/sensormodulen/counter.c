/*
 * counter.c
 *
 * Created: 4/13/2014
 *  Author: perjo018
 */


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"
#include "twi.h"
#include "counter.h"

// OBS: fick byta timer 2 mot timer 3 s� kommentarerna nedan st�mmer inte riktigt...
//---------------------------------------COUNTERS/TIMERS-----------
void init_counters()
{
	// WGMn3:0 = 4 (OCRnA) or 12 (OCRn), where top value is read from.
	TCCR0B |= 0b00000101; // clock 1, prescaler 1024
	TCCR1B |= 0b00000101; // clock 1, prescaler 1024
	TCCR3B |= 0b00000101; // clock 2, prescaler 1024
	
	// standardvalue for interrupt is 1000ms
	set_counter_0(100);
	set_counter_1(1000);
	set_counter_2(1000);
	
	enable_counter_0(0);
	TIMSK1 |= 0b00000010; // Enable interrupts when OCF1A, in TIFR1, is set.
	TIMSK3 |= 0b00000010; // Enable interrupts when OCF2A, in TIFR2, is set.
	// OCF1A (or ICFn) Flag, in TIFR1, can be used to generate interrupts.
	TCNT0 = 0;
	TCNT1 = 0;
	TCNT3 = 0;
}

void enable_counter_0(uint8_t enable)
{
	if(enable)
	{
		TIMSK0 |= 0b00000010;
	} else {
		TIMSK0 &= 0b11111101;
	}
}

void set_counter_0(uint8_t delay)
{
	delay = 15.625 * delay;
	OCR0A = delay;
}

void set_counter_1(uint16_t delay)
{
	delay = 15.625 * delay;
	OCR1A = delay;
}

void set_counter_2(uint16_t delay)
{
	delay = 15.625 * delay;
	OCR3A = delay;
}



//---------------------------------------------------------------------------------------
