/*
* Sensors.c
* 
* Created: 16/4/2014 around 13:30 PM
* Author: Martin
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include "sensors.h"
#include "display.h"
#include "twi.h"

static uint8_t gSelectedSensor = 0;
static uint8_t gSensorBuffer[8];
static float IRShort[13][2];
static float IRLong[15][2];
static bool sensorDataFlag = false;

static void select_sensor(int sensor);
static void start_ul_sensor();
static int voltage_to_mm_short(float voltage);
static int voltage_to_mm_long(float voltage);
static void adc_init();
static void adc_start();
static void init_mux();
static void init_tables();
static void init_UL();

void sensors_init()
{
	adc_init();
	init_tables();
	init_mux();
	init_UL();
}

void init_tables()
{
	// 10-80 cm
	IRShort[0][0] = 3.15;
	IRShort[0][1] = 6;
	
	IRShort[1][0] = 2.98;
	IRShort[1][1] = 7;

	IRShort[2][0] = 2.75;
	IRShort[2][1] = 8;
	
	IRShort[3][0] = 2.31;
	IRShort[3][1] = 10;
	
	IRShort[4][0] = 1.64;
	IRShort[4][1] = 15;
	
	IRShort[5][0] = 1.31;
	IRShort[5][1] = 20;
	
	IRShort[6][0] = 1.08;
	IRShort[6][1] = 25;
	
	IRShort[7][0] = 0.92;
	IRShort[7][1] = 30;
	
	IRShort[8][0] = 0.74;
	IRShort[8][1] = 40;
	
	IRShort[9][0] = 0.61;
	IRShort[9][1] = 50;
	
	IRShort[10][0] = 0.51;
	IRShort[10][1] = 60;
	
	IRShort[11][0] = 0.45;
	IRShort[11][1] = 70;
	
	IRShort[12][0] = 0.41;
	IRShort[12][1] = 80;
	
	// 20-150 cm
	IRLong[0][0] = 2.75;
	IRLong[0][1] = 15;
	
	IRLong[1][0] = 2.55;
	IRLong[1][1] = 20;
	
	IRLong[2][0] = 2.00;
	IRLong[2][1] = 30;
	
	IRLong[3][0] = 1.55;
	IRLong[3][1] = 40;
	
	IRLong[4][0] = 1.25;
	IRLong[4][1] = 50;
	
	IRLong[5][0] = 1.07;
	IRLong[5][1] = 60;
	
	IRLong[6][0] = 0.85;
	IRLong[6][1] = 70;
	
	IRLong[7][0] = 0.80;
	IRLong[7][1] = 80;
	
	IRLong[8][0] = 0.75;
	IRLong[8][1] = 90;
	
	IRLong[9][0] = 0.65;
	IRLong[9][1] = 100;
	
	IRLong[10][0] = 0.60;
	IRLong[10][1] = 110;
	
	IRLong[11][0] = 0.55;
	IRLong[11][1] = 120;
	
	IRLong[12][0] = 0.50;
	IRLong[12][1] = 130;
	
	IRLong[13][0] = 0.45;
	IRLong[13][1] = 140;
	
	IRLong[14][0] = 0.42;
	IRLong[14][1] = 150;
}

int voltage_to_mm_short(float voltage)
{
	if(voltage >= IRShort[0][0])
	{
		return IRShort[0][1]*10;
	} else if(voltage <= IRShort[12][0])
	{
		return IRShort[12][1]*10;
	}
	
	for(int i = 0; i < 13; ++i)
	{
		float prev = IRShort[i][0];
		float next = IRShort[i+1][0];
		if(next == voltage)
		{
			return IRShort[i+1][1]*10;
		} else if(prev > voltage && next < voltage)
		{
			int high = IRShort[i][1]*10;
			int low = IRShort[i+1][1]*10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (int) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return IRShort[12][1]*10;
}

int voltage_to_mm_long(float voltage)
{
	if(voltage >= IRLong[0][0])
	{
		return IRLong[0][1]*10;
	} else if(voltage <= IRLong[14][0])
	{
		return IRLong[14][1]*10;
	}
	
	for(int i = 0; i < 13; ++i)
	{
		float prev = IRLong[i][0];
		float next = IRLong[i+1][0];
		if(next == voltage)
		{
			return IRLong[i+1][1]*10;
		} else if(prev > voltage && next < voltage)
		{
			int high = IRLong[i][1]*10;
			int low = IRLong[i+1][1]*10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (int) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return IRLong[14][1]*10;
}void init_mux()
{
	DDRA |= 0b00111110;
	DDRA &= ~(1<<PORTA0);
	PORTA &= ~(1<<PORTA5);
	PORTA &= ~((1<<PORTA1) | (1<<PORTA2) | (1<<PORTA3) | (1<<PORTA4));
	//PORTA &= 0b11100001;
}

void init_UL()
{
	DDRD |= 1;
	PCICR = 1;
	PCMSK0 = (1<<PCINT6);
	DDRA |= (1<<PORTA7);
	DDRA &= ~(1<<PORTA6);
	TCCR0B = 0x05;
}

void adc_init()
{
	// ADC enabled, enable interrupt, set division factor for clock to be 128
	ADCSRA = (1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);
	
	// Left adjust, set voltage reference selection
	ADMUX = 1<<ADLAR | 1<<REFS0;
}

void adc_start()
{
	ADCSRA |= 1<<ADSC;
}

void sensors_start_sample()
{
	select_sensor(0);
	adc_start();
}

void sensors_display_data()
{
	display_clear();
	
	display_set_pos(0,0);
	display_text("0: ");
	display_value(gSensorBuffer[0]);
	
	display_set_pos(0,8);
	display_text("1: ");
	display_value(gSensorBuffer[1]);
	
	display_set_pos(1,0);
	display_text("2: ");
	display_value(gSensorBuffer[2]);
	
	display_set_pos(1,8);
	display_text("3: ");
	display_value(gSensorBuffer[3]);
	
	display_set_pos(2,0);
	display_text("4: ");
	display_value(gSensorBuffer[4]);
	
	display_set_pos(2,8);
	display_text("5: ");
	display_value(gSensorBuffer[5]);
	
	display_set_pos(3,0);
	display_text("6: ");
	display_value(gSensorBuffer[6]);
	
	display_set_pos(3,8);
	display_text("7: ");
	display_value(gSensorBuffer[7]);
}

void select_sensor(int sensor)
{
	gSelectedSensor = sensor;
	PORTA &= ~((1<<PORTA1) | (1<<PORTA2) | (1<<PORTA3) | (1<<PORTA4));
	switch(sensor)
	{
		case(0):
		PORTA |= 1<<PORTA1;
		break;
		case(1):
		PORTA |= 1<<PORTA2;
		break;
		case(2):
		PORTA |= 1<<PORTA1 | 1<<PORTA2;
		break;
		case(3):
		PORTA |= 1<<PORTA3;
		break;
		case(4):
		PORTA |= 1<<PORTA1 | 1<<PORTA3;
		break;
		case(5):
		PORTA |= 1<<PORTA2 | 1<<PORTA3;
		break;
		case(6):
		PORTA |= 1<<PORTA1 | 1<<PORTA2 | 1<<PORTA3;
		break;
		default:
		//Do nada
		break;
	}
}

void start_ul_sensor()
{
	TCNT0 = 0;
	PORTA |= (1<<PORTA7);
	_delay_us(15);
	PORTA &= ~(1<<PORTA7);
}

bool sensors_sampling_done()
{
	return sensorDataFlag;
}

void sensors_reset_flag()
{
	sensorDataFlag = false;
}

uint8_t* sensors_get_data()
{
	return gSensorBuffer;
}


ISR(ADC_vect)
{
	cli();	uint8_t adcValue = ADCH;	float vin = adcValue * 5.0 / 256.0;	if(gSelectedSensor == 4)	{		gSensorBuffer[gSelectedSensor] = voltage_to_mm_long(vin)/10;		} else {		gSensorBuffer[gSelectedSensor] = voltage_to_mm_short(vin)/10;	}		if(gSelectedSensor < 6)	{		// Not last sensor		select_sensor(gSelectedSensor + 1);		adc_start();	} else {
		select_sensor(0);		start_ul_sensor();	}	sei();}

ISR(PCINT0_vect)
{
	cli();
	if(PINA & (1<<PINA6))
	{
		TCNT0 = 0;
	}
	else
	{
		uint8_t UL = TCNT0;
		gSensorBuffer[7] = UL;
		sensorDataFlag = true;
	}
	TWI_send_sensors(sensors_get_data(), 0);
	sei();
}

