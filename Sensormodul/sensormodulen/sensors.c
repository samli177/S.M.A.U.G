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
#include "counter.h"

#define sensorPointsShort 39
#define sensorPointsLong 67

uint8_t gSelectedSensor = 0;
uint16_t gSensorBuffer[8];
float IRShort[sensorPointsShort][2];
float IRLong[sensorPointsLong][2];
bool sensorDataFlag = false;
uint8_t sensorDataSentFlag = 1;

static void select_sensor(int sensor);
static void start_ul_sensor();
static uint16_t voltage_to_mm_short(float voltage);
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
	
	IRShort[3][0] = 2.53;
	IRShort[3][1] = 10;
	
	IRShort[4][0] = 2.17;
	IRShort[4][1] = 12;
	
	IRShort[5][0] = 1.91;
	IRShort[5][1] = 14;
	
	IRShort[6][0] = 1.7;
	IRShort[6][1] = 16;
	
	IRShort[7][0] = 1.54;
	IRShort[7][1] = 18;
	
	IRShort[8][0] = 1.43;
	IRShort[8][1] = 20;
	
	IRShort[9][0] = 1.33;
	IRShort[9][1] = 22;
	
	IRShort[10][0] = 1.25;
	IRShort[10][1] = 24;
	
	IRShort[11][0] = 1.17;
	IRShort[11][1] = 26;
	
	IRShort[12][0] = 1.08;
	IRShort[12][1] = 28;
	
	IRShort[13][0] = 1.01;
	IRShort[13][1] = 30;
	
	IRShort[14][0] = 0.97;
	IRShort[14][1] = 32;
	
	IRShort[15][0] = 0.92;
	IRShort[15][1] = 34;
	
	IRShort[16][0] = 0.88;
	IRShort[16][1] = 36;
	
	IRShort[17][0] = 0.84;
	IRShort[17][1] = 38;
	
	IRShort[18][0] = 0.8;
	IRShort[18][1] = 40;
	
	IRShort[19][0] = 0.77;
	IRShort[19][1] = 42;
	
	IRShort[20][0] = 0.74;
	IRShort[20][1] = 44;
	
	IRShort[21][0] = 0.7;
	IRShort[21][1] = 46;
	
	IRShort[22][0] = 0.68;
	IRShort[22][1] = 48;
	
	IRShort[23][0] = 0.65;
	IRShort[23][1] = 50;
	
	IRShort[24][0] = 0.64;
	IRShort[24][1] = 52;
	
	IRShort[25][0] = 0.61;
	IRShort[25][1] = 54;
	
	IRShort[26][0] = 0.59;
	IRShort[26][1] = 56;
	
	IRShort[27][0] = 0.57;
	IRShort[27][1] = 58;
	
	IRShort[28][0] = 0.55;
	IRShort[28][1] = 60;
	
	IRShort[29][0] = 0.54;
	IRShort[29][1] = 62;
	
	IRShort[30][0] = 0.53;
	IRShort[30][1] = 64;
	
	IRShort[31][0] = 0.5;
	IRShort[31][1] = 66;
	
	IRShort[32][0] = 0.49;
	IRShort[32][1] = 68;
	
	IRShort[33][0] = 0.47;
	IRShort[33][1] = 70;
	
	IRShort[34][0] = 0.46;
	IRShort[34][1] = 72;
	
	IRShort[35][0] = 0.45;
	IRShort[35][1] = 74;
	
	IRShort[36][0] = 0.44;
	IRShort[36][1] = 76;
	
	IRShort[37][0] = 0.43;
	IRShort[37][1] = 78;
	
	IRShort[38][0] = 0.42;
	IRShort[38][1] = 80;
	
	// 20-150 cm
	IRLong[0][0] = 2.75;
	IRLong[0][1] = 15;
	
	IRLong[1][0] = 2.55;
	IRLong[1][1] = 20;
	
	IRLong[2][0] = 2.5;
	IRLong[2][1] = 22;
	
	IRLong[3][0] = 2.33;
	IRLong[3][1] = 24;
	
	IRLong[4][0] = 2.2;
	IRLong[4][1] = 26;
	
	IRLong[5][0] = 2.09;
	IRLong[5][1] = 28;
	
	IRLong[6][0] = 1.96;
	IRLong[6][1] = 30;
	
	IRLong[7][0] = 1.87;
	IRLong[7][1] = 32;
	
	IRLong[8][0] = 1.74;
	IRLong[8][1] = 34;
	
	IRLong[9][0] = 1.64;
	IRLong[9][1] = 36;
	
	IRLong[10][0] = 1.57;
	IRLong[10][1] = 38;
	
	IRLong[11][0] = 1.47;
	IRLong[11][1] = 40;
	
	IRLong[12][0] = 1.4;
	IRLong[12][1] = 42;
	
	IRLong[13][0] = 1.33;
	IRLong[13][1] = 44;
	
	IRLong[14][0] = 1.29;
	IRLong[14][1] = 46;
	
	IRLong[15][0] = 1.23;
	IRLong[15][1] = 48;
	
	IRLong[16][0] = 1.19;
	IRLong[16][1] = 50;
	
	IRLong[17][0] = 1.11;
	IRLong[17][1] = 52;
	
	IRLong[18][0] = 1.1;
	IRLong[18][1] = 54;
	
	IRLong[19][0] = 1.04;
	IRLong[19][1] = 56;
	
	IRLong[20][0] = 1.01;
	IRLong[20][1] = 58;
	
	IRLong[21][0] = 0.98;
	IRLong[21][1] = 60;
	
	IRLong[22][0] = 0.95;
	IRLong[22][1] = 62;
	
	IRLong[23][0] = 0.93;
	IRLong[23][1] = 64;
	
	IRLong[24][0] = 0.88;
	IRLong[24][1] = 66;
	
	IRLong[25][0] = 0.87;
	IRLong[25][1] = 68;
	
	IRLong[26][0] = 0.86;
	IRLong[26][1] = 70;
	
	IRLong[27][0] = 0.82;
	IRLong[27][1] = 72;
	
	IRLong[28][0] = 0.81;
	IRLong[28][1] = 74;
	
	IRLong[29][0] = 0.79;
	IRLong[29][1] = 76;
	
	IRLong[30][0] = 0.77;
	IRLong[30][1] = 78;
	
	IRLong[31][0] = 0.76;
	IRLong[31][1] = 80;
	
	IRLong[32][0] = 0.73;
	IRLong[32][1] = 82;
	
	IRLong[33][0] = 0.72;
	IRLong[33][1] = 84;
	
	IRLong[34][0] = 0.71;
	IRLong[34][1] = 86;
	
	IRLong[35][0] = 0.69;
	IRLong[35][1] = 88;
	
	IRLong[36][0] = 0.68;
	IRLong[36][1] = 90;
	
	IRLong[37][0] = 0.676;
	IRLong[37][1] = 92;
	
	IRLong[38][0] = 0.66;
	IRLong[38][1] = 94;
	
	IRLong[39][0] = 0.656;
	IRLong[39][1] = 96;
	
	IRLong[40][0] = 0.64;
	IRLong[40][1] = 98;
	
	IRLong[41][0] = 0.63;
	IRLong[41][1] = 100;
	
	IRLong[42][0] = 0.628;
	IRLong[42][1] = 102;
	
	IRLong[43][0] = 0.59;
	IRLong[43][1] = 104;
	
	IRLong[44][0] = 0.585;
	IRLong[44][1] = 106;
	
	IRLong[45][0] = 0.57;
	IRLong[45][1] = 108;
	
	IRLong[46][0] = 0.56;
	IRLong[46][1] = 110;
	
	IRLong[47][0] = 0.535;
	IRLong[47][1] = 112;
	
	IRLong[48][0] = 0.53;
	IRLong[48][1] = 114;
	
	IRLong[49][0] = 0.52;
	IRLong[49][1] = 116;
	
	IRLong[50][0] = 0.517;
	IRLong[50][1] = 118;
	
	IRLong[51][0] = 0.51;
	IRLong[51][1] = 120;
	
	IRLong[52][0] = 0.49;
	IRLong[52][1] = 122;
	
	IRLong[53][0] = 0.48;
	IRLong[53][1] = 124;
	
	IRLong[54][0] = 0.47;
	IRLong[54][1] = 126;
	
	IRLong[55][0] = 0.465;
	IRLong[55][1] = 128;
	
	IRLong[56][0] = 0.46;
	IRLong[56][1] = 130;
	
	IRLong[57][0] = 0.45;
	IRLong[57][1] = 132;
	
	IRLong[58][0] = 0.426;
	IRLong[58][1] = 134;
	
	IRLong[59][0] = 0.424;
	IRLong[59][1] = 136;
	
	IRLong[60][0] = 0.415;
	IRLong[60][1] = 138;
	
	IRLong[61][0] = 0.42;
	IRLong[61][1] = 140;
	
	IRLong[62][0] = 0.402;
	IRLong[62][1] = 142;
	
	IRLong[63][0] = 0.4;
	IRLong[63][1] = 144;
	
	IRLong[64][0] = 0.38;
	IRLong[64][1] = 146;
	
	IRLong[65][0] = 0.375;
	IRLong[65][1] = 148;
	
	IRLong[66][0] = 0.37;
	IRLong[66][1] = 150;
}

uint16_t voltage_to_mm_short(float voltage)
{
	if(voltage >= IRShort[0][0])
	{
		return IRShort[0][1] * 10;
	} else if(voltage <= IRShort[sensorPointsShort - 1][0])
	{
		return IRShort[sensorPointsShort - 1][1] * 10;
	}
	
	for(int i = 0; i < sensorPointsShort; ++i)
	{
		float prev = IRShort[i][0];
		float next = IRShort[i+1][0];
		if(next == voltage)
		{
			return IRShort[i+1][1] * 10;
		} else if(prev > voltage && next < voltage)
		{
			uint16_t high = IRShort[i][1] * 10;
			uint16_t low = IRShort[i+1][1] * 10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (uint16_t) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return IRShort[sensorPointsShort - 1][1] * 10;
}

int voltage_to_mm_long(float voltage)
{
	if(voltage >= IRLong[0][0])
	{
		return IRLong[0][1] * 10;
	} else if(voltage <= IRLong[sensorPointsLong - 1][0])
	{
		return IRLong[sensorPointsLong - 1][1] * 10;
	}
	
	for(int i = 0; i < sensorPointsLong - 1; ++i)
	{
		float prev = IRLong[i][0];
		float next = IRLong[i+1][0];
		if(next == voltage)
		{
			return IRLong[i+1][1] * 10;
		} else if(prev > voltage && next < voltage)
		{
			uint16_t high = IRLong[i][1] * 10;
			uint16_t low = IRLong[i+1][1] * 10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (int) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return IRLong[sensorPointsLong - 1][1] * 10;
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
	sensorDataSentFlag = 0;
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

uint16_t* sensors_get_data()
{
	return gSensorBuffer;
}


ISR(ADC_vect)
{
	cli();	uint8_t adcValue = ADCH;	float vin = adcValue * 5.0 / 256.0;	if(gSelectedSensor == 4)	{		gSensorBuffer[gSelectedSensor] = voltage_to_mm_long(vin);		} else {		gSensorBuffer[gSelectedSensor] = voltage_to_mm_short(vin);	}		if(gSelectedSensor < 6)	{		// Not last sensor		select_sensor(gSelectedSensor + 1);		adc_start();	} else {
		select_sensor(0);		enable_counter_0(1);		set_counter_0(50);		start_ul_sensor();	}	sei();}

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
	if(!sensorDataSentFlag)
	{
		sensorDataSentFlag = 1;
		TWI_send_sensors(sensors_get_data(), 0);
	}
	sei();
}

ISR(TIMER0_COMPA_vect)
{
	if(!sensorDataSentFlag)
	{
		sensorDataSentFlag = 1;
		TWI_send_sensors(sensors_get_data(), 0);
	}
	enable_counter_0(0);
	TCNT0 = 0;
}