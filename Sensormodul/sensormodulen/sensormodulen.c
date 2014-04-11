/*
* Sensormodulen.c
*
* Created: 3/28/2014 9:43:46 AM
*  Author: perjo018
*/


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>

#include "display.h"
#include "twi.h"
#include "fifo.h"

void send_data(void);
void init_TWI_sensor(void);
void adc_init();
void adc_enable(bool enable);
void adc_start();
void init_mux();
void select_sensor(int sensor);
void init_tables();
unsigned int UL_sensor();
void init_UL();
int voltage_to_cm(float voltage);
void print_sensor_data();

uint8_t gSelectedSensor = 0;
int gSensorBuffer[8]; // NOTE: should probably be uint8_t

float IR_short[13][2];
float IR_long[15][2];

char display_buffer[64][20];
int buffer_size = 0;

int UL;
void init_counter();
void set_counter(uint16_t delay);

bool sensor_data_flag = false;

// define FIFO for received packets (USART)
MK_FIFO(4096); // use 4 kB
DEFINE_FIFO(gTwiFIFO, 4096);

uint8_t decode_message_TwiFIFO();
uint8_t write_to_FIFO(char msg[]);

int main(void)
{
	init_display();
	// init TWI

	TWI_init(S_ADRESS);
	init_counter();
	set_counter(2000);
	adc_init();
	init_tables();
	
	write_to_FIFO("test");

	// Test code for sensor
	
	print_text("Testing ADC");
	_delay_ms(1000);
	init_mux();
	init_UL();
	while(1)
	{
		
		_delay_ms(1000);
		//if(sensor_data_flag)
			//print_sensor_data();
			
		decode_message_TwiFIFO();
		
			
		/*UL_sensor();
		_delay_ms(3000);
		clear_display();
		print_value(UL);*/
		/*for(int i = 0; i < 8; ++i)
		{
			select_sensor(i);
			adc_start();
			_delay_ms(10);
		}*/
		
		//send_sensors(gSensorBuffer, 0);
		
		//clear_display();
		//print_text("Sensors sent");
		/*for(int i = 0; i < 8; ++i)
		{
			print_value(gSensorBuffer[i]);
			print_text(", ");
			
		}*/

		//TWI_send_autonom_settings(C_ADRESS, 5);
		//_delay_ms(680);

	}
	
	//displaytest();
	
	// testcode
	/*_delay_ms(1000);

	send_string(C_ADRESS, "kom fram");
	clear_display();
	print_text("skickas");*/
	
	//display top in buffer or sensordata
	clear_display();
	if(decode_message_TwiFIFO())
	{
		//display sensordata
		print_sensor_data();
	}
}

void init_tables()
{
	// 10-80 cm
	IR_short[0][0] = 3.15;
	IR_short[0][1] = 6;
	
	IR_short[1][0] = 2.98;
	IR_short[1][1] = 7;

	IR_short[2][0] = 2.75;
	IR_short[2][1] = 8;
	
	IR_short[3][0] = 2.31;
	IR_short[3][1] = 10;
	
	IR_short[4][0] = 1.64;
	IR_short[4][1] = 15;
	
	IR_short[5][0] = 1.31;
	IR_short[5][1] = 20;
	
	IR_short[6][0] = 1.08;
	IR_short[6][1] = 25;
	
	IR_short[7][0] = 0.92;
	IR_short[7][1] = 30;
	
	IR_short[8][0] = 0.74;
	IR_short[8][1] = 40;
	
	IR_short[9][0] = 0.61;
	IR_short[9][1] = 50;
	
	IR_short[10][0] = 0.51;
	IR_short[10][1] = 60;
	
	IR_short[11][0] = 0.45;
	IR_short[11][1] = 70;
	
	IR_short[12][0] = 0.41;
	IR_short[12][1] = 80;
	
	// 20-150 cm
	IR_long[0][0] = 2.75;
	IR_long[0][1] = 15;
	
	IR_long[1][0] = 2.55;
	IR_long[1][1] = 20;
	
	IR_long[2][0] = 2.00;
	IR_long[2][1] = 30;
	
	IR_long[3][0] = 1.55;
	IR_long[3][1] = 40;
	
	IR_long[4][0] = 1.25;
	IR_long[4][1] = 50;
	
	IR_long[5][0] = 1.07;
	IR_long[5][1] = 60;
	
	IR_long[6][0] = 0.85;
	IR_long[6][1] = 70;
	
	IR_long[7][0] = 0.80;
	IR_long[7][1] = 80;
	
	IR_long[8][0] = 0.75;
	IR_long[8][1] = 90;
	
	IR_long[9][0] = 0.65;
	IR_long[9][1] = 100;
	
	IR_long[10][0] = 0.60;
	IR_long[10][1] = 110;
	
	IR_long[11][0] = 0.55;
	IR_long[11][1] = 120;
	
	IR_long[12][0] = 0.50;
	IR_long[12][1] = 130;
	
	IR_long[13][0] = 0.45;
	IR_long[13][1] = 140;
	
	IR_long[14][0] = 0.42;
	IR_long[14][1] = 150;
}

int voltage_to_mm_short(float voltage)
{	
	if(voltage >= IR_short[0][0])
	{
		return IR_short[0][1]*10;
	} else if(voltage <= IR_short[12][0])
	{
		return IR_short[12][1]*10;
	}
	
	for(int i = 0; i < 13; ++i)
	{
		float prev = IR_short[i][0];
		float next = IR_short[i+1][0];
		if(next == voltage)
		{
			return IR_short[i+1][1]*10;
		} else if(prev > voltage && next < voltage)
		{
			int high = IR_short[i][1]*10;
			int low = IR_short[i+1][1]*10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (int) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return 0;
}

int voltage_to_mm_long(float voltage)
{
	if(voltage >= IR_long[0][0])
	{
		return IR_long[0][1]*10;
	} else if(voltage <= IR_long[14][0])
	{
		return IR_long[14][1]*10;
	}
	
	for(int i = 0; i < 13; ++i)
	{
		float prev = IR_long[i][0];
		float next = IR_long[i+1][0];
		if(next == voltage)
		{
			return IR_long[i+1][1]*10;
		} else if(prev > voltage && next < voltage)
		{
			int high = IR_long[i][1]*10;
			int low = IR_long[i+1][1]*10;
			int diff = high - low;
			float diff_to_prev = prev - voltage;
			float volt_diff = prev - next;
			return (int) (high - diff * diff_to_prev / volt_diff);
		}
	}
	
	return 0;
}



ISR(ADC_vect)
{
	cli();	uint8_t adcValue = ADCH;	float vin = adcValue * 5.0 / 256.0;	if(gSelectedSensor == 4)	{		gSensorBuffer[gSelectedSensor] = voltage_to_mm_long(vin)/10;	} else {		gSensorBuffer[gSelectedSensor] = voltage_to_mm_short(vin)/10;	}			if(gSelectedSensor < 6)	{		// Not last sensor		select_sensor(gSelectedSensor + 1);		adc_start();	} else {
		select_sensor(0);
		UL_sensor();	}	sei();}

void init_mux()
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

void init_counter()
{
	// WGMn3:0 = 4 (OCRnA) or 12 (OCRn), where top value is read from.
	TCCR1B |= 0b00000101; // Choosing clock. (clkI/O/1024)
	
	// standardvalue for interrupt is 1000ms
	set_counter(1000);
	
	TIMSK1 |= 0b00000010; // Enable interrupts when OCF1A, in FIFR1, is set. 
	// OCF1A (or ICFn) Flag, in TIFR1, can be used to generate interrupts.
	TCNT1 = 0;
}

void set_counter(uint16_t delay)
{
	delay = 15.625 * delay;
	OCR1A = delay;
}

ISR(TIMER1_COMPA_vect)
{
	adc_start();
	TCNT1 = 0;
}

void adc_init()
{
	// ADC enabled, enable interrupt, set division factor for clock to be 128
	ADCSRA = (1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);
	// Disable auto trigger
	// ADCSRA &= ~(1<<ADATE);
	
	// Left adjust, set voltage reference selection
	ADMUX = 1<<ADLAR | 1<<REFS0;
}

void adc_enable(bool enable)
{
	if(enable)
	{
		ADCSRA |= 1<<ADEN;
	}
	else
	{
		ADCSRA &= !(1<<ADEN);
	}
}

void adc_start()
{
	ADCSRA |= 1<<ADSC;
}

void print_sensor_data()
{
	clear_display();
	
	set_display_pos(0,0);
	print_text("0: ");
	print_value(gSensorBuffer[0]);
	
	set_display_pos(0,8);
	print_text("1: ");
	print_value(gSensorBuffer[1]);
	
	set_display_pos(1,0);
	print_text("2: ");
	print_value(gSensorBuffer[2]);
	
	set_display_pos(1,8);
	print_text("3: ");
	print_value(gSensorBuffer[3]);
	
	set_display_pos(2,0);
	print_text("4: ");
	print_value(gSensorBuffer[4]);
	
	set_display_pos(2,8);
	print_text("5: ");
	print_value(gSensorBuffer[5]);
	
	set_display_pos(3,0);
	print_text("6: ");
	print_value(gSensorBuffer[6]);
	
	set_display_pos(3,8);
	print_text("7: ");
	print_value(gSensorBuffer[7]);
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



unsigned int UL_sensor()
{
	//unsigned int i;
	TCNT0 = 0;
	PORTA |= (1<<PORTA7);
	_delay_us(15);
	PORTA &= ~(1<<PORTA7);
	
	/*TCNT0 = 0;
	if(PORTA & (1<<PORTA6))
		while((PORTA & (1<<PORTA6)));
	else
	{
		while(!(PORTA & (1<<PORTA6)));
		while((PORTA & (1<<PORTA6)));
	}
	i = TCNT0;*/
	return 1;
}

ISR(PCINT0_vect)
{
	cli();
	if(PINA & (1<<PINA6))
	{
		TCNT0 = 0;
	}
	else
	{
		UL = TCNT0;
		gSensorBuffer[7] = UL;
		sensor_data_flag = true;
		//UL = (UL * 340 / (2 * 15625));
	}
	sei();
}

void displaytest(void)
{
	print_line(0, "Initiating AI");
}

uint8_t decode_message_TwiFIFO()
{
	
	uint8_t *len = 0;
	uint8_t *character = 0;
	
	if(FifoRead(gTwiFIFO, len))
	{
		//print_text("TwiFIFO ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len; // I don't know why I can't use *len directly... but it took me 4h to figure out that you can't do it....
	
	//NOTE: there has to be a better way of doing this...
	int ifzero = 0;
	if(length == 0) ifzero = 1;
	char msg[length-1+ifzero];

	for(int i = 0; i < length; ++i)
	{
		if(FifoRead(gTwiFIFO, character))
		{
			//print_text("TwiFIFO ERROR: DATA MISSING");
			return 1; // error
		}

		msg[i] = *character;
	}
	
	
	// TODO: send to relevant party... the display for now
	
	print_text_fixed_length(msg, length);
	
	return 0;
}

uint8_t write_to_FIFO(char msg[])
{
	if(FifoWrite(gTwiFIFO, (unsigned char)strlen(msg)))
	{
		print_text("TwiFIFO ERROR: 1");
		return 1;
	}
	
	for(int i = 0; i < strlen(msg); ++i)
	{
		if(FifoWrite(gTwiFIFO, msg[i]))
		{
			print_text("TwiFIFO ERROR: 2");
			return 1;
		}
	}
	
	return 0;
}

