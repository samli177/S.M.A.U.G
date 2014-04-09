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

#include "display.h"
#include "twi.h"

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

// -- Global variables --

uint8_t gSelectedSensor = 0;
int gSensorBuffer[7]; // NOTE: should probably be uint8_t

float IR_short[13][2];
char display_buffer[64][20];
int buffer_size = 0;

int my_adress;
bool instruction;
int current_instruction;

int UL;

int main(void)
{
	init_display();
	// init TWI
	my_adress = S_ADRESS;
	init_TWI(my_adress);
	
	
	
	adc_init();
	init_tables();
	
	sei();
	
	// Test code for sensor
	
	print_text("Testing ADC");
	_delay_ms(1000);
	init_mux();
	init_UL();
	while(1)
	{
		UL_sensor();
		_delay_ms(3000);
		clear_display();
		print_value(UL);
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
	}
	
	//displaytest();
	
	// testcode
	/*_delay_ms(1000);

	send_string(C_ADRESS, "kom fram");
	clear_display();
	print_text("skickas");*/
	
	//display top in buffer and sensordata
	clear_display();
	if(buffer_size > 0)
	{
		//Display top row:
		for(int i = 0; i < display_buffer[0][0]; ++i)
		{
			print_char(display_buffer[i+1][0]);
		}
		//Delete top row:
		for(int it_row = 0; it_row <= buffer_size; ++it_row)
		{
			for(int it_col = 0; it_col <= display_buffer[0][it_row + 1]; ++it_col)
			{
				display_buffer[it_col][it_row] = display_buffer[it_col][it_row + 1];
			}
		}
		buffer_size = buffer_size - 1;
	}
	else
	{
		//display sensordata
		for(int i = 0; i < 8; ++i)
		{
			print_value(gSensorBuffer[i]);
			print_text(", ");
			
		}
	}
}

void init_tables()
{
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
	
}

int voltage_to_mm(float voltage)
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


ISR(ADC_vect)
{
	cli();
	/*
	clear_display();
	print_line(0, "ADC complete");	set_display_pos(1, 0);	 	print_value(voltage_to_mm(vin));	print_text(", ");	print_value((int)(vin*100));	*/	clear_display();	print_value(gSelectedSensor);		uint8_t adcValue = ADCH;	float vin = adcValue * 5.0 / 256.0;		gSensorBuffer[gSelectedSensor] = voltage_to_mm(vin)/10;	print_text(", ");	print_value(voltage_to_mm(vin)/10);	sei();}

void init_mux()
{
	DDRA |= 0b00111110;
	DDRA &= ~(1<<PORTA0);
	PORTA &= ~(1<<PORTA5);
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
	// ADC enabled, enable interupt, set division factor for clock to be 128
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

void select_sensor(int sensor)
{
	gSelectedSensor = sensor;
	PORTA &= ~((1<<PORTA1) | (1<<PORTA2) | (1<<PORTA3) | (1<<PORTA4));
	switch(sensor)
	{
		case(0):
			// Do nothing
			break;
		case(1):
			PORTA |= 1<<PORTA1;
			break;
		case(2):
			PORTA |= 1<<PORTA2;
			break;
		case(3):
			PORTA |= 1<<PORTA1 | 1<<PORTA2;
			break;
		case(4):
			PORTA |= 1<<PORTA3;
			break;
		case(5):
			PORTA |= 1<<PORTA1 | 1<<PORTA3;
			break;
		case(6):
			PORTA |= 1<<PORTA2 | 1<< PORTA3;
			break;
		case(7):
			PORTA |= 1<<PORTA1 | 1<<PORTA2 | 1<<PORTA3;
			break;
		default:
			// Do nothing
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
	if(PINA & (1<<PINA6))
	{
		TCNT0 = 0;
		PORTD |= (1<<PORTD0);
	}
	else
	{
		UL = TCNT0;
		UL = (UL * 340 / (2 * 15625));
		PORTD &= ~(1<<PORTD0);
	}
}

void displaytest(void)
{
	print_line(0, "Initiating AI");
}

// TWI interrupt vector
ISR(TWI_vect)
{
	cli();
	if(CONTROL == SLAW || CONTROL == ARBIT_SLAW)
	{
		instruction = true;
	}
	else if(CONTROL == DATA_SLAW)
	{
		if(instruction)
		{
			current_instruction = get_data();
			instruction = false;
		}
		else
		{
			switch(current_instruction)
			{
				case(I_SWEEP):
				{
					get_sweep_from_bus();
					break;
				}
				case(I_STRING):
				{
					get_char_from_bus();
					break;
				}
			}
		}
	}
	else if (CONTROL == STOP)
	{
		switch(current_instruction)
		{
			case(I_SWEEP):
			{
				// TODO: change to get_sweep_from_bus() or implement get_sweep()...
				print_value(get_sweep());
				break;
			}
			case(I_STRING):
			{
				display_buffer[0][buffer_size] = get_message_length();
				for(int i = 0; i < get_message_length(); ++i)
				{
					// TODO: change to get_char_from_bus() or implement get_char()...
					//Take char and put in display_buffer in top empty row
					display_buffer[i+1][buffer_size] = get_char(i);
				}
				buffer_size = buffer_size + 1;
				break;
			}
		}
		stop_twi();
	}
	reset_TWI();
	sei();
}
