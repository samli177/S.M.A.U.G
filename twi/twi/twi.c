/*
 * twi.c
 *
 * Created: 4/4/2014 2:40:42 PM
 *  Author: perjo018
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include "twi.h"


// Global variables for response functions
int my_adress;
char message[255];
int message_counter;
int settings;
int sensor_buffer[7];
int sensors[7];
int servo;
int sweep;
int sensor;
int command[3];
int current_command;
int message_length;

//Global variables for the interrupts
bool instruction;
int current_instruction;
int my_adress;

void init_TWI(uint8_t module_adress)
{
	my_adress = module_adress;
	switch(module_adress)
	{
		case(C_ADRESS):
		{
			PORTC = 0x03; // Pull up, only 1!
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWBR = 0b00111111; //bit rate 27
			TWAR = (1<<TWA6) | (1<<TWGCE); // Address 100 0000, General Call Accepted
			break;
		}
		case(S_ADRESS):
		{
			TWBR = 0b00111111; //bit rate 23
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWAR = (1<<TWA5); // Address 010 0000, General Call Not Accepted
			break;
		}
		case(ST_ADRESS):
		{
			TWBR = 0b00111111; //bit rate
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWAR = (1<<TWA4) | (1<<TWGCE); // Address 001 0000, General Call Accepted
			break;
		}
	}
}

void set_twi_reciever_enable()
{
	TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE); //TWI enable, TWI interrupt enable
}

void Error()
{
	if(CONTROL != ARBITRATION)
	{
		stop_bus();
	}
	else
	{
		clear_int();
	}
}

void clear_int()
{
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWIE);
}

void start_bus()
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
}

void stop_bus()
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

uint8_t get_data()
{
	return TWDR;
}

void send_data_and_wait(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}


bool TWI_send_status(uint8_t adr)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_STATUS);
	if(CONTROL == ARBITRATION)
	{
		clear_int();
		return true;
	}
	stop_bus();
	return true;
}

bool TWI_send_settings(uint8_t set)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(C_ADRESS);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_SETTINGS);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(set);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool TWI_send_sweep(uint8_t pos)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(S_ADRESS);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_SWEEP);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(pos);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool TWI_send_command(uint8_t direction, uint8_t rotation, uint8_t speed)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(ST_ADRESS);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_COMMAND);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(direction);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(rotation);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(speed);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool TWI_send_elevation(uint8_t elevation)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(ST_ADRESS);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_ELEVATION);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(elevation);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool TWI_send_sensors(uint8_t sens[7], uint8_t serv)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(G_ADRESS);//General Call, NO instruction byte, NO NACK control of data
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	for(int i=0; i < 7; ++i) //7 Sensors?
	{
		send_data_and_wait(sens[i]);
	}
	send_data_and_wait(serv);
	stop_bus();
	return true;
}

bool TWI_send_string(uint8_t adr, char str[])
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_STRING);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	for(int i = 0; i < strlen(str); ++i)
	{
		send_data_and_wait(str[i]);
	}
	stop_bus();
	return true;
}

bool TWI_send_string_fixed_length(uint8_t adr, uint8_t str[], int length)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(I_STRING);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	for(int i = 0; i < length; ++i)
	{
		send_data_and_wait(str[i]);
	}
	stop_bus();
	return true;
}

bool TWI_send_something(uint8_t adr, uint8_t instruction, uint8_t packet)
{
	start_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(instruction);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	send_data_and_wait(packet);
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

//------------------------------------------------------------------------------------------

void stop_twi()
{
	current_command = 0;
	sensor = 0;
	message_counter = 0;
}

void reset_TWI()
{
	TWCR |= (1<<TWINT) | (1<<TWEA);
}

void get_sensor_from_bus()
{
	if(sensor == 7)
	{
		for(int i = 0; i < sensor;++i)
		{
			sensors[i] = sensor_buffer[i];
		}
		servo = get_data();
	}
	else
	{
		sensor_buffer[sensor] = get_data();
		sensor += 1;
	}
}

void get_sweep_from_bus()
{
	sweep = get_data();
}

void get_settings_from_bus()
{
	settings = get_data();
}

void get_char_from_bus()
{
	message[message_counter] = get_data();
	message_counter += 1;
	message_length = message_counter;
}

void get_command_from_bus()
{
	command[current_command] = get_data();
	current_command += 1;
}

//------------------------------------------------------------------------------------------
uint8_t TWI_get_command(int i)
{
	return command[i];
}

uint8_t TWI_get_message_length()
{
	return message_length;
}

char TWI_get_char(int i)
{
	return message[i];
}

uint8_t TWI_get_sensor(int i)
{
	return sensors[i];
}

uint8_t TWI_get_servo()
{
	return servo;
}

uint8_t TWI_get_sweep()
{
	return sweep;
}

uint8_t TWI_get_settings()
{
	return settings;
}

//TWI Interrupt vector MUHAHAHAHA
// ----------------------------------------------------------------------------- Communications
ISR(TWI_vect)
{
	switch(my_adress)
	{
		case(C_ADRESS):
		{
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
						case(I_SETTINGS):
						{
							get_settings_from_bus();
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
			else if (CONTROL == DATA_GENERAL)
			{
				get_sensor_from_bus();
			}
			else if (CONTROL == STOP)
			{
				stop_twi();
			}
			reset_TWI();
			break;
		}
		// ----------------------------------------------------------------------------- Sensors
		case(S_ADRESS):
		{
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
				stop_twi();
				switch(current_instruction)
				{
					case(I_STRING):
					{
						get_char_from_bus();
						break;
					}
				}
			}
			reset_TWI();
			break;
		}
		// ----------------------------------------------------------------------------- Steer
		case(ST_ADRESS):
		{
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
						case(I_COMMAND):
						{
							get_command_from_bus();
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
			else if (CONTROL == DATA_GENERAL)
			{
				get_sensor_from_bus();
			}
			else if (CONTROL == STOP)
			{
				stop_twi();
			}
			reset_TWI();
			break;
		}
	}
}