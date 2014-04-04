/*
 * twi.c
 *
 * Created: 4/4/2014 2:40:42 PM
 *  Author: perjo018
 */ 

#define F_CPU 18432000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include "twi.h"

void init_TWI(int module_adress)
{
	switch(module_adress)
	{
		case(C_ADRESS):
		{
			PORTC = 0x03; // Pull up, only 1!
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWBR = 0b00011011; //bit rate 27 => clk = 79.448 kHz
			TWAR = (1<<TWA6) | (1<<TWGCE); // Address 100 0000, General Call Accepted
			break;
		}
		case(S_ADRESS):
		{
			TWBR = 0b00010111; //bit rate 23 => clk = 80.0 kHz
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWAR = (1<<TWA5); // Address 010 0000, General Call Not Accepted
			break;
		}
		case(ST_ADRESS):
		{
			TWBR = 0b00010111; //bit rate 23 => clk = 80.0 kHz
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
	_delay_ms(100);
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
}

void stop_bus()
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

void set_data(int b)
{
	TWDR = b;
}

int get_data()
{
	return TWDR;
}

void send_bus()
{
	TWCR = (1<<TWINT) | (1<<TWEN);
}

void wait_for_bus()
{
	while (!(TWCR & (1<<TWINT)));
}

bool send_status(int adr)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(adr);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_STATUS);
	send_bus();
	wait_for_bus();
	if(CONTROL == ARBITRATION)
	{
		return true;
	}
	else if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool send_settings()
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(C_ADRESS);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_SETTINGS);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	//set_data(settings)
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool send_sweep(int pos)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(S_ADRESS);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_SWEEP);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	set_data(pos);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool send_command(int direction, int rot_elev, int speed)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(ST_ADRESS);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_COMMAND);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	set_data(direction);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	set_data(rot_elev);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	set_data(speed);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

bool send_sensors(int sens[8], int serv)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(G_ADRESS); //General Call, NO instruction byte, NO NACK control of data
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	for(int i=0; i < 8; ++i) //8 Sensors?
	{
		set_data(sens[i]);
		send_bus();
		wait_for_bus();
	}
	set_data(serv);
	send_bus();
	wait_for_bus();
	stop_bus();
	return true;
}

bool send_string(int adr, char str[])
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(adr);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_STRING);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	for(int i = 0; i < strlen(str); ++i)
	{
		set_data(str[i]);
		send_bus();
		wait_for_bus();
	}
	stop_bus();
	return true;
}


bool send_string_fixed_length(int adr, uint8_t str[], int len)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(adr);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(I_STRING);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	for(int i = 0; i < len; ++i)
	{
		set_data(str[i]);
		send_bus();
		wait_for_bus();
	}
	stop_bus();
	return true;
}

bool send_something(int adr, int instruction, int packet)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return false;
	}
	set_data(adr);
	send_bus();
	wait_for_bus();
	if(CONTROL != ADRESS_W)
	{
		Error();
		return false;
	}
	set_data(instruction);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	set_data(packet);
	send_bus();
	wait_for_bus();
	if(CONTROL != DATA_W)
	{
		Error();
		return false;
	}
	stop_bus();
	return true;
}

void reset_TWI()
{
	TWCR |= (1<<TWINT) | (1<<TWEA);
}

/*
//TWI Interrupt vector to exist in every module
ISR(TWI_vect)
{
	switch(my_adress)
	{
		// ----------------------------------------------------------------------------- Communications
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
							PORTA |= (1<<PORTA0);
							settings = get_data();
							break;
						}
						case(I_STRING):
						{
							message[message_counter] = get_data();
							message_counter += 1;
							break;
						}
					}
				}
			}
			else if (CONTROL == DATA_GENERAL)
			{
				if(sensor == 8)
				{
					for(int i = 0; i < sizeof(sensors)/sizeof(int);++i)
					{
						sensors[i] = buffer[i];
					}
					servo = get_data();
				}
				else
				{
					buffer[sensor] = get_data();
					sensor += 1;
				}
			}
			else if (CONTROL == STOP)
			{
				sensor = 0;
				//Do something smart with the message.
				message_counter = 0;
			}
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
							//sweep = get_data(); ? :O
							break;
						}
						case(I_STRING):
						{
							message[message_counter] = get_data();
							message_counter += 1;
							break;
						}
					}
				}
			}
			else if (CONTROL == STOP)
			{
				//Gör något smart med message
				message_counter = 0;
			}
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
							command[current_command] = get_data();
							current_command += 1;
							break;
						}
						case(I_STRING):
						{
							message[message_counter] = get_data();
							message_counter += 1;
							break;
						}
					}
				}
			}
			else if (CONTROL == DATA_GENERAL)
			{
				if(sensor == 8)
				{
					for(int i = 0; i < sizeof(sensors)/sizeof(int);++i)
					{
						sensors[i] = buffer[i];
					}
					servo = get_data();
				}
				else
				{
					buffer[sensor] = get_data();
					sensor += 1;
				}
			}
			else if (CONTROL == STOP)
			{
				sensor = 0;
				current_command = 0;
				//Do something with the commands.
				//Nothing to do with a string here really...
				message_counter = 0;
			}
		}
		break;
	}
	reset_TWI();
}


// Global variables
// to exist in the module!
int my_adress;
char message[255];
int message_counter;
bool instruction;
int current_instruction;
int settings;
int buffer[8];
int sensors[8];
int servo;
int sensor;
int command[3];
int current_command;

*/