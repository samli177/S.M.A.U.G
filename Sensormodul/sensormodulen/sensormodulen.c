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

// -- Global variables --

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

int main(void)
{
	init_display();
	// init TWI
	my_adress = S_ADRESS;
	init_TWI(my_adress);
	
	sei();
	displaytest();
	
	// testcode
	_delay_ms(1000);

	send_string(C_ADRESS, "kom fram");
	clear_display();
	print_text("skickas");
	while(1)
	{
		
	}
}

void init_mux()
{
	DDRA |= 0b00111110;
	DDRA &= !(1<<PORTA0);
	PORTA &= !(1<<PORTA5);
	PORTA &= 0b11100001;
}

void init_UL()
{
	DDRA |= (1<<PORTA7);
	DDRA &= !(1<<PORTA6);
	TCCR0B = 0x05;
}

void select_sensor(int sensor)
{
	PORTA &= 0b11100001;
	switch(sensor)
	{
		case(8):
		{
			break;
		}
		case(7):
		{
			PORTA |= 0b00000010;
			break;
		}
		case(6):
		{
			PORTA |= 0b00001110;
			break;
		}
		case(5):
		{
			PORTA |= 0b00001100;
			break;
		}
		case(4):
		{
			PORTA |= 0b00001010;
			break;
		}
		case(3):
		{
			PORTA |= 0b00001000;
			break;
		}
		case(2):
		{
			PORTA |= 0b00000110;
			break;
		}
		default:
		{
			PORTA |= 0b00000100;
			break;
		}
	}
}

int get_sensor()
{
	return (PORTA & (1<<PORTA0));
}

unsigned int UL_sensor()
{
	unsigned int i;
	PORTA |= (1<<PORTA7);
	_delay_us(15);
	PORTA &= !(1<<PORTA7);
	//while(!(PORTA & (1<<PORTA6)));
	TCNT0 = 0;
	while((PORTA & (1<<PORTA6)));
	i = TCNT0;
	return i;
}

void displaytest(void)
{
	print_line(0, "Initiating AI");
}

/*
void init_TWI_sensor(void)
{
	TWBR = (1<<TWBR0); //bit rate
	TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE); //TWI enable, TWI interrupt enable
	//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
	TWAR = (1<<TWA3); // Address 001000, General Call Not Accepted	
}
*/

void send_data(void) //Gör om så att all data skickas i rad med repeated START, lägg till säkerhet!
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//if ((TWSR & 0xF8) != 0x08)	
	TWDR = 0; //General Call
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//TWDR = Sensornr, Servoposition
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	//TWDR = Sensorutslag
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

// TWI interrupt vector

/* ISR(TWI_vect)
{
	if (TWSR == 0x60)
		//Dags att utföra ett svep (och sen skicka resultat kanske?)
	TWCR |= (1<<TWINT) | (1<<TWEA);
} */

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
							PORTA |= (1<<PORTA1);
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
				clear_display();
				
				switch(current_instruction)
				{
					case(I_STRING):
					{
						for(int i = 0; i < message_counter; ++i)
						{
							print_char(message[i]);
						}
						
						break;
					}
				}
				
				current_instruction = 0;
				
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