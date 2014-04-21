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
#include <string.h>
#include "twi.h"
#include "display.h"
#include "fifo.h"

static void set_twi_reciever_enable();
static void Error();
static void start_bus();
static void stop_bus();
static void clear_int();
static uint8_t get_data();
static void send_data_and_wait(uint8_t data);
static void wait_for_bus();

static void stop_twi();
static void reset_TWI();
static void get_control_settings_from_bus();
static void get_autonom_settings_from_bus();
static void get_char_from_bus();
static void get_sweep_from_bus();
static void get_command_from_bus();
static void get_float_from_bus();
static void get_sensor_from_bus();
static void get_elevation_from_bus();

// Global variables for response functions
uint8_t myAdress;
char message[255];
int messageCounter;
uint8_t controlSettings[3]; //KP, KI, KD
uint8_t sensorBuffer[8];
uint8_t sensors[8];
uint8_t servo;
uint8_t sweep;
int sensor;
uint8_t command[3];
int currentCommand;
int messageLength;
uint8_t autonomSettings;
int currentSetting;
int elevation;
uint8_t floatMessage[4];
int floatCounter;

//Global variables for the interrupts
uint8_t instruction;
int currentInstruction;

//Flags for new data, should be set 0 when read 1
uint8_t sensorFlag_ = 0;
uint8_t commandFlag_ = 0;
uint8_t controlSettingsFlag_ = 0;
uint8_t autonomSettingsFlag_ = 0;
uint8_t elevationFlag_ = 0;
uint8_t sweepFlag_ = 0;


// define FIFO for received packets (USART)
MK_FIFO(4096); // use 4 kB
DEFINE_FIFO(gTwiFIFO, 4096);


void TWI_init(uint8_t moduleAdress)
{
	myAdress = moduleAdress;
	switch(moduleAdress)
	{
		case(C_ADDRESS):
		{
			PORTC = 0x03; // Pull up, only 1!
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWBR = 0b00111111; //bit rate 27
			TWAR = (1<<TWA6) | (1<<TWGCE); // Address 100 0000, General Call Accepted
			break;
		}
		case(S_ADDRESS):
		{
			TWBR = 0b00111111; //bit rate 23
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWAR = (1<<TWA5); // Address 010 0000, General Call Not Accepted
			break;
		}
		case(ST_ADDRESS):
		{
			TWBR = 0b00111111; //bit rate
			set_twi_reciever_enable();
			//TWSR = (0<<TWPS0) | (0<<TWPS1); //Prescaler 0 0 -> 1
			TWAR = (1<<TWA4) | (1<<TWGCE); // Address 001 0000, General Call Accepted
			break;
		}
	}
}

union Union_floatcast
{
	float f;
	char s[sizeof(float)];	
};

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
	wait_for_bus();
}

void wait_for_bus()
{
	while (!(TWCR & (1<<TWINT)));
}

uint8_t TWI_send_status(uint8_t adr)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_STATUS);
	if(CONTROL == ARBITRATION)
	{
		clear_int();
		return 1;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_control_settings(uint8_t adr, uint8_t KP,uint8_t KI,uint8_t KD)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_SETTINGS);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(KP);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(KI);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(KD);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_autonom_settings(uint8_t adr,uint8_t autoset)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_AUTONOM);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(autoset);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_sweep(uint8_t pos)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(S_ADDRESS);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_SWEEP);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(pos);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_command(uint8_t direction, uint8_t rotation, uint8_t speed)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(ST_ADDRESS);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_COMMAND);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(direction);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(rotation);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(speed);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_elevation(uint8_t elevation)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(ST_ADDRESS);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_ELEVATION);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(elevation);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_sensors(uint8_t sens[8], uint8_t serv)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(G_ADDRESS);//General Call, NO instruction byte, NO NACK control of data
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	for(int i=0; i < 8; ++i) //7 Sensors?
	{
		send_data_and_wait(sens[i]);
	}
	send_data_and_wait(serv);
	stop_bus();
	return 1;
}

uint8_t TWI_send_string(uint8_t adr, char str[])
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_STRING);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	for(int i = 0; i < strlen(str); ++i)
	{
		send_data_and_wait(str[i]);
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_string_fixed_length(uint8_t adr, uint8_t str[], int length)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_STRING);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	for(int i = 0; i < length; ++i)
	{
		send_data_and_wait(str[i]);
	}
	stop_bus();
	return 1;
}


uint8_t TWI_send_float(uint8_t adr, float flo)
{
	union Union_floatcast foo;
	foo.f = flo;
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(I_FLOAT);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	for(int i = 0; i < 4; ++i)
	{
		send_data_and_wait(foo.s[i]);
	}
	stop_bus();
	return 1;
}

uint8_t TWI_send_something(uint8_t adr, uint8_t instruction, uint8_t packet)
{
	start_bus();
	wait_for_bus();
	if(CONTROL != START)
	{
		Error();
		return 0;
	}
	send_data_and_wait(adr);
	if(CONTROL != ADDRESS_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(instruction);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	send_data_and_wait(packet);
	if(CONTROL != DATA_W)
	{
		Error();
		return 0;
	}
	stop_bus();
	return 1;
}

//------------------------------------------------------------------------------------------

void stop_twi()
{
	currentCommand = 0;
	sensor = 0;
	messageCounter = 0;
	currentSetting = 0;
	floatCounter = 0;
}

void reset_TWI()
{
	TWCR |= (1<<TWINT) | (1<<TWEA);
}

void get_sensor_from_bus()
{
	if(sensor == 8)
	{
		for(int i = 0; i < sensor;++i)
		{
			sensors[i] = sensorBuffer[i];
		}
		servo = get_data();
		sensorFlag_ = 1;
	}
	else
	{
		sensorBuffer[sensor] = get_data();
		sensor += 1;
	}
}

void get_sweep_from_bus()
{
	sweep = get_data();
}

void get_control_settings_from_bus()
{
	controlSettings[currentSetting] = get_data();
	currentSetting = 0;
}

void get_autonom_settings_from_bus()
{
	autonomSettings = get_data();
}

void get_char_from_bus()
{
	message[messageCounter] = get_data();
	messageCounter += 1;
	messageLength = messageCounter;
}

void get_command_from_bus()
{
	command[currentCommand] = get_data();
	currentCommand += 1;
}

void get_float_from_bus()
{
	floatMessage[floatCounter] = get_data();
	floatCounter += 1;
}

void get_elevation_from_bus()
{
	elevation += get_data();
	if(elevation < 1)
		elevation = 1;
	else if(elevation > 7) // 7 nivåer?!
		elevation = 7;
}


//------------------------------------------------------------------------------------------
uint8_t TWI_get_command(int i)
{
	return command[i];
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

uint8_t TWI_get_control_setting(int i)
{
	return controlSettings[i];
}

uint8_t TWI_get_autonom_settings()
{
	return autonomSettings;
}

uint8_t TWI_get_elevation()
{
	return elevation;
}

//----------------------Flags----------------------------------------------------------------
uint8_t TWI_sensor_flag()
{
	if(sensorFlag_)
	{
		sensorFlag_ = 0;
		return 1;
	}
	return 0;
}

uint8_t TWI_command_flag()
{
	if(commandFlag_)
	{
		commandFlag_ = 0;
		return 1;
	}
	return 0;
}

uint8_t TWI_control_settings_flag()
{
	if(controlSettingsFlag_)
	{
		controlSettingsFlag_ = 0;
		return 1;
	}
	return 0;
}

uint8_t TWI_autonom_settings_flag()
{
	if(autonomSettingsFlag_)
	{
		autonomSettingsFlag_ = 0;
		return 1;
	}
	return 0;
}

uint8_t TWI_elevation_flag()
{
	if(elevationFlag_)
	{
		elevationFlag_ = 0;
		return 1;
	}
	return 0;
}

uint8_t TWI_sweep_flag()
{
	if(sweepFlag_)
	{
		sweepFlag_ = 0;
		return 1;
	}
	return 0;
}

//------------------------------------------------------------------------------FIFO

uint8_t decode_message_TwiFIFO()
{
	
	uint8_t *len = 0;
	uint8_t *character = 0;
	
	if(FifoRead(gTwiFIFO, len))
	{
		//No new messages
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
			display_text("FIFO ERROR 2!");
			return 1; // error
		}

		msg[i] = *character;
	}
	
	
	// TODO: send to relevant party... the display for now
	
	display_clear();
	display_text_fixed_length(msg, length);
	
	return 0;
}

uint8_t write_to_TwiFIFO(char msg[])
{
	if(FifoWrite(gTwiFIFO, (unsigned char)strlen(msg)))
	{
		display_text("FIFO ERROR 3");
		return 1;
	}
	
	for(int i = 0; i < strlen(msg); ++i)
	{
		if(FifoWrite(gTwiFIFO, msg[i]))
		{
			display_text("FIFO ERROR 4");
			return 1;
		}
	}
	
	return 0;
}


//TWI Interrupt vector MUHAHAHAHA
// ----------------------------------------------------------------------------- Communications
ISR(TWI_vect)
{
	switch(myAdress)
	{
		case(C_ADDRESS):
		{
			if(CONTROL == SLAW || CONTROL == ARBIT_SLAW)
			{
				instruction = 1;
				
			}
			else if(CONTROL == DATA_SLAW)
			{
				if(instruction)
				{
					currentInstruction = get_data();
					instruction = 0;
				}
				else
				{
					switch(currentInstruction)
					{
						case(I_SETTINGS):
						{
							get_control_settings_from_bus();
							break;
						}
						case(I_AUTONOM):
						{
							get_autonom_settings_from_bus();
							break;
						}
						case(I_STRING):
						{
							get_char_from_bus();
							break;
						}
						case(I_FLOAT):
						{
							get_float_from_bus();
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
				switch(currentInstruction)
				{
					case(I_SETTINGS):
					{
						controlSettingsFlag_ = 1;
						break;
					}
					case(I_AUTONOM):
					{
						autonomSettingsFlag_ = 1;
						break;
					}
					case(I_STRING):
					{
						//Add message to buffer
						break;
					}
					case(I_FLOAT):
					{
						//Do smart stuff
						break;
					}
				}
			}
			reset_TWI();
			break;
		}
		// ----------------------------------------------------------------------------- Sensors
		case(S_ADDRESS):
		{
			if(CONTROL == SLAW || CONTROL == ARBIT_SLAW)
			{
				instruction = 1;
			}
			else if(CONTROL == DATA_SLAW)
			{
				if(instruction)
				{
					currentInstruction = get_data();
					instruction = 0;
				}
				else
				{
					switch(currentInstruction)
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
				switch(currentInstruction)
				{
					case(I_SWEEP):
					{
						sweepFlag_ = 1;
						break;
					}
					case(I_STRING):
					{
						write_to_TwiFIFO(message);
						break;
					}
				}
			}
			reset_TWI();
			break;
		}
		// ----------------------------------------------------------------------------- Steer
		case(ST_ADDRESS):
		{
			if(CONTROL == SLAW || CONTROL == ARBIT_SLAW)
			{
				instruction = 1;
			}
			else if(CONTROL == DATA_SLAW)
			{
				if(instruction)
				{
					currentInstruction = get_data();
					instruction = 0;
				}
				else
				{
					switch(currentInstruction)
					{
						case(I_COMMAND):
						{
							get_command_from_bus();
							break;
						}
						case(I_ELEVATION):
						{
							get_elevation_from_bus();
							break;
						}
						case(I_SETTINGS):
						{
							get_control_settings_from_bus();
							break;
						}
						case(I_AUTONOM):
						{
							get_autonom_settings_from_bus();
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
				case(I_COMMAND):
				{
					commandFlag_ = 1;
					break;
				}
				case(I_ELEVATION):
				{
					elevationFlag_ = 1;
					break;
				}
				case(I_SETTINGS):
				{
					controlSettingsFlag_ = 1;
					break;
				}
				case(I_AUTONOM):
				{
					autonomSettingsFlag_ = 1;
					break;
				}
			}
			reset_TWI();
			break;
		}
	}
}