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
#include "fifo.h"
#include "Navigation.h"


//------------------Internal declarations---------------------------------

/**
 * \brief 
 * Sets the registers for the module to
 * acknowledge the SCL and SDA lines
 * 
 * \return void
 */
static void set_twi_reciever_enable();
/**
 * \brief 
 * To be activated when something unexpected
 * has happened on the bus. Error() determines
 * the correct action to be taken.
 * 
 * \return void
 */
static void Error();
/**
 * \brief 
 * Sets the registers to send a START on the
 * TWI bus.
 * 
 * \return void
 */
static void start_bus();
/**
 * \brief 
 * Sets the registers to send a STOP on the
 * TWI bus.
 * 
 * \return void
 */
static void stop_bus();
/**
 * \brief 
 * Sets the registers to release the TWI bus
 * and enter possible slave receiver mode in case
 * of arbitration on the TWI bus.
 * 
 * \return void
 */
static void clear_int();
/**
 * \brief 
 * Picks out the data from the TWI data register.
 * 
 * \return uint8_t
 * Returns the byte received on the TWI bus.
 */
static uint8_t get_data();
/**
 * \brief 
 * Sets the data to be sent, sends it and waits for the hardware
 * to complete the sending.
 *
 * \param data
 * The byte to be sent
 *
 * \return void
 */
static void send_data_and_wait(uint8_t data);
/**
 * \brief 
 * Waits for the hardware to complete the TWI function.
 * 
 * \return void
 */
static void wait_for_bus();

/**
 * \brief 
 * Resets the assistance variables for the TWI
 * retrieve functions.
 * 
 * \return void
 */
static void stop_twi();
/**
 * \brief 
 * Resets the TWI to be able to receive the 
 * next byte.
 * 
 * \return void
 */
static void reset_TWI();
/**
 * \brief 
 * Retrieves the control settings from the bus and 
 * saves them in the appropriate variable.
 * 
 * \return void
 */
static void get_parameters_from_bus();
/**
 * \brief 
 * Retrieves the autonomous settings from the bus and 
 * saves them in the appropriate variable.
 * 
 * \return void
 */
static void get_autonom_settings_from_bus();
/**
 * \brief 
 * Retrieves a byte from the bus and adds it to 
 * the end of the current message.
 * 
 * \return void
 */
static void get_char_from_bus();
/**
 * \brief 
 * Retrieves the value for the sweep from the 
 * bus.
 * 
 * \return void
 */
static void get_sweep_from_bus();
/**
 * \brief 
 * Retrieves the command from the bus and adds
 * it to the appropriate variable.
 * 
 * \return void
 */
static void get_command_from_bus();
/**
 * \brief 
 * Retrieves the float from the bus and adds
 * it to the appropriate variable.
 * 
 * \return void
 */
static void get_float_from_bus();
/**
 * \brief 
 * Retrieves the sensors from the bus and adds
 * them to a buffer, which is translated to the 
 * appropriate variable in case all sensors are 
 * properly received.
 * 
 * \return void
 */
static void get_sensor_from_bus();
/**
 * \brief 
 * Retrieves the elevation from bus and adds it 
 * to the current elevation value.
 * 
 * \return void
 */
static void get_elevation_from_bus();

static void get_status_settings_from_bus();

//------------------------ Global variables for response functions--------
uint8_t myAdress;
char message[255];
int messageCounter;
uint8_t controlSettings[3]; //KP, KI, KD
uint16_t sensorBuffer[8];
uint16_t sensors[8];
uint8_t servo;
uint8_t sweep;
uint8_t statusSettings;
int sensor;
uint8_t command[3];
int currentCommand;
int messageLength;
uint8_t autonomSettings;
int currentSetting;
int elevation;
uint8_t floatMessage[4];
int floatCounter;
uint8_t gMSB = 0;
uint8_t parametersTag;
uint8_t parameters[255];
uint8_t parameterCount = 0;

//-----------Global variables for the interrupts--------------------------
uint8_t instruction;
int currentInstruction;

//------------Flags for new data, should be set 0 when read 1-------------
uint8_t sensorFlag_ = 0;
uint8_t commandFlag_ = 0;
uint8_t parametersFlag_ = 0;
uint8_t autonomSettingsFlag_ = 0;
uint8_t elevationFlag_ = 0;
uint8_t sweepFlag_ = 0;
uint8_t statusSettingsFlag_ = 0;

// ------------- FIFO for TWI --------------------------------------------
MK_FIFO(1); // use 1 B
DEFINE_FIFO(gTwiFIFO, 1);


//-------------- Initialize ----------------------------------------------------
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

// -------------- Bus functions ------------------------------------------
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

//---------------- Send funcitons for the TWI ----------------------------
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

//-----------------------------Receive functions -------------------------

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
		navigation_fill_buffer();
	}
	else if(gMSB)
	{
		uint16_t tempSensor = (get_data()<<8);
		sensorBuffer[sensor] = tempSensor;
		gMSB = 0;
	}else
	{
		sensorBuffer[sensor] += get_data();
		gMSB = 1;
		sensor += 1;
	}
}

void get_sweep_from_bus()
{
	sweep = get_data();
}

void get_parameters_from_bus()
{
	if(parameterCount == 0)
	{
		parametersTag = get_data();
	} else {
		parameters[parameterCount - 1] = get_data();
	}
	++parameterCount;
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
	elevation = get_data();
}

void get_status_settings_from_bus()
{
	statusSettings = get_data();
}

//----------------Access functions for the TWI----------------------------
uint8_t TWI_get_command(int i)
{
	return command[i];
}

float TWI_get_sensor(int i)
{
	float temp = sensors[i];
	temp = temp/10.0;
	return temp;
}

uint8_t TWI_get_servo()
{
	return servo;
}

uint8_t TWI_get_sweep()
{
	return sweep;
}

uint8_t TWI_get_parameter(uint8_t index)
{
	return parameters[index];
}

uint8_t TWI_get_autonom_settings()
{
	return autonomSettings;
}

uint8_t TWI_get_elevation()
{
	return elevation;
}

uint8_t TWI_get_status_settings()
{
	return statusSettings;
}

//----------------------Flags---------------------------------------------
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

uint8_t TWI_parameters_flag()
{
	if(parametersFlag_)
	{
		parametersFlag_ = 0;
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

uint8_t TWI_status_settings_flag()
{
	if(statusSettingsFlag_)
	{
		statusSettingsFlag_ = 0;
		return 1;
	}
	return 0;
}

//--------------------------FIFO------------------------------------------

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
	uint8_t msg[length-1+ifzero];

	for(int i = 0; i < length; ++i)
	{
		if(FifoRead(gTwiFIFO, character))
		{
			//Wrong length!?
			return 1; // error
		}

		msg[i] = *character;
	}
	
	//Do something with the message here... Send to display for now
	TWI_send_string_fixed_length(S_ADDRESS, msg, length);
	
	return 0;
}

uint8_t write_to_TwiFIFO(char msg[])
{
	if(FifoWrite(gTwiFIFO, (unsigned char)messageLength))
	{
		//Can't add length!?
		return 1;
	}
	
	for(int i = 0; i < messageLength; ++i)
	{
		if(FifoWrite(gTwiFIFO, msg[i]))
		{
			//Can't add chars!?
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
			else if(CONTROL == GENERAL || CONTROL == ARBIT_GENERAL)
			{
				currentInstruction = 255;
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
							get_parameters_from_bus();
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
						parametersFlag_ = 1;
						parameterCount = 0;
						break;
					}
					case(I_AUTONOM):
					{
						autonomSettingsFlag_ = 1;
						break;
					}
					case(I_STRING):
					{
						write_to_TwiFIFO(message);
						break;
					}
					case(I_FLOAT):
					{
						//USART_SendValue(floatMessage);
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
			else if(CONTROL == GENERAL || CONTROL == ARBIT_GENERAL)
			{
				currentInstruction = 255;
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
							get_parameters_from_bus();
							break;
						}
						case(I_AUTONOM):
						{
							get_autonom_settings_from_bus();
							break;
						}
						case(I_STATUS_SETTINGS):
						{
							get_status_settings_from_bus();
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
						parametersFlag_ = 1;
						break;
					}
					case(I_AUTONOM):
					{
						autonomSettingsFlag_ = 1;
						break;
					}
					case(I_STATUS_SETTINGS):
					{
						statusSettingsFlag_ = 1;
						break;
					}
				}
			}
			reset_TWI();
			break;
		}
	}
}