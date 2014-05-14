/*
 * serialServoControl.c
 *
 * Created: 4/1/2014 10:53:49 AM
 *  Author: samli177
 */ 

#include <avr/io.h>
#include "serialServoControl.h"
#include "fifo.h"
#include "usart.h"
#include "highLevelWalking.h"

#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


// -- Global Variables --
uint8_t gServoParameters[128];
uint8_t gServoTxBuffer[128];

uint8_t gServoRxBuffer[260];
uint8_t gServoRxReadMode = RM_WAIT_FOR_START;
uint8_t gServoLengthCounter = 0;

uint8_t gRxIndex = 0;

uint16_t gServoPos = 0;
uint16_t gServoSpeed = 0;
uint16_t gServoLoad = 0;
uint8_t gServoVoltage = 0;
uint8_t gServoTemperature = 0;


// define FIFO for received packets (USART)
MK_FIFO(1024); // use 1 kB
DEFINE_FIFO(gServoRxFIFO, 1024);


void SERVO_init()
{
	servoTx;
	//set baud rate
	//typecasting of "int" to byte truncates to the lowest uint8_t
	UBRR1H = (uint8_t) (((F_CPU / 16 / ServoBaudRate ) - 1)>>8);
	UBRR1L = (uint8_t) ((F_CPU / 16 / ServoBaudRate ) - 1);
	
	//enable receiver and transmitter and enable interrupts
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	
	//Frame format: 8data, no parity, 1 stop bit
	UCSR1C = (1<<UCSZ10 | 1<<UCSZ11);
	
	servoDDR |= (1<<servoDirPin); //set pin for controlling direction of serial communication w. servo.
	
	// Set torque limit
	SERVO_set_torque_limit(BROADCASTING_ID, 0x3ff); // 100% of max
}

void SERVO_update_EEPROM(uint8_t ID)
{	
	servoTx;
	SERVO_set_return_level(ID, 1);
	_delay_ms(2);
	SERVO_set_return_delay_time(ID, 20);
}

uint8_t servo_check_rx_complete()
{
	return (UCSR1A & (1<<RXC1)); // zero if no data is available to read
}

uint8_t servo_check_tx_ready()
{
	return (UCSR1A & (1<<UDRE1)); // zero if transmit register is not ready to receive new data
}

void servo_write_byte(uint8_t DataByteOut)
{
	while(servo_check_tx_ready() == 0)
	{;;} // wait until ready to transmit NOTE: Can probably be optimized using interrupts
	UDR1 = DataByteOut;
}


void send_servo_packet(uint8_t ID, uint8_t instruction, uint8_t parametersLength)
{
	uint8_t count, checkSum, packetLength;
	
	// construct packet an put in global transmit buffer
	gServoTxBuffer[0] = 0xff;
	gServoTxBuffer[1] = 0xff;
	gServoTxBuffer[2] = ID;
	gServoTxBuffer[3] = parametersLength + 2; // +2 for instruction and checksum
	gServoTxBuffer[4] = instruction;
	
	for (count = 0; count < parametersLength; ++count)
	{
		gServoTxBuffer[count + 5] = gServoParameters[count];
	}
	
	// calculate checksum
	checkSum = 0;
	packetLength = parametersLength + 2 + 4; // 2+ same as above +4 for start bytes, ID and parameterLength

	for(count = 2; count < packetLength -1; ++count) // calculation of checksum starts with ID-byte, not the 0xff start bytes
	{
		checkSum += gServoTxBuffer[count]; // NOTE: make sure overflow truncates to lower byte
	}
	
	gServoTxBuffer[count] = ~checkSum; // checksum is inverted (~ = NOT)
	
	//servoTx; // enable transmit, disable receive
	for(count = 0; count < packetLength; ++count)
	{
		servo_write_byte(gServoTxBuffer[count]);
	}
	//servoRx;
	
}


void SERVO_goto(uint8_t ID, double angle, uint16_t speed)
{
	int16_t goalPosition; 
	servoTx;
	_delay_ms(2);
	angle = 150 + angle * 150/3.1415;
	
	// limit inputs to between 0 and 300 degrees
	if (angle > 300)
	{
		angle = 300;
	} else if (angle < 0)
	{
		angle = 0;
	}
	
	goalPosition = (uint16_t)(angle * 0x3ff / 300); //this will probably truncate correctly...or not....
	
	
	gServoParameters[0] = 0x1E;
	gServoParameters[1] = (uint8_t)goalPosition; //truncates to low byte
	gServoParameters[2] = (uint8_t)(goalPosition>>8); //high byte
	gServoParameters[3] = (uint8_t)speed; 
	gServoParameters[4] = (uint8_t)(speed>>8); 
	
	send_servo_packet(ID, INST_WRITE, 5);	
}

void SERVO_buffer_position(uint8_t ID, double angle, uint16_t speed)
{
	int16_t goalPosition;
	servoTx;
	_delay_ms(2);
	angle = 150 + angle * 150/3.1415;
	
	// limit inputs to between 0 and 300 degrees
	if (angle > 300)
	{
		angle = 300;
	} else if (angle < 0)
	{
		angle = 0;
	}
	
	goalPosition = (uint16_t)(angle * 0x3ff / 300); //this will probably truncate correctly...or not....
	
	
	gServoParameters[0] = 0x1E;
	gServoParameters[1] = (uint8_t)goalPosition; //truncates to low byte
	gServoParameters[2] = (uint8_t)(goalPosition>>8); //high byte
	gServoParameters[3] = (uint8_t)speed;
	gServoParameters[4] = (uint8_t)(speed>>8);
	
	send_servo_packet(ID, INST_REG_WRITE, 5);
}

void SERVO_action()
{
	send_servo_packet(BROADCASTING_ID, INST_ACTION, 0);
}

void SERVO_set_angle_limit(uint8_t ID, double minAngle, double maxAngle)
{
	uint16_t minPosition, maxPosition;
	
	minAngle = minAngle - 0.52359877559;
	maxAngle = maxAngle - 0.52359877559;
	// limit inputs to between 0 and 300 degrees
	if (minAngle > 5.23598776)
	{
		minAngle = 5.23598776;
	} else if (minAngle < 0)
	{
		minAngle = 0;
	}
	
	if (maxAngle > 5.23598776)
	{
		maxAngle = 5.23598776;
	} else if (minAngle < 0)
	{
		maxAngle = 0;
	}
	
	minPosition = (uint16_t)((minAngle/5.23598776)*0x3ff);
	maxPosition = (uint16_t)((maxAngle/5.23598776)*0x3ff);
	
	gServoParameters[0] = 0x06; // address for CW Angle Limit(L)
	gServoParameters[1] = (uint8_t)minPosition; //truncates to low byte
	gServoParameters[2] = (uint8_t)(minPosition>>8); //high byte
	gServoParameters[3] = (uint8_t)maxPosition;
	gServoParameters[4] = (uint8_t)(maxPosition>>8);
	
	send_servo_packet(ID, INST_WRITE, 5);	
}

void SERVO_set_torque_limit(uint8_t ID, uint16_t maxTorque)
{
	gServoParameters[0] = P_TORQUE_LIMIT_L; 
	gServoParameters[1] = (uint8_t)maxTorque; //truncates to low byte
	gServoParameters[2] = (uint8_t)(maxTorque>>8); //high byte
	
	send_servo_packet(ID, INST_WRITE, 3);
}

void SERVO_set_return_level(uint8_t ID, uint8_t level)
{
	gServoParameters[0] = P_RETURN_LEVEL; 
	gServoParameters[1] = level;
	send_servo_packet(ID, INST_WRITE, 2);	
}

void SERVO_set_return_delay_time(uint8_t ID, uint8_t delay)
{
	gServoParameters[0] = P_RETURN_DELAY_TIME;
	gServoParameters[1] = delay;
	send_servo_packet(ID, INST_WRITE, 2);
}

void SERVO_update_data(uint8_t ID)
{
	uint8_t *data = 0;
	uint8_t dataint;
	
	char servoReadMode;
	uint8_t byteCount = 0;
	uint8_t packetLength = 0;
	uint8_t parameters[10]; // TODO: look up max length

	uint8_t checkSum = 0;
	uint8_t inverseCheckSum = 0;
	uint8_t checkSumValid = 0;
	uint8_t exitFlag = 0;
	
	uint8_t error = 0;
	

	servoTx;

	gServoParameters[0] = P_PRESENT_POSITION_L;
	gServoParameters[1] = 8; // read 8 bytes
	send_servo_packet(ID, INST_READ, 2);
	while(servo_check_tx_ready() == 0) // wait until last byte has been transmitted
	_delay_us(20); // wait for stop bits
	servoRx;
	
	_delay_us(250); // receive packet
	
	servoTx;
	servoReadMode = 'W';
	for(int i = 0; i>10; i++)
	{
		parameters[i] = 0;
	}
	while(!(FifoRead(gServoRxFIFO, data)) && !exitFlag)
	{
		dataint = *data;
		switch(servoReadMode){
			case('W'): // wait for start
			{
				if(dataint == 0xff)
				{
					servoReadMode = 'S';
				}
				break;
			}
			case('S'): // check for second start
			{
				if(dataint == 0xff)
				{
					servoReadMode = 'I';
				}else
				{
					servoReadMode = 'W';
				}
				break;
			}
			case('I'):
			{
				
				if(!(*data == ID))
				{
					USART_SendMessage("ServoError: 2");
					while(!(FifoRead(gServoRxFIFO, data))); //flush buffer
				}
				
				servoReadMode = 'L';
				break;
			}
			case('L'):
			{
				packetLength = dataint;
				byteCount = 0;
				servoReadMode = 'E';
				break;
			}
			case('E'):
			{
				// TODO: something...probably...
				error = *data;
				servoReadMode = 'P';
				break;
			}
			case('P'):
			{
				if(byteCount < packetLength-2)
				{
					parameters[byteCount] = dataint;
				}
				
				if(byteCount == packetLength-3){
					servoReadMode = 'C';
				}
	
				++byteCount;
				break;
			}
			case('C'):
			{
				// calculate checksum
				checkSum = ID + packetLength + error;

				for(int count = 0; count < packetLength -2; ++count) // calculation of checksum starts with ID-byte
				{
					checkSum += parameters[count]; // NOTE: make sure overflow truncates to lower byte
				}
				inverseCheckSum = ~checkSum; // don't know why you need to do this...
				if(inverseCheckSum == dataint)
				{
					// correct packet
					checkSumValid = 1; // set flag
				}
				
				
				exitFlag = 1;
				break;
			}
		}
		
	}
	
	// if there is stuff left in buffer
	if(!(FifoRead(gServoRxFIFO, data)))
	{
		USART_SendMessage("ServoError: 3");
		while(!(FifoRead(gServoRxFIFO, data))); //flush buffer
	}
	
	if(checkSumValid)
	{
		uint16_t temp = (uint16_t)parameters[1];
		gServoPos = temp << 8;
		temp = (uint16_t) parameters[0];
		gServoPos = gServoPos + temp;
		
		temp = (uint16_t)parameters[3];
		gServoSpeed = (temp << 8);
		temp = (uint16_t)parameters[2];
		gServoSpeed = gServoSpeed + temp;
		
		temp = (uint16_t)parameters[5];
		gServoLoad = ((temp & 0x01) << 8);
		temp = (uint16_t)parameters[4];
		gServoLoad = gServoLoad + temp;
		//TODO: direction i parameter[5] bit 2
		
		
		gServoVoltage = parameters[6];
		
		gServoTemperature = parameters[7];
			
	}
	
	
}



uint16_t SERVO_get_pos()
{
	return gServoPos;
}

uint16_t SERVO_get_speed()
{
	return gServoSpeed;
}

uint16_t SERVO_get_load()
{
	return gServoLoad;
}

uint8_t SERVO_get_temperature()
{
	return gServoTemperature;
}

uint8_t SERVO_get_voltage()
{
	return gServoVoltage;
}


// -- Interrupts --

ISR (USART1_RX_vect)
{
	uint8_t data;
	data = UDR1;
	if(FifoWrite(gServoRxFIFO, data))
	{
		USART_SendMessage("ServoError: 1");
	}
}


