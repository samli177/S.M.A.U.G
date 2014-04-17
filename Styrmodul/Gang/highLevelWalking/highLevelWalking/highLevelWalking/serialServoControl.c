/*
 * serialServoControl.c
 *
 * Created: 4/1/2014 10:53:49 AM
 *  Author: samli177
 */ 

#include <avr/io.h>
#include "serialServoControl.h"

#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h>


// -- Global Variables --
uint8_t gServoParameters[128];
uint8_t gServoTxBuffer[128];

uint8_t gServoRxBuffer[260];
uint8_t gServoRxReadMode = RM_WAIT_FOR_START;
uint8_t gServoLengthCounter = 0;


void initServoSerial()
{
	servoTx;
	//set baud rate
	//typecasting of "int" to byte truncates to the lowest uint8_t
	UBRR1H = (uint8_t) (((F_CPU / 16 / ServoBaudRate ) - 1)>>8);
	UBRR1L = (uint8_t) ((F_CPU / 16 / ServoBaudRate ) - 1) ;
	
	//enable receiver and transmitter and disable interrupts
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	
	//Frame format: 8data, no parity, 1 stop bit
	UCSR1C = (1<<UCSZ10 | 1<<UCSZ11);
	
	servoDDR |= (1<<servoDirPin); //set pin for controlling direction of serial communication w. servo.
	
	// Set torque limit
	servoTorqueLimit(BROADCASTING_ID, 0x200); // 50% of max
}

uint8_t servoCheckRxComplete()
{
	return (UCSR1A & (1<<RXC1)); // zero if no data is available to read
}

uint8_t servoCheckTxReady()
{
	return (UCSR1A & (1<<UDRE1)); // zero if transmit register is not ready to receive new data
}

void servolWriteByte(uint8_t DataByteOut)
{
	while(servoCheckTxReady() == 0)
	{;;} // wait until ready to transmit NOTE: Can probably be optimized using interrupts
	UDR1 = DataByteOut;
}

uint8_t servoReadByte()
{
	// NOTE: check if data is available before calling this function. Should probably be implemented w. interrupt.
	return UDR1;
	
}

void sendServoPacket(uint8_t ID, uint8_t instruction, uint8_t parametersLength)
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
		servolWriteByte(gServoTxBuffer[count]);
	}
	//servoRx;
	
}

void servoGoto(uint8_t ID, double angle, uint16_t speed)
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
	
	sendServoPacket(ID, INST_WRITE, 5);	
}

void servoBufferPosition(uint8_t ID, double angle, uint16_t speed)
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
	
	sendServoPacket(ID, INST_REG_WRITE, 5);
}

void servoAction()
{
	sendServoPacket(BROADCASTING_ID, INST_ACTION, 0);
}

void servoAngleLimit(uint8_t ID, double minAngle, double maxAngle)
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
	
	sendServoPacket(ID, INST_WRITE, 5);	
}

void servoTorqueLimit(uint8_t ID, uint16_t maxTorque)
{
	gServoParameters[0] = P_TORQUE_LIMIT_L; 
	gServoParameters[1] = (uint8_t)maxTorque; //truncates to low byte
	gServoParameters[2] = (uint8_t)(maxTorque>>8); //high byte
	
	sendServoPacket(ID, INST_WRITE, 3);
}

void servoRetrunLevel(uint8_t ID, uint8_t level)
{
	gServoParameters[0] = P_RETURN_LEVEL; // address for CW Angle Limit(L)
	gServoParameters[1] = level;
	sendServoPacket(ID, INST_WRITE, 2);	
}

uint16_t servoGetPosition(uint8_t ID)
{
	gServoParameters[0] = P_PRESENT_POSITION_L;
	gServoParameters[1] = 2; // read 2 bytes
	sendServoPacket(ID, INST_READ, 2);
	
	servoRx;
	
	uint16_t data = 0;
	int pos;
	
	for(int i = 0; i < 8; ++i)
	{
		while ( !(UCSR1A & (1<<RXC1)) );
		data = UDR1;
		if(i == 5) 
		{
			pos = data;
		} else if(i == 6)
		{
			pos = pos + (data << 8);
		}
	}
	
	servoTx;

	return data;
	
}


// -- Interrupts --

ISR (USART1_RX_vect)
{
	uint8_t data;
	data = UDR1; // read data from buffer TODO: add check for overflow
	
	if(data == 0xff)
	{
		if(gServoRxReadMode == RM_WAIT_FOR_START)
		{
			gServoRxReadMode = RM_CHECK_FOR_SECOND_START;
			
		}else if(gServoRxReadMode == RM_CHECK_FOR_SECOND_START)
		{
			gServoRxReadMode = RM_READ_ID;
		}
		
	}else if(gServoRxReadMode == RM_READ_ID)
	{
		gServoRxBuffer[0] = data;
		gServoRxReadMode = RM_READ_LENGTH;
	}else if(gServoRxReadMode == RM_READ_LENGTH)
	{
		gServoRxBuffer[1] = data;
		// TODO: add check for correct length maybe?
		gServoLengthCounter = data;
		gServoRxReadMode = RM_READ_ERROR;	
	}else if(gServoRxReadMode == RM_READ_ERROR)
	{
		gServoRxBuffer[2] = data;
		gServoRxReadMode = RM_READ_PARAMETERS;
		--gServoLengthCounter;
	}else if(gServoRxReadMode == RM_READ_PARAMETERS)
	{
		if(gServoLengthCounter == 2)  
		{
			gServoRxReadMode = RM_READ_CHECKSUM;
		}
	// TODO: figure out how to get the index for gbuffer correct...
	}
}

