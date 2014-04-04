/*
 * serialServoControl.c
 *
 * Created: 4/1/2014 10:53:49 AM
 *  Author: samli177
 */ 
#define F_CPU 16000000
#define BaudRate 1000000

#define servoDDR DDRD
#define servoDirPort PORTD
#define servoDirPin PD4


#define servoRx servoDirPort |= (1<<servoDirPin)
#define servoTx servoDirPort &= ~(1<<servoDirPin)

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>



//--- Control Table Address ---
//EEPROM AREA
#define P_MODEL_NUMBER_L 0
#define P_MODOEL_NUMBER_H 1
#define P_VERSION 2
#define P_ID 3
#define P_BAUD_RATE 4
#define P_RETURN_DELAY_TIME 5
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_SYSTEM_DATA2 10
#define P_LIMIT_TEMPERATURE 11
#define P_DOWN_LIMIT_VOLTAGE 12
#define P_UP_LIMIT_VOLTAGE 13
#define P_MAX_TORQUE_L 14
#define P_MAX_TORQUE_H 15
#define P_RETURN_LEVEL 16
#define P_ALARM_LED 17
#define P_ALARM_SHUTDOWN 18
#define P_OPERATING_MODE 19
#define P_DOWN_CALIBRATION_L 20
#define P_DOWN_CALIBRATION_H 21
#define P_UP_CALIBRATION_L 22
#define P_UP_CALIBRATION_H 23

//RAM AREA

#define P_TORQUE_ENABLE (24)
#define P_LED (25)
#define P_CW_COMPLIANCE_MARGIN (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE (28)
#define P_CCW_COMPLIANCE_SLOPE (29)
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
#define P_TORQUE_LIMIT_L (34)
#define P_TORQUE_LIMIT_H (35)
#define P_PRESENT_POSITION_L (36)
#define P_PRESENT_POSITION_H (37)
#define P_PRESENT_SPEED_L (38)
#define P_PRESENT_SPEED_H (39)
#define P_PRESENT_LOAD_L (40)
#define P_PRESENT_LOAD_H (41)
#define P_PRESENT_VOLTAGE (42)
#define P_PRESENT_TEMPERATURE (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME (45)
#define P_MOVING (46)
#define P_LOCK (47)
#define P_PUNCH_L (48)
#define P_PUNCH_H (49)

//--- Instruction ---
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_DIGITAL_RESET 0x07
#define INST_SYSTEM_READ 0x0C
#define INST_SYSTEM_WRITE 0x0D
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_REG_WRITE 0x84

#define BROADCASTING_ID 0xfe


// -- Global Variables --
uint8_t gServoParameters[128];
uint8_t gServoTxBuffer[128];


void initServoSerial()
{
	//set baud rate
	//typecasting of "int" to byte truncates to the lowest uint8_t
	UBRR1H = (uint8_t) (((F_CPU / 16 / BaudRate ) - 1)>>8);
	UBRR1L = (uint8_t) ((F_CPU / 16 / BaudRate ) - 1) ;
	
	//enable receiver and transmitter
	//TODO:: enbable interrupts?
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	
	//Frame format: 8data, no parity, 1 stop bit
	UCSR1C = (1<<UCSZ10 | 1<<UCSZ11);
	
	servoDDR |= (1<<servoDirPin); //set pin for controlling direction of serial communication w. servo.
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
	
	angle = angle - 0.52359877559;
	
	// limit inputs to between 0 and 300 degrees
	if (angle > 5.23598776)
	{
		angle = 5.23598776;
	} else if (angle < 0)
	{
		angle = 0;
	}
	
	goalPosition = (uint16_t)((angle/5.23598776)*0x3ff); //this will probably truncate correctly...or not....
	
	
	gServoParameters[0] = 0x1E;
	gServoParameters[1] = (uint8_t)goalPosition; //truncates to low byte
	gServoParameters[2] = (uint8_t)(goalPosition>>8); //high byte
	gServoParameters[3] = (uint8_t)speed; 
	gServoParameters[4] = (uint8_t)(speed>>8); 
	
	sendServoPacket(ID, INST_WRITE, 5);	
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

int main(void)
{
	initServoSerial();
	
	servoTx;
	
	
	//set servo angle
	gServoParameters[0] = 0x1E;
	gServoParameters[1] = 0x00;
	gServoParameters[2] = 0x02;
	gServoParameters[3] = 0x00;
	gServoParameters[4] = 0x02;
	
	
	
	while(1)
	{
		_delay_ms(500);
		servoGoto(16, 3.14/2 ,0x200);
		_delay_ms(500);
		servoGoto(16, 0 ,0x200);
		
	}
	
	
}
