/*
 * CFile1.c
 *
 * Created: 4/7/2014 11:34:04 AM
 *  Author: samli177
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"
#include "usart.h"
#include "MpuInit.h"
#include "LED.h"


// -- USART Stuff --

// -- Global variables form USART --
uint8_t gTxPayload [255];
uint8_t gTxBuffer [517]; // if there are only control octets in data it needs to be 255 + 7 + 255 = 517
uint8_t gRxBuffer [517];
uint16_t gRxBufferIndex = 0;
uint16_t gInvertNextFlag = 0;

uint8_t gRotation=50, gSpeed=0, gDirection=0;
uint16_t gTurnAngle = 0;
int8_t gTurnDirection = 1;
uint8_t gTurnFlag = 0;
uint8_t gElevationFlag = 0;

float gZ = -120;

// define FIFO for received packets (USART)
MK_FIFO(4096); // use 4 kB
DEFINE_FIFO(gRxFIFO, 4096);


uint8_t USART_getRotation()
{
	uint8_t rotation = gRotation;
	gRotation = 50;
	return  rotation;
}

uint8_t USART_getSpeed()
{
	uint8_t speed = gSpeed;
	gSpeed = 0;
	return speed;
}

uint8_t USART_getDirection()
{
	uint8_t direction = gDirection;
	gDirection = 0;
	return direction;
}

uint8_t USART_get_turn_dir()
{
	return gTurnDirection;
}

uint8_t USART_get_turn_flag()
{
	if(gTurnFlag)
	{
		gTurnFlag = 0;
		return 1;
	}
	return 0;
}

uint16_t USART_get_turn_angle()
{
	return gTurnAngle;
}

float USART_get_z()
{
	return gZ;
}

void USART_init()
{
	//set baud rate
	//typecasting of "int" to byte truncates to the lowest uint8_t
	UBRR0H = (uint8_t) (((F_CPU / 16 / BaudRate ) - 1)>>8);
	UBRR0L = (uint8_t) ((F_CPU / 16 / BaudRate ) - 1) ;

	//enable receiver and transmitter and enable receive interrupt
	UCSR0B = (1<<RXEN0)|(1<<TXEN0|(1<<RXCIE0));

	//Frame format: 8data, no parity, 1 stop bit
	UCSR0C = (1<<UCSZ00 | 1<<UCSZ01);
	
	gRotation = 50;
	gSpeed = 0;
	gDirection = 0;

}

uint8_t USART_check_rx_complete()
{
	return (UCSR0A & (1<<RXC0)); // zero if no data is available to read
}

uint8_t USART_check_tx_ready()
{
	return (UCSR0A & (1<<UDRE0)); // zero if transmit register is not ready to receive new data
}

void USART_write_byte(uint8_t DataByteOut)
{
	while(USART_check_tx_ready() == 0)
	{;;} // wait until ready to transmit NOTE: Can probably be optimized using interrupts
	UDR0 = DataByteOut;
}

uint8_t USART_read_byte()
{
	// NOTE: check if data is available before calling this function. Should probably be implemented w. interrupt.
	return UDR0;

}


// calculate crc for packet
uint16_t USART_crc16(uint8_t tag, uint8_t length)
{
	int count;
	uint16_t data, i;
	unsigned int crc; // maybe use uint16_t here
	
	crc = 0xffff;
	
	if (length == 0)
	return (~crc);
	
	for(count = -2; count < length ; ++count)
	{
		if(count == -2)
		{
			data = tag;
		}else if(count == -1)
		{
			data = length;
		}else
		{
			data = gTxPayload[count];
		}
		
		for (i = 0; i < 8; ++i)
		{
			if ((crc & 0x0001) ^ (data & 0x0001))
			{
				crc = (crc >> 1) ^ POLY;
			}else
			{
				crc >>= 1;
			}
			
			data >>= 1;
		}
	}
	
	crc = ~crc;
	
	data = crc;
	crc = (crc << 8 | (data >> 8 & 0xff));
	
	return (crc);
}

void USART_send_packet(char tag, uint8_t length)
{
	int count, offset, buffersize;
	uint16_t crc;
	
	// put start of packet into transmit buffer
	gTxBuffer[0] = 0x7e; // frame delimiter
	gTxBuffer[1] = tag;
	gTxBuffer[2] = length;
	
	// payload
	
	offset = 3;
	
	for(count = 0; count < length; ++count)
	{
		if(gTxPayload[count] == 0x7e || gTxPayload[count] == 0x7d) // if frame delimiter or escape octet occurs in data
		{
			gTxBuffer[count + offset] = 0x7d; // add the escape octet
			offset++;
			gTxBuffer[count + offset] = (1<<5)^gTxPayload[count]; // invert bit 5 in data
		}else
		{
			gTxBuffer[count + offset] = gTxPayload[count];
		}
	}
	
	crc = USART_crc16(tag, length);
	
	gTxBuffer[count+offset] = (uint8_t)(crc >> 8);
	++count;
	gTxBuffer[count + offset] = (uint8_t)(crc);
	++count;
	
	
	gTxBuffer[count+offset] = 0x7e;
	
	buffersize = count + offset;
	
	for(count = 0; count < buffersize + 1; ++count)
	{
		USART_write_byte(gTxBuffer[count]);
	}
}

void USART_send_message(char msg[])
{
	for(int i = 0; i < strlen(msg); ++i )
	{
		gTxPayload[i] = msg[i];
	}
	
	USART_send_packet('M', strlen(msg));
}


union Union_floatcast
{
	float f;
	char s[sizeof(float)];
};

void USART_send_value(float flo)
{
	union Union_floatcast foo;
	foo.f = flo;
	
	for(int i = 0; i < 4; ++i)
	{
		gTxPayload[i] = foo.s[i]; 
	}
	
	USART_send_packet('V', 4);
	
}

void USART_send_gyro()
{
	union Union_floatcast foo;
	
	foo.f = MPU_get_p();
	for(int i = 0; i < 4; ++i)
	{
		gTxPayload[i] = foo.s[i];
	}
	
	foo.f = MPU_get_r();
	for(int i = 0; i < 4; ++i)
	{
		gTxPayload[i + 4] = foo.s[i];
	}
	
	foo.f = MPU_get_y();
	for(int i = 0; i < 4; ++i)
	{
		gTxPayload[i + 8] = foo.s[i];
	}
	
	USART_send_packet('G', 12);	
}


void USART_send_ready()
{
	USART_send_packet('R', 0);
}

void USART_send_turn_done()
{
	USART_send_packet('T', 0);
}

uint8_t USART_decode_message_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *character = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		//send_string(S_ADRESS, "RxFIFO MESSAGE ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len; // don't know why I can't use *len directly
	
	//NOTE: there has to be a better way of doing this...
	//int ifzero = 0;
	//if(length == 0) ifzero = 1;
	//uint8_t msg[length-1+ifzero];

	for(int i = 0; i < length; ++i)
	{
		if(FifoRead(gRxFIFO, character))
		{
			//send_string(S_ADRESS, "RxFIFO MESSAGE ERROR: DATA MISSING");
			return 1; // error
		}

		//msg[i] = *character;
	}
	
	
	// TODO: send to relevant party... the display for now
	//send_string_fixed_length(S_ADRESS, msg, length);
	
	
	return 0;
}

uint8_t USART_decode_command_rx_fifo()
{
	
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		//send_string(S_ADRESS, "RxFIFO COMMAND ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t direction, rotation, speed;
	
	if(length == 3)
	{
		
			if(FifoRead(gRxFIFO, data))
			{
				//send_string(S_ADRESS, "RxFIFO COMMAND ERROR: DIRECTION MISSING");
				return 1; // error
			}
			direction = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				//send_string(S_ADRESS, "RxFIFO COMMAND ERROR: ROTATION MISSING");
				return 1; // error
			}
			
			rotation = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				//send_string(S_ADRESS, "RxFIFO COMMAND ERROR: SPEED MISSING");
				return 1; // error
			}
			
			speed = *data;
		
		gSpeed = speed;
		gDirection = direction;
		gRotation = rotation;
		
		LED0_TOGGLE;

	}else
	{
		//send_string(S_ADRESS, "RxFIFO COMMAND ERROR: INCORRECT LENGTH");
		return 1;
	}


	return 0;
	
}

uint8_t USART_decode_elevation_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	int length = 0;
	uint8_t direction;
	
	if(FifoRead(gRxFIFO, len))
	{
		return 1; // error
	}
	
	length = *len;
	
	if(length != 1)
	{
		return 1;
	}
	
	if(FifoRead(gRxFIFO, data))
	{
		return 1; // error
	}
	
	direction = *data;
	
	if(direction == 1)
	{
		if(gZ > -150)
		{
			gZ -= 5;
			gElevationFlag = 1;
		}
		
	}else if(direction == 0)
	{
		if(gZ < -80)
		{
			gZ += 5;
			gElevationFlag = 1;
		}
	}
	
	return 0;
	
}

uint8_t USART_decode_turn_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	int length = 0;
	uint16_t angle;
	uint8_t dir;
	
	if(FifoRead(gRxFIFO, len))
	{
		return 1; // error
	}
	
	length = *len;
	
	if(length != 3)
	{
		return 1;
	}
	
	if(FifoRead(gRxFIFO, data))
	{
		return 1;
	}
	angle = 0x0000 | *data;
	
	if(FifoRead(gRxFIFO, data))
	{
		return 1;
	}
	angle = (angle << 8) | *data;
	
	if(FifoRead(gRxFIFO, data))
	{
		return 1;
	}
	dir = *data;
	
	gTurnAngle = angle;
	if(dir == 0)
	{
		gTurnDirection = -1;
	} else {
		gTurnDirection = 1;
	}
	gTurnFlag = 1;
	
	return 0;
}

void USART_decode_rx_fifo()
{
	uint8_t *tag = 0;
	
	while(!(FifoRead(gRxFIFO, tag))) // if the buffer is NOT empty
	{
		switch(*tag){
			case('M'): // if 'tag' is 'M'
			{
				if(USART_decode_message_rx_fifo()) // if decoding failed
				{
					// TODO: flush buffer?
					return;
				}
				
				break;
			}
			case('C'): // 
			{
				if(USART_decode_command_rx_fifo())
				{
					// TODO: flush buffer?
					return;
				}
				break;
			}
			case('E'):
			{
				if(USART_decode_elevation_rx_fifo())
				{
					// TODO: flush buffer?
					return;
				}
				break;
			}
			case('G'):
			{
				// Maybe must do check like the others?
				USART_send_gyro();
				break;
			}
			case('T'):
			{
				if(USART_decode_turn_rx_fifo())
				{
					// TODO: flush buffer?
					return;
				}
				break;
			}
		}
	}
}

uint8_t USART_elevation_flag()
{
	if(gElevationFlag)
	{
		gElevationFlag = 0;
		return 1;
	}
	return 0;
}


void USART_bounce()
{
	for(int i = 0; i < gRxBuffer[1]; i++)
	{
		gTxPayload[i] = gRxBuffer[i+2];
	}
	USART_send_packet(gRxBuffer[0], gRxBuffer[1]);
}


ISR (USART0_RX_vect)
{
	uint8_t data;
	data = UDR0; // read data from buffer TODO: add check for overflow
	
	
	
	if(data == 0x7e)
	{
		if(gRxBufferIndex >= 4 || gRxBufferIndex == gRxBuffer[1] + 4) //TODO: add crc check
		{
			if(gInvertNextFlag)
			{
				data = (1<<5)^data;
				gInvertNextFlag = 0;
			}
			
			
			// Add packet (no crc) to fifo-buffer to cue it for decoding
			for(int i = 0; i < gRxBuffer[1] + 2; ++i)
			{
				if(FifoWrite(gRxFIFO, gRxBuffer[i]))
				{
					//send_string(S_ADRESS,"U_FIFO-full");
				}
			}
		}
		
		gRxBufferIndex = 0; // always reset buffer index when frame delimiter (0x7e) is read
		
	}else if(data == 0x7d)
	{
		gInvertNextFlag = 1;
	}else
	{
		gRxBuffer[gRxBufferIndex] = data;
		++gRxBufferIndex;
	}
	
	
}

// -- END USART stuff --