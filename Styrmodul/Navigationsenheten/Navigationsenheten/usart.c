/*
 * CFile1.c
 *
 * Created: 4/7/2014 11:34:04 AM
 *  Author: samli177
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"
#include "usart.h"
#include "twi.h"


// -- USART Stuff --

// -- Global variables form USART --
uint8_t gTxPayload [255];
uint8_t gTxBuffer [517]; // if there are only control octets in data it needs to be 255 + 7 + 255 = 517
uint8_t gRxBuffer [517];
uint16_t gRxBufferIndex = 0;
uint16_t gInvertNextFlag = 0;
uint16_t gMessageDesitnation = C_ADDRESS;

uint8_t gGangReady = 0;
uint8_t gTurnDone = 0;
uint8_t gClimbDone = 0;

uint8_t gGyroFlag = 0;
float gGyroP = 0;
float gGyroR = 0;
float gGyroY = 0;


// define FIFO for received packets (USART)
MK_FIFO(4096); // use 4 kB
DEFINE_FIFO(gRxFIFO, 4096);


// --------- TODOs -----------

// decide what to do with error messages
// figure out if is is possible to stop "packet inception", flag maybe?

// ---------------------------

union Union_floatcast
{
	float f;
	char s[sizeof(float)];
};

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

}

void USART_set_twi_message_destination(uint16_t address)
{
	gMessageDesitnation = address;
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

void USART_send_sensors()
{
	for(int i = 0; i < 8; i++)
	{
		gTxPayload[i] = TWI_get_sensor(i);
	}

	gTxPayload[8] = TWI_get_servo();
	
	USART_send_packet('S', 9);
}

void USART_send_command()
{
	for(int i = 0; i < 3; i++)
	{
		gTxPayload[i] = TWI_get_command(i);
	}
	
	USART_send_packet('C', 3);
	// clear flag
	
}

void USART_send_elevation()
{
	int temp = TWI_get_elevation();
	gTxPayload[0] = temp;
	USART_send_packet('E', 1);
}

void USART_send_turn(uint16_t angle, uint8_t dir)
{
	uint8_t b1 = (uint8_t) ((angle >> 8) & 0x00FF);
	uint8_t b2 = (uint8_t) (angle & 0x00FF);
	gTxPayload[0] = b1;
	gTxPayload[1] = b2;
	gTxPayload[2] = dir;
	USART_send_packet('T', 3);
}


void USART_send_climb()
{
	USART_send_packet('H', 0);
}


void USART_request_gyro()
{
	USART_send_packet('G', 0);
}

uint8_t USART_decode_message_rx_fifo()
{
	
	uint8_t *len = 0;
	uint8_t *character = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO MESSAGE ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len; // don't know why I can't use *len directly
	
	//NOTE: there has to be a better way of doing this...
	int ifzero = 0;
	if(length == 0) ifzero = 1;
	uint8_t msg[length-1+ifzero];

	for(int i = 0; i < length; ++i)
	{
		if(FifoRead(gRxFIFO, character))
		{
			TWI_send_string(S_ADDRESS, "RxFIFO MESSAGE ERROR: DATA MISSING");
			return 1; // error
		}

		msg[i] = *character;
	}
	
	
	// TODO: make message destination configurable
	TWI_send_string_fixed_length(gMessageDesitnation, msg, length);

	return 0;
}

uint8_t USART_decode_gyro_rx_fifo()
{
	gGyroFlag = 1;
	
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO GYRO ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len; // I don't know why I can't use *len directly... but it took me 4h to figure out that you can't do it....

	if(length == 12)
	{
		union Union_floatcast foo;
		
		for(int j = 0; j < 3; ++j)
		{
			for(int i = 0; i < 4; ++i)
			{
				if(FifoRead(gRxFIFO, data))
				{
					TWI_send_string(S_ADDRESS, "RxFIFO GYRO ERROR: DATA MISSING");
					return 1; // error
				}

				foo.s[i] = *data;
			}
			switch (j)
			{
				case 0:
					gGyroP = foo.f;
					break;
				case 1:
					gGyroR = foo.f;
					break;
				case 2:
					gGyroY = foo.f;
					break;
			}
		}
		
		TWI_send_float(C_ADDRESS, gGyroY);
	} else {
		// Wrong length
		return 1;
	}
	
	return 0;
}

uint8_t USART_decode_command_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t direction, rotation, speed;
	
	if(length == 3)
	{
		
			if(FifoRead(gRxFIFO, data))
			{
				TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: DIRECTION MISSING");
				return 1; // error
			}
			direction = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: ROTATION MISSING");
				return 1; // error
			}
			
			rotation = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: SPEED MISSING");
				return 1; // error
			}
			
			speed = *data;
		
		TWI_send_command(direction, rotation, speed);

	}else
	{
		TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: INCORRECT LENGTH");
		return 1;
	}

	return 0;
	
}

uint8_t USART_DecodeValueFIFO()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	union Union_floatcast foo;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO VALUE ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	
	if(length == 4)
	{
		for(int i = 0; i < 4; ++i)
		{
				if(FifoRead(gRxFIFO, data))
				{
					//send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: DIRECTION MISSING");
					return 1; // error
				}
				
			foo.s[i] = *data;		
		}
	TWI_send_float(gMessageDesitnation, foo.f);
	
	return 0;
	}
	
	return 1;
}

uint8_t USART_DecodeReadyFIFO()
{
	uint8_t *len = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO COMMAND ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	
	if(length == 0)
	{
		gGangReady = 1; // set flag
		return 0;
	}
	
	return 1;
}

uint8_t USART_decode_turn_done_rx_fifo()
{
	uint8_t *len = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO TURN-DONE ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	
	if(length == 0)
	{
		gTurnDone = 1; // set flag
		return 0;
	}
	
	return 1;
}

uint8_t USART_decode_climb_done_rx_fifo()
{
	uint8_t *len = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		TWI_send_string(S_ADDRESS, "RxFIFO CLIMB-DONE ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	
	if(length == 0)
	{
		gClimbDone = 1; // set flag
		return 0;
	}
	
	return 1;
}

uint8_t USART_ready()
{
	if(gGangReady)
	{
		gGangReady = 0;
		return 1;
	}
	return 0;
}

uint8_t USART_GyroFlag()
{
	if(gGyroFlag == 1)
	{
		gGyroFlag = 0;
		return 1;
	}
	return 0;
}

float USART_gyro_get_P()
{
	return gGyroP;
}

float USART_gyro_get_R()
{
	return gGyroR;
}

float USART_gyro_get_Y()
{
	return gGyroY;
}

uint8_t USART_turn_done()
{
	if(gTurnDone)
	{
		gTurnDone = 0;
		return 1;
	}
	return 0;
}

uint8_t USART_climb_done()
{
	if(gClimbDone)
	{
		gClimbDone = 0;
		return 1;
	}
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
			case('V'):
			{
				if(USART_DecodeValueFIFO())
				{
					
					return;
				}
				break;

			}
			case('R'):
			{
				if(USART_DecodeReadyFIFO())
				{
					return;
				}
				break;
			}
			case('G'):
			{
				if(USART_decode_gyro_rx_fifo())
				{
					return;
				}
				break;
			}
			case('T'):
			{
				if(USART_decode_turn_done_rx_fifo())
				{
					return;
				}
				break;
			}
			case('H'):
			{
				if(USART_decode_climb_done_rx_fifo())
				{
					return;
				}
				break;
			}
		}
	}
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
					TWI_send_string(S_ADDRESS,"U_FIFO-full");
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

void USART_send_command_parameters(uint8_t direction, uint8_t rotation, uint8_t speed)
{
	gTxPayload[0] = direction;
	gTxPayload[1] = rotation;
	gTxPayload[2] = speed;
	
	
	USART_send_packet('C', 3);
	// clear flag
	
}

// -- END USART stuff --