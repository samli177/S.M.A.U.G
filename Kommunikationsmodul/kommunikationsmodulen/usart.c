/*
 * usart.c
 *
 * Created: 4/7/2014 11:34:04 AM
 *  Author: samli177
 */ 

#define F_CPU 18432000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>
#include "fifo.h"
#include "usart.h"
#include "twi.h"
#include "LED.h"


// -- USART Stuff --

// -- Global variables form USART --
uint8_t gTxPayload [255];
uint8_t gTxBuffer [517]; // if there are only control octets in data it needs to be 255 + 7 + 255 = 517
uint8_t gRxBuffer [517];
uint16_t gRxBufferIndex = 0;
uint16_t gInvertNextFlag = 0;


// define FIFO for received packets (USART)
MK_FIFO(4096); // use 4 kB
DEFINE_FIFO(gRxFIFO, 4096);


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
	{;;} // wait until ready to transmit, NOTE: Can probably be optimized using interrupts
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

void USART_send_value(uint8_t msg[])
{
	for(int i = 0; i < 4; ++i )
	{
		gTxPayload[i] = msg[i];
	}
	
	USART_send_packet('V', 4);
}

void USART_send_sensors()
{
	for(int i = 0; i < 8; i++)
	{
		gTxPayload[i*2] = (uint8_t)(TWI_get_sensor(i)>>8);
		gTxPayload[i*2+1] = (uint8_t)(TWI_get_sensor(i));
	}
	
	//gTxPayload[8] = TWI_get_servo();
	
	USART_send_packet('S', 16);
}

void USART_send_autonom(uint8_t settings)
{
	gTxPayload[0] = settings;
	USART_send_packet('A', 1);
}

uint8_t USART_decode_message_rx_fifo()
{
	
	uint8_t *len = 0;
	uint8_t *character = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		USART_send_message( "RxFIFO MESSAGE ERROR: LEN MISSING");
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
			USART_send_message( "RxFIFO MESSAGE ERROR: DATA MISSING");
			return 1; // error
		}

		msg[i] = *character;
	}
	
	
	// TODO: send to relevant party... 
	TWI_send_string_fixed_length(S_ADDRESS, msg, length);
	
	return 0;
}

uint8_t USART_decode_command_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		USART_send_message( "RxFIFO COMMAND ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t direction, rotation, speed;
	
	if(length == 3)
	{
		
			if(FifoRead(gRxFIFO, data))
			{
				USART_send_message( "RxFIFO COMMAND ERROR: DIRECTION MISSING");
				return 1; // error
			}
			direction = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				USART_send_message( "RxFIFO COMMAND ERROR: ROTATION MISSING");
				return 1; // error
			}
			
			rotation = *data;
			
			if(FifoRead(gRxFIFO, data))
			{
				USART_send_message( "RxFIFO COMMAND ERROR: SPEED MISSING");
				return 1; // error
			}
			
			speed = *data;
		LED0_TOGGLE;
		TWI_send_command(direction, rotation, speed);

	}else
	{
		USART_send_message( "RxFIFO COMMAND ERROR: INCORRECT LENGTH");
		return 1;
	}

	return 0;
	
}

uint8_t USART_decode_parameters_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		USART_send_message( "RxFIFO Parameters ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t parameters[255];
	
	if(FifoRead(gRxFIFO, data))
	{
		USART_send_message("RxFIFO Parameters ERROR!");
		return 1; // error
	}
	
	if(*data == 'N' || *data == 'G')
	{
		parameters[0] = *data;
		for(int i = 1; i < length; i++)
		{
			if(FifoRead(gRxFIFO, data))
			{
				USART_send_message("RxFIFO Parameters ERROR!");
				return 1; // error
			}
			parameters[i] = *data;
		}
		
		TWI_send_parameters(ST_ADDRESS, length, parameters);
	}

	return 0;
	
}

uint8_t USART_decode_autonom_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		USART_send_message( "RxFIFO Autonom ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t sett;
	
	if(length == 1)
	{
		
		if(FifoRead(gRxFIFO, data))
		{
			USART_send_message( "RxFIFO Autonom ERROR!");
			return 1; // error
		}
		sett = *data;
		LED2_TOGGLE;
		TWI_send_autonom_settings(ST_ADDRESS,sett);

	}else
	{
		USART_send_message( "RxFIFO Autonom ERROR: INCORRECT LENGTH");
		return 1;
	}

	return 0;
	
}

uint8_t USART_decode_status_rx_fifo()
{
	uint8_t *len = 0;
	uint8_t *data = 0;
	
	if(FifoRead(gRxFIFO, len))
	{
		USART_send_message( "RxFIFO Status ERROR: LEN MISSING");
		return 1; // error
	}
	
	int length = *len;
	uint8_t sett;
	
	if(length == 1)
	{
		
		if(FifoRead(gRxFIFO, data))
		{
			USART_send_message( "RxFIFO Status ERROR!");
			return 1; // error
		}
		sett = *data;
		TWI_send_status_settings(ST_ADDRESS, sett);

	}else
	{
		USART_send_message( "RxFIFO Status ERROR: INCORRECT LENGTH");
		return 1;
	}

	return 0;	
}

uint8_t USART_DecodeElevationRxFIFO()
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
	

	LED1_TOGGLE;

	TWI_send_elevation(direction);

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
					// TODO: flush buffet?
					return;
				}
				
				break;
			}
			case('P'): //
			{
				if(USART_decode_parameters_rx_fifo())
				{
					// TODO: flush buffet?
					return;
				}
				
				break;
			}
			case('A'): //
			{
				if(USART_decode_autonom_rx_fifo())
				{
					// TODO: flush buffet?
					return;
				}
				
				break;
			}
			case('E'):
			{
				if(USART_DecodeElevationRxFIFO())
				{
					// TODO: flush buffet?
					return;
				}
				break;
			}
			case('T'):
			{
				if(USART_decode_status_rx_fifo())
				{
					// TODO: flush buffet?
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
			
			//USART_Bounce();
			
			// Add packet (no crc) to fifo-buffer to cue it for decoding
			for(int i = 0; i < gRxBuffer[1] + 2; ++i)
			{
				if(FifoWrite(gRxFIFO, gRxBuffer[i]))
				{
					USART_send_message("U_FIFO-full");
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