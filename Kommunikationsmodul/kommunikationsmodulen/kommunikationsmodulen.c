/*
 * kommunikationsmodulen.c
 *
 * Created: 4/3/2014 10:12:45 AM
 *  Author: samli177
 */ 


#define F_CPU 18432000
#define BaudRate 115200

#define POLY 0x8408

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// -- Global variables --
uint8_t gTxPayload [255];
uint8_t gTxBuffer [517]; // if there are only control octets in data it needs to be 255 + 7 + 255 = 517

void init()
{
	DDRA |= (1<<PORTA0|1<<PORTA1); //set status diodes to outputs
	
}

void initSerial()
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

uint8_t CheckRxComplete()
{
	return (UCSR0A & (1<<RXC0)); // zero if no data is available to read
}

uint8_t CheckTxReady()
{
	return (UCSR0A & (1<<UDRE0)); // zero if transmit register is not ready to receive new data
}

void WriteByte(uint8_t DataByteOut)
{
	while(CheckTxReady() == 0)
	{;;} // wait until ready to transmit NOTE: Can probably be optimized using interrupts
	UDR0 = DataByteOut;
}

uint8_t ReadByte()
{
	// NOTE: check if data is available before calling this function. Should probably be implemented w. interrupt.
	return UDR0;

}

uint16_t crc16(uint8_t tag, uint8_t length)
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

void sendPacket(char tag, uint8_t length)
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
	
	crc = crc16(tag, length);
	
	gTxBuffer[count+offset] = (uint8_t)(crc >> 8);
	++count;
	gTxBuffer[count + offset] = (uint8_t)(crc);
	++count;
	
	
	gTxBuffer[count+offset] = 0x7e;
	
	buffersize = count + offset;
	
	for(count = 0; count < buffersize + 1; ++count)
	{
		WriteByte(gTxBuffer[count]);
	}
}


	

void sendPacket2(char tag, uint8_t length)
{
	WriteByte(0x7e);
	WriteByte(tag);
	WriteByte(length);
	for (int i = 0; i < length; ++i)
	{
		WriteByte(gTxPayload[i]);
	}
	WriteByte(0x00);
	WriteByte(0x00);
	WriteByte(0x7e);
	
}

int main(void)
{
	init();
	initSerial();
	sei();

	
    while(1)
    {
		PORTA ^= (1<<PORTA0);
		gTxPayload[0] = 'p';
		gTxPayload[1] = 'a';
		gTxPayload[2] = 'j';
		
		sendPacket('M', 3);
		
		_delay_ms(1000);
    }
}

// -- Interrupts -- 

ISR (USART0_RX_vect)
{
	int8_t dummy;
	dummy = UDR0;
	if (dummy) {}
	PORTA ^= (1<<PORTA1);
	
	
	
}