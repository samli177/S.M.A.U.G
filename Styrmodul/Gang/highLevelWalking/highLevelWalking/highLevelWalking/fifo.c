/*
 * fifo.c
 *
 * Created: 4/7/2014 11:36:39 AM
 *  Author: samli177
 */ 


// -- FIFO --

#include <avr/io.h>
#include "fifo.h"

uint8_t FifoDataLength (struct fifo *fifo)
{
	return (fifo->write - fifo->read) & (fifo->size -1);
};

uint8_t FifoWrite (struct fifo *fifo, unsigned char data)
{
	// fifo full : error
	if (FifoDataLength(fifo) == (fifo->size - 1))
	{
		return 1;
	}
	// write data & increment write pointer
	fifo->buffer[fifo->write] = data;
	fifo->write = (fifo->write + 1) & (fifo->size - 1);
	return 0;
};


uint8_t FifoRead (struct fifo *fifo, unsigned char *data)
{
	// fifo empty : error
	if (FifoDataLength(fifo) == 0)
	{
		return 1;
	}
	// read data & increment read pointer
	*data = fifo->buffer[fifo->read];
	fifo->read = (fifo->read + 1) & (fifo->size - 1);
	return 0;
};