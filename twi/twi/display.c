/* Display.c
 * Defines functions for the display.
 * 
 * Created: 2014-04-02
 * Martin, Per
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "display.h"

static void toggle_enable();
static int display_busy();
static void print_int(int integer);
static void print_digit(int digit);
static uint8_t display_read_adress();

void display_init()
{
	DDRB = 255;
	DDRD |= 0b11100000;
	
	//TODO: fix this....
	PORTD &= !(1<<PORTD5) | !(1<<PORTD7); //Instruction mode, Write mode
	_delay_ms(31);
	PORTB = 0b00111000; //Function set
	toggle_enable();
	_delay_us(40);
	PORTB = 0b00001111; //Display control
	toggle_enable();
	_delay_us(40);
	PORTB = 0b00000001; //Clear Display
	toggle_enable();
	_delay_ms(2);
	PORTB = 0b00000110; //Set entry mode
	toggle_enable();
}

void toggle_enable()
{
	PORTD |= (1<<PORTD6);
	_delay_ms(10);
	PORTD &= !(1<<PORTD6);
}

void display_char(char c)
{
	while(display_busy()); //Wait for display
	PORTD |= (1<<PORTD5); //Data mode
	PORTB = c;
	toggle_enable();
	
	switch (display_read_adress())
	{
		case 16:
		display_set_pos(1,0);
		break;
		case 80:
		display_set_pos(2,0);
		break;
		case 32:
		display_set_pos(3,0);
		break;
		case 96:
		display_set_pos(0,0);
		break;
	}
}

int display_busy()
{
	PORTB = 0;
	DDRB = 0;
	PORTD &= ~(1<<PORTD5); //Instruction mode
	PORTD |= (1<<PORTD7); //Read mode
	_delay_us(10);
	toggle_enable();
	if(PINB & (1<<PINB7))
	{
		PORTD &= ~(1<<PORTD7);
		DDRB = 255;
		return 1;
	}
	else
	{
		PORTD &= ~(1<<PORTD7);
		DDRB = 255;
		return 0;
	}
}

void display_text(char text[])
{
	for(int i = 0; i < strlen(text); ++i)
	{
		display_char(text[i]);
	}
}

void display_text_fixed_length(char text[], int length)
{
	for(int i = 0; i < length; ++i)
	{
		display_char(text[i]);
	}
}

void display_text_line(int line, char text[])
{
	switch(line)
	{
		case 0:
			display_set_pos(0,0);
			break;
		case 1:
			display_set_pos(1,0);
			break;
		case 2:
			display_set_pos(2,0);
			break;
		case 3:
			display_set_pos(3,0);
			break;
		default:
			break;
	}
	
	for(int i = 0; i < strlen(text); ++i)
	{
		display_char(text[i]);
	}
}

void display_value(float value)
{
	if(value == (int) value)
	{
		// Integer
		if(value < 10)
		{
			print_digit(value);
		} else 
		{
			print_int(value);
		}
	}
}

void print_int(int number)
{
	if(number < 10)
	{
		if(number != 0)
		{
			print_digit(number);
		}
	} else
	{
		int mod = number % 10;
		int left = number - mod;
		print_int(left / 10);
		print_digit(mod);
	}
}

void print_digit(int digit)
{
	switch(digit)
	{
		case 0:
			display_char('0');
			break;
		case 1:
			display_char('1');
			break;
		case 2:
			display_char('2');
			break;
		case 3:
			display_char('3');
			break;
		case 4:
			display_char('4');
			break;
		case 5:
			display_char('5');
			break;
		case 6:
			display_char('6');
			break;
		case 7:
			display_char('7');
			break;
		case 8:
			display_char('8');
			break;
		case 9:
			display_char('9');
			break;
		default:
			display_char('0');
			break;
	}
}

void display_clear()
{
	PORTB = 1<<PORTB0; //Clear Display
	toggle_enable();
	toggle_enable();
}

void display_set_pos(int line, int pos)
{
	PORTD  &= !(1<<PORTD5); //RS = 0 (Instruction mode)
	int data;
	switch(line)
	{
		case 0:
		data = 0b10000000;
		break;
		case 1:
		data = 0b11000000;
		break;
		case 2:
		data = 0b10010000;
		break;
		case 3:
		data = 0b11010000;
		break;
		default:
		data = 0b10000000;
		break;
	}
	data += pos;
	
	PORTB = data; //Set adress
	toggle_enable();
}

uint8_t display_read_adress()
{
	PORTB = 0;
	DDRB = 0;
	PORTD &= ~(1<<PORTD5);
	PORTD |= 1<<PORTD7;
	_delay_us(10);
	toggle_enable();
	
	uint8_t adress = PINB;
	DDRB = 0xFF;
	PORTD |= 1<<PORTD5;
	
	return adress;
}