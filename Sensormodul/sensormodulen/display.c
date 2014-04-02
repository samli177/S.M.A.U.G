/* Display.c
 * Include this file to get functions
 * for writing to the display.
 * 
 * Created: 2014-04-02
 * Martin, Per
 */

#define F_CPU 18300000UL

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "display.h"

void init_display(void)
{
	DDRB = 255;
	DDRD |= 0b11100000;
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

void toggle_enable(void)
{
	PORTD |= (1<<PORTD6);
	_delay_ms(10);
	PORTD &= !(1<<PORTD6);
}

void print_char(char c)
{
	while(display_busy()); //Wait for display
	PORTD |= (1<<PORTD5); //Data mode
	PORTB = c;
	toggle_enable();
}

int display_busy(void)
{
	PORTB = 0;
	DDRB = 0;
	PORTD &= !(1<<PORTD5); //Instruction mode
	PORTD |= (1<<PORTD7); //Read mode
	_delay_us(10);
	toggle_enable();
	if(PORTB & (1<<PORTB7))
	{
		PORTD &= !(1<<PORTD7);
		DDRB = 255;
		return 1;
	}
	else
	{
		PORTD &= !(1<<PORTD7);
		DDRB = 255;
		return 0;
	}
}

void print_text(char text[])
{
	for(int i = 0; i < strlen(text); ++i)
	{
		if(i == 16)
		{
			set_display_pos(1,0);
		} else if(i == 32)
		{
			set_display_pos(2,0);
		} else if(i == 48)
		{
			set_display_pos(3,0);
		}
		print_char(text[i]);
	}
}

void print_line(int line, char text[])
{
	switch(line)
	{
		case 0:
		set_display_pos(0,0);
		break;
		case 1:
		set_display_pos(1,0);
		break;
		case 2:
		set_display_pos(2,0);
		break;
		case 3:
		set_display_pos(3,0);
		break;
		default:
		break;
	}
	
	for(int i = 0; i < 5; ++i)
	{
		print_char(text[i]);
	}
}

void clear_display()
{
	PORTB = 0b00000001; //Clear Display
	toggle_enable();
}

void set_display_pos(int line, int pos)
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