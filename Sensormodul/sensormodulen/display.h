/* Display.c
 * Include this file to get functions
 * for writing to the display.
 * 
 * Created: 2014-04-02
 * Martin, Per
 */

#ifndef DISPLAY_H
#define DISPLAY_H

void init_display(void);
void print_char(char);
void print_text(char[]);
void print_line(int, char[]);
void print_value(float);
void set_display_pos(int, int);
void clear_display(void);

#endif