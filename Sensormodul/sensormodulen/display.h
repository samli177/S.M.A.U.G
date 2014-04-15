/* Display.c
 * Include this file to get functions
 * for writing to the display.
 * 
 * Created: 2014-04-02
 * Martin, Per
 */

#ifndef DISPLAY_H
#define DISPLAY_H

void display_init();
void display_char(char c);
void display_text(char text[]);
void display_text_fixed_length(char text[], int length);
void display_text_line(int line, char text[]);
void display_value(float value);
void display_set_pos(int row, int column);
void display_clear();
uint8_t display_read_adress();

#endif