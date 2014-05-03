/* Display.c
 * Include this file to get functions
 * for writing to the display.
 * 
 * Created: 2014-04-02
 * Martin, Per
 */

#ifndef DISPLAY_H
#define DISPLAY_H

/**
 * \brief 
 * Initializes the display.
 * \return void
 */
void display_init();
/**
 * \brief 
 * Displays a single character on the display and increases cursor position one step.
 * If it reaches the end of a line it moves down to the next. Last line connects to the first line.
 * \param c
 * The character to be displayed.
 * \return void
 */
void display_char(char c);
/**
 * \brief 
 * Prints a multi character text on the display. Swaps line if needed.
 * \param text
 * The array of chars to be displayed.
 * \return void
 */
void display_text(char text[]);
/**
 * \brief 
 * Prints a set amount of characters from a text to the display. Swaps line if needed.
 * \param text
 * The array of characters to be displayed.
 * \param length
 * The number of characters to be displayed.
 * \return void
 */
void display_text_fixed_length(char text[], int length);
/**
 * \brief 
 * Prints a text to the display. Starts on 
 * \param line
 * Start line, 0-3.
 * \param text
 * The array of characters to be displayed.
 * \return void
 */
void display_text_line(int line, char text[]);
/**
 * \brief 
 * Prints a numerical value to the display.
 * If the value is not an integer it will display 3 decimals.
 * \param value
 * The value to be displayed.
 * \return void
 */
void display_value(float value);
/**
 * \brief 
 * Sets the cursor position on the display.
 * \param row
 * The row, 0-3.
 * \param column
 * The column, 0-15.
 * \return void
 */
void display_set_pos(int row, int column);
/**
 * \brief 
 * Clears all text from the display and sets cursor position to (0,0).
 * \return void
 */
void display_clear();

#endif