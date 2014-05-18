/* LED.h
 * Defines On, Off and toggle as well as init for the LEDs.
 * Created by Martin
 * 2014-06-12
 */

#ifndef LED_H
#define LED_H

#define LED0_ON PORTC |= (1 << PORTC6)
#define LED0_OFF PORTC &= ~(1 << PORTC6)
#define LED0_TOGGLE PORTC ^= (1 << PORTC6)

#define LED1_ON PORTC |= (1 << PORTC7)
#define LED1_OFF PORTC &= ~(1 << PORTC7)
#define LED1_TOGGLE PORTC ^= (1 << PORTC7)

#define LED_INIT DDRC |= (1<<PORTC6|1<<PORTC7)

#endif