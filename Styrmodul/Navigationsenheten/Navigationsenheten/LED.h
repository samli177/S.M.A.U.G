/* LED.h
 * Defines On, Off and toggle as well as init for the LEDs.
 * Created by Martin
 * 2014-06-12
 */

#ifndef LED_H
#define LED_H

#define LED0_ON PORTA |= (1 << PORTC0)
#define LED0_OFF PORTA &= ~(1 << PORTC0)
#define LED0_TOGGLE PORTA ^= (1 << PORTC0)

#define LED1_ON PORTA |= (1 << PORTC1)
#define LED1_OFF PORTA &= ~(1 << PORTC1)
#define LED1_TOGGLE PORTA ^= (1 << PORTC1)

#define LED2_ON PORTC |= (1 << PORTC6)
#define LED2_OFF PORTC &= ~(1 << PORTC6)
#define LED2_TOGGLE PORTC ^= (1 << PORTC6)

#define LED3_ON PORTC |= (1 << PORTC7)
#define LED3_OFF PORTC &= ~(1 << PORTC7)
#define LED3_TOGGLE PORTC ^= (1 << PORTC7)

#define LED_INIT DDRA |= (1<<PORTA0 | 1<<PORTA1); DDRC |= (1<<PORTC6 | 1<<PORTC7)

#endif