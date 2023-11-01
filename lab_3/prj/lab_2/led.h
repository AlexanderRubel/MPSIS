/*
 * led.h
 *
 *  Created on: 10 сент. 2023 г.
 *      Author: Alexander
 */

#ifndef LED_H_
#define LED_H_

#define LED_3          BIT2
#define LED_IMPULSE_US (400000u)
#define LED_ON         (1u)
#define LED_OFF        (0u)

void led_toggle(int led_bit, int state);

#endif /* LED_H_ */
