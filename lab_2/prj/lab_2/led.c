/*
 * led.c
 *
 *  Created on: 10 сент. 2023 г.
 *      Author: Alexander
 */

#include <msp430.h>

#include "led.h"

//Включает или выключает светодиод
void led_toggle(int led_bit, int state) {
    if (state) {
        P8OUT |=  led_bit;
    } else {
        P8OUT &= ~led_bit;
    }
}
