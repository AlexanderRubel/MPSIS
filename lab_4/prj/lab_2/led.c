/*
 * led.c
 *
 *  Created on: 10 ����. 2023 �.
 *      Author: Alexander
 */

#include <msp430.h>

#include "led.h"

//�������� ��� ��������� ���������
void led_toggle(int led_bit, int state) {
    if (state) {
        P8OUT |=  led_bit;
    } else {
        P8OUT &= ~led_bit;
    }
}
