#include <msp430.h> 
#include "switch.h"
#include "led.h"

void main() {
    WDTCTL = WDTPW + WDTHOLD; 

//    P1OUT = 0;
//    P8OUT = 0;
//    P2OUT = 0;

    P1DIR &= ~SW_1; //set input
    P1REN |=  SW_1; //enable pull up
    P1OUT |=  SW_1; //choose pull up

    P8DIR |=  LED_3; //set led to output


    int sw_state       = 0;
    int prev_sw_state  = 1;

    while (1) {
        //Detect switch state change
        if (is_sw_pressed(SW_1) != sw_state) {
            
            //Process switch contact bouncing
            sw_state = dbnc_sw_state(SW_1); //get switch state

            //if switch state changed then init led impulse
            if (prev_sw_state != sw_state) {
                led_toggle(LED_3, LED_ON);
                __delay_cycles(LED_IMPULSE_US);
                led_toggle(LED_3, LED_OFF);
            }

            prev_sw_state = sw_state;
        }
    }

    return;
}
