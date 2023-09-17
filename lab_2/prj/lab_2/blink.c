#include <msp430.h>
#include "switch.h"
#include "led.h"

#define SW      BIT3                    // Switch -> P1.3
#define GREEN   BIT6                    // Green LED -> P1.6

int sw_1_state      = 0;
int prev_sw_1_state = 0;

void main(void) {
    //setup watchdog timer
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    WDTCTL |= WDTTMSEL; //set interval mode
    WDTCTL |= WDTSSEL_1; //set clock source to ACLK 32768 kHz
    WDTCTL |= WDTIS_5; //set timer interval to 250 ms

    //setup switch 1
    P1DIR &= ~SW_1; //set input
    P1REN |=  SW_1; //enable pull up
    P1OUT |=  SW_1; //choose pull up
    P1IES &= ~SW_1; //Select Interrupt on Rising Edge
    P1IE  |=  SW_1; //Enable Interrupt on SW pin

    //setup switch 2
    P2DIR &= ~SW_2; //set input
    P2REN |=  SW_2; //enable pull up
    P2OUT |=  SW_2; //choose pull up
    P2IES &= ~SW_2; //Select Interrupt on Rising Edge
    P2IE  |=  SW_2; //Enable Interrupt on SW pin

    P8DIR |=  LED_3; //set led to output
//    P1DIR |= GREEN;                     // Set LED pin -> Output
//    P1DIR &= ~SW;                       // Set SW pin -> Input
//    P1REN |= SW;                        // Enable Resistor for SW pin
//    P1OUT |= SW;                        // Select Pull Up for SW pin

    __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 and Enable CPU Interrupt
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_interval(void) {
    led_toggle(LED_3, LED_OFF);
    WDTCTL |= WDTHOLD;
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
    sw_1_state = dbnc_sw_state(SW_1); //get switch state

    //if switch state changed then init led impulse
    if (prev_sw_1_state != sw_1_state) {
        led_toggle(LED_3, LED_ON);
        WDTCTL &= ~WDTHOLD;
    }

    prev_sw_1_state = sw_1_state;
}

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void) {
    WDTCTL ^= 0x001u; //toggle between WDTIS_5 and WDTIS_4
}
