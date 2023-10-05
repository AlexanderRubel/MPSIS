#include <msp430.h>
#include "switch.h"
#include "led.h"


#define SW      BIT3                    // Switch -> P1.3
#define GREEN   BIT6                    // Green LED -> P1.6

int sw_1_state      = 0;
int sw_2_state      = 0;
int prev_sw_1_state = 0;
volatile unsigned int wdtState = WDTPW + WDTHOLD;
unsigned int doubleDelay = 0;

void main(void) {
    //setup watchdog timer
    WDTCTL = wdtState; // Stop watchdog timer

    //setup switch 1
    P1DIR &= ~SW_1; //set input
    P1REN |=  SW_1; //enable pull up
    P1OUT |=  SW_1; //choose pull up
    P1IES &= ~SW_1; //Select Interrupt on Rising Edge
    P1IE  |=  SW_1; //Enable Interrupt on SW pin

//    UCSCTL5 = DIVA__1;
    //setup switch 2
    P2DIR &= ~SW_2; //set input
    P2REN |=  SW_2; //enable pull up
    P2OUT |=  SW_2; //choose pull up
    P2IES &= ~SW_2; //Select Interrupt on Rising Edge
    P2IE  |=  SW_2; //Enable Interrupt on SW pin

    P8DIR |=  LED_3; //set led to output
    P8OUT &= ~LED_3;



    wdtState |= WDTTMSEL;
    WDTCTL = wdtState; //set interval mode
    wdtState |= WDTSSEL__ACLK;
    WDTCTL = wdtState; //set clock source to ACLK 32768 kHz
    wdtState |= WDTIS_4;
    WDTCTL = wdtState; //set timer interval to 250 ms
//    P1DIR |= GREEN;                     // Set LED pin -> Output
//    P1DIR &= ~SW;                       // Set SW pin -> Input
//    P1REN |= SW;                        // Enable Resistor for SW pin
//    P1OUT |= SW;                        // Select Pull Up for SW pin

    __bis_SR_register(GIE); // Enter LPM4 and Enable CPU Interrupt
//    __bis_SR_register(GIE);

    __no_operation();
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_interval(void) {

    led_toggle(LED_3, LED_OFF);
    wdtState |= WDTHOLD;
    WDTCTL = wdtState;
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
    P1IE  &=  ~SW_1;
    sw_1_state = dbnc_sw_state(SW_1); //get switch state

    //if switch state changed then init led impulse
    if (prev_sw_1_state != sw_1_state) {
        led_toggle(LED_3, LED_ON);
//        __delay_cycles(700000u);
//        led_toggle(LED_3, LED_OFF);
        wdtState |= WDTCNTCL;
        wdtState &= ~WDTHOLD;
        WDTCTL = wdtState;

        while (wdtState & WDTHOLD);

        if (doubleDelay) {
            led_toggle(LED_3, LED_ON);
            wdtState |= WDTCNTCL;
            wdtState &= ~WDTHOLD;
            WDTCTL = wdtState;
        }

//        led_toggle(LED_3, LED_OFF);
    }
    prev_sw_1_state = sw_1_state;

    SFRIE1 |= WDTIE;
    P1IE  |=  SW_1; //Enable Interrupt on SW pin
}

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void) {
    P2IE  &=  ~SW_2;
    sw_2_state = dbnc_sw_state(SW_2);
    if (sw_2_state) {
        wdtState &= ~0x07;
        wdtState |= WDTIS_3;
        WDTCTL = wdtState;
    }
            doubleDelay ^= 1;
    P2IE  |=  SW_2;
}
