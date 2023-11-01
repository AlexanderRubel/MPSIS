#include <msp430.h>
#include "switch.h"
#include "delay_u.h"

extern unsigned int wdtState;

int dbnc_sw_state(int sw_bit, int port_num) {
    int debounce_cntr = 0; //счетчик зафиксированных нажатий
    int sw_state    = 0;
    //Считаем количество моментов нажатой кнопки за промежуток времени
    int i;
    unsigned int prevState = wdtState;
//    wdtState &= ~0x07;
//    wdtState |= WDTIS_7;
//    WDTCTL = wdtState;
    for (i = 0; i < DBNC_CYCLES; i++) {
        if (is_sw_pressed(sw_bit, port_num)) {
            debounce_cntr += 1;
        }

//        wdtState |= WDTCNTCL;
//        wdtState &= ~WDTHOLD;
//        WDTCTL = wdtState;
//
//        while (wdtState & WDTHOLD);
        __delay_cycles(DBNC_DELAY_US);
    }
//    wdtState = prevState;
//    WDTCTL = wdtState;
    //Если кнопка была в состоянии нажатия более DBNC_BOUND раз,
    // то считаем что кнопка нажата
    if (debounce_cntr > DBNC_BOUND) {
        sw_state = 1;
    } else {
        sw_state = 0;
    }

    return sw_state;
}

// Если кнопка нажата, то возвращает TRUE
int is_sw_pressed (int sw_bit, int port_num) {
    if (port_num == 1) {
        return (P1IN & sw_bit);
    } else if (port_num == 2) {
        return (P2IN & sw_bit);
    }
}









