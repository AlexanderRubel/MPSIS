#include <msp430.h>
#include "switch.h"

int dbnc_sw_state(int sw_bit) {
    int debounce_cntr = 0; //счетчик зафиксированных нажатий
    int sw_state    = 0;
    //Считаем количество моментов нажатой кнопки за промежуток времени
    int i;
    for (i = 0; i < DBNC_CYCLES; i++) {
        if (is_sw_pressed(sw_bit)) {
            debounce_cntr += 1;
        }
        __delay_cycles(DBNC_DELAY_US);
    }

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
int is_sw_pressed (int sw_bit) {
    return (P1IN & sw_bit);
}
