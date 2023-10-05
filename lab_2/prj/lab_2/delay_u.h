#include <msp430.h>

int wait_wdt_delay() {
    __delay_cycles(700000u);
}
