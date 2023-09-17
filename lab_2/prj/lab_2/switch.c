#include <msp430.h>
#include "switch.h"

int dbnc_sw_state(int sw_bit) {
    int debounce_cntr = 0; //������� ��������������� �������
    int sw_state    = 0;
    //������� ���������� �������� ������� ������ �� ���������� �������
    int i;
    for (i = 0; i < DBNC_CYCLES; i++) {
        if (is_sw_pressed(sw_bit)) {
            debounce_cntr += 1;
        }
        __delay_cycles(DBNC_DELAY_US);
    }

    //���� ������ ���� � ��������� ������� ����� DBNC_BOUND ���,
    // �� ������� ��� ������ ������
    if (debounce_cntr > DBNC_BOUND) {
        sw_state = 1;
    } else {
        sw_state = 0;
    }

    return sw_state;
}

// ���� ������ ������, �� ���������� TRUE
int is_sw_pressed (int sw_bit) {
    return (P1IN & sw_bit);
}
