/*
 * switch.h
 *
 *  Created on: 10 сент. 2023 г.
 *      Author: Alexander
 */

#ifndef SWITCH_H_
#define SWITCH_H_

#define SW_1           BIT7
#define SW_2           BIT2

#define DBNC_DELAY_US  (2000u) //Delay of 1 cycle of debounce
#define DBNC_CYCLES    (10u) //Debounce cycles
#define DBNC_BOUND     (8u)

//enum usr_state{STATE_RELEASED, STATE_PRESSED};

int dbnc_sw_state(int sw_bit);
int is_sw_pressed (int sw_bit);

#endif /* SWITCH_H_ */
