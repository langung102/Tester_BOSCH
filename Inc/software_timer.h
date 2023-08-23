/*
 * software_timer.h
 *
 *  Created on: Aug 18, 2023
 *      Author: minhl
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#include "global.h"

extern int timer0_flag;
extern int timer1_flag;
extern int timer2_flag;
extern int timer3_flag;

void setTimer0(int duration);
void setTimer1(int duration);
void setTimer2(int duration);
void setTimer3(int duration);
void runTimer();

#endif /* INC_SOFTWARE_TIMER_H_ */
