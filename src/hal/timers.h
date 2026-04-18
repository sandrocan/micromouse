/*  FILE: timers.c
 *  Authors:
 *
 *  Micromouse
 *  Wintersemester 2025
 */

#ifndef TIMERS_H
#define TIMERS_H

#include <xc.h>

void initTimer1ms(float timeinms);
void startTimer1(void);
void initTimer2ms(float timeinms);
void startTimer2(void);
void startGoalMelody(void);

#endif /* TIMERS_H */
