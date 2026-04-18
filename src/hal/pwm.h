/*  FILE: pwm.c
 *  Authors:
 *
 *  Micromouse
 *  Wintersemester 2025
 */

#ifndef PWM_H
#define PWM_H

#include <xc.h>

#define PWM_MAX (40000L)

void setupPWM();
void setDCMotorLeft(float dc);
void setDCMotorRight(float dc);
void setDCLEDGreen(float dc);
void setDCExtra(float dc);
void activateBuzzer(float dc);

#endif /* PWM_H */