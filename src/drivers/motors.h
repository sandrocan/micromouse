#ifndef MOTORS_H
#define MOTORS_H

#include <xc.h>

void initMotors(void);
void initEncoders(void);
long readLeftEncoderCounts(void);
long readRightEncoderCounts(void);
float readLeftMotorSpeedMps(void);
float readRightMotorSpeedMps(void);
void setMotorStopLatch(int enabled);
void toggleMotorStopLatch(void);
int isMotorStopLatched(void);
void setLeftMotor(float speed);
void setRightMotor(float speed);
void stopMotors(void);

#endif /* MOTORS_H */
