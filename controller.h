#ifndef CONTROLLER_H
#define CONTROLLER_H

void initLeftMotorController(float kp, float ki);
void initRightMotorController(float kp, float ki);
void setLeftMotorSpeedTarget(float target_mps);
void setRightMotorSpeedTarget(float target_mps);
void updateMotorControllers(void);
float getLeftMeasuredSpeedMps(void);
float getRightMeasuredSpeedMps(void);
float getLeftMotorSpeedTarget(void);
float getRightMotorSpeedTarget(void);
float getLeftControlEffort(void);
float getRightControlEffort(void);

#endif /* CONTROLLER_H */
