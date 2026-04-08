#ifndef CONTROLLER_H
#define CONTROLLER_H

void initController(void);
void setDriveSpeedMmps(int speed_mmps);
void turnLeft90(void);
void turnRight90(void);
void updateController(void);

int getLeftTargetSpeedMmps(void);
int getDriveTargetSpeedMmps(void);
int getLeftMotorCommandPermille(void);
int getRightMotorCommandPermille(void);
int isTurnInProgress(void);

#endif /* CONTROLLER_H */
