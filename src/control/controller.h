#ifndef CONTROLLER_H
#define CONTROLLER_H

void initController(void);
void driveStraight(int speed_mmps);
void turnLeft90(void);
void turnRight90(void);
void setStraightSpeedMmps(int speed_mmps);
void setWallFollowEnabled(int enabled);
void updateController(float measured_left_speed_mps,
                      float measured_right_speed_mps,
                      unsigned int left_sensor_value,
                      unsigned int right_sensor_value);

int getControllerTargetMmps(void);
int getControllerTrimMmps(void);
int getControllerLeftCommandPermille(void);
int getControllerRightCommandPermille(void);
int isWallFollowTrimActive(void);
int isControllerTurning(void);

#endif /* CONTROLLER_H */
