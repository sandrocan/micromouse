#ifndef TESTS_H
#define TESTS_H

void initButtonLedIndicator(void);
void updateButtonLedIndicator(void);
int isButtonLedDriveEnabled(void);

void initRightMotorPiTest(void);
void updateRightMotorPiTest(float measured_right_speed_mps);
int getRightMotorTargetMmps(void);
int getRightMotorCommandPermille(void);

void initLeftMotorPiTest(void);
void updateLeftMotorPiTest(float measured_left_speed_mps);
int getLeftMotorTargetMmps(void);
int getLeftMotorCommandPermille(void);

void initDualMotorPiDriveTest(void);
void updateDualMotorPiDriveTest(float measured_left_speed_mps, float measured_right_speed_mps);
int getDualMotorTargetMmps(void);
int getDualLeftCommandPermille(void);
int getDualRightCommandPermille(void);

void initWallFollowDriveTest(void);
void updateWallFollowDriveTest(float measured_left_speed_mps,
                               float measured_right_speed_mps,
                               unsigned int left_sensor_value,
                               unsigned int right_sensor_value);
int getWallFollowBaseTargetMmps(void);
int getWallFollowTrimMmps(void);
int getWallFollowLeftCommandPermille(void);
int getWallFollowRightCommandPermille(void);

#endif /* TESTS_H */
