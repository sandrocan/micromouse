#include <stdbool.h>

#ifndef CONTROLLER_H
#define CONTROLLER_H

#define WHEEL_SPEED_KP (1.6f)
#define WHEEL_SPEED_KI (18.0f)

// used to adjust the speed of the motors when mouse is not in the middle
#define TRIM_ADJUST_KP (0.005f)

// target distance from wall
#define TARGET_DISTANCE (1130)

// min sensor reading for a wall to be considered next to mouse
#define WALL_MIN_DISTANCE (800)

// The wheel speed controller runs from the 10 ms timer interrupt.
#define WHEEL_SPEED_SAMPLE_TIME_S (0.01f)

// In-place 90 degree turns use equal wheel speeds in opposite directions and
// stop once the average encoder travel reaches this count.
#define TURN_90_TARGET_COUNTS (780)

// Target encoder value to turn 180 degrees
#define TURN_180_TARGET_COUNTS (TURN_90_TARGET_COUNTS * 2)

// wheel speed during the turn
#define TURN_SPEED (0.3f)

// front sensor reading of when to start the turn
#define TURN_DISTANCE (1100)

typedef enum
{
    CONTROLLER_MODE_STOP = 0,
    CONTROLLER_MODE_DRIVE_STRAIGHT,
    CONTROLLER_MODE_TURN_LEFT_90,
    CONTROLLER_MODE_TURN_RIGHT_90,
    CONTROLLER_MODE_TURN_180
} DriveMode;

typedef struct
{
    float target_speed_mps;
    float integral_term_straight;
    float adjusted_speed_mps;
} WheelSpeedController;

typedef struct
{
    WheelSpeedController left_wheel;
    WheelSpeedController right_wheel;
    long left_turn_start_counts;
    long right_turn_start_counts;
    DriveMode mode;
} DriveControllerState;

void stopDriveControl(void);
void initController(void);
void setDriveSpeedMmps(int speed_mmps);
bool isWallLeft(void);
bool isWallRight(void);
bool isWallFront(void);
void turnLeft90(void);
void turnRight90(void);
void turn180(void);
void updateController(void);

int getLeftTargetSpeedMmps(void);
int getDriveTargetSpeedMmps(void);
int getLeftMotorCommandPermille(void);
int getRightMotorCommandPermille(void);
int isTurnInProgress(void);

DriveControllerState *getDriveStatePtr(void);

#endif /* CONTROLLER_H */
