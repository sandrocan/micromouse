#include "controller.h"

#include "IOconfig.h"
#include "motors.h"
#include "adc.h"
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define WHEEL_SPEED_KP (1.6f)
#define WHEEL_SPEED_KI (18.0f)

//used to adjust the speed of the motors when mouse is not in the middle
#define TRIM_ADJUST_KP (0.005f)

//target distance from wall
#define TARGET_DISTANCE (1130)

//max sensor reading for a wall to be considered next to mouse
#define WALL_MIN_DISTANCE (800)

// The wheel speed controller runs from the 10 ms timer interrupt.
#define WHEEL_SPEED_SAMPLE_TIME_S (0.01f)

// In-place 90 degree turns use equal wheel speeds in opposite directions and
// stop once the average encoder travel reaches this count.
#define TURN_90_TARGET_COUNTS (780)

//wheel speed during the turn
#define TURN_SPEED (0.1f)

//front sensor reading of when to start the turn
#define TURN_DISTANCE (1300)

typedef enum
{
    CONTROLLER_MODE_STOP = 0,
    CONTROLLER_MODE_DRIVE_STRAIGHT,
    CONTROLLER_MODE_TURN_LEFT_90,
    CONTROLLER_MODE_TURN_RIGHT_90
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

static DriveControllerState drive_state;

static float clampMotorCommand(float value)
{
    if (value > 1.0f)
    {
        return 1.0f;
    }

    if (value < -1.0f)
    {
        return -1.0f;
    }

    return value;
}

static void resetWheelSpeedController(WheelSpeedController *controller)
{
    controller->integral_term_straight = 0.0f;
    controller->adjusted_speed_mps = 0.0f;
}


static void stopDriveControl(void)
{
    drive_state.mode = CONTROLLER_MODE_STOP;
    drive_state.left_wheel.target_speed_mps = 0.0f;
    drive_state.right_wheel.target_speed_mps = 0.0f;
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    stopMotors();
}

void initController(void)
{
    drive_state.left_wheel.target_speed_mps = 0.0f;
    drive_state.right_wheel.target_speed_mps = 0.0f;
    drive_state.left_turn_start_counts = 0L;
    drive_state.right_turn_start_counts = 0L;
    drive_state.mode = CONTROLLER_MODE_STOP;

    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    stopMotors();
}



void setDriveSpeedMmps(int speed_mmps)
{
    drive_state.mode = (speed_mmps == 0) ? CONTROLLER_MODE_STOP : CONTROLLER_MODE_DRIVE_STRAIGHT;
    
    drive_state.left_wheel.target_speed_mps = ((float)speed_mmps) / 1000.0f;
    drive_state.right_wheel.target_speed_mps = ((float)speed_mmps) / 1000.0f;
}


bool isWallLeft(void)
{
    return (readLeftSensorValue() > 1000);
}

bool isWallRight(void)
{
    return (readRightSensorValue() > 1000);
}


void turnLeft90(void)
{
    drive_state.left_turn_start_counts = readLeftEncoderCounts();
    drive_state.right_turn_start_counts = readRightEncoderCounts();
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    drive_state.mode = CONTROLLER_MODE_TURN_LEFT_90;
}

void turnRight90(void)
{
    drive_state.left_turn_start_counts = readLeftEncoderCounts();
    drive_state.right_turn_start_counts = readRightEncoderCounts();
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    drive_state.mode = CONTROLLER_MODE_TURN_RIGHT_90;
}

static void updateWheelSpeedController(WheelSpeedController *controller, float measured_speed, float distance)
{
    float speed_error = controller->target_speed_mps - measured_speed;    
    controller->integral_term_straight += WHEEL_SPEED_KI * speed_error * WHEEL_SPEED_SAMPLE_TIME_S;
    
    // Clamp integral to prevent windup
    if (controller->integral_term_straight > 0.5f) controller->integral_term_straight = 0.5f;
    if (controller->integral_term_straight < -0.5f) controller->integral_term_straight = -0.5f;

    float output = WHEEL_SPEED_KP * speed_error + controller->integral_term_straight;

    if (distance > WALL_MIN_DISTANCE)
    {
        //if the sensor reading is valid (a wall is next to sensor)
        //adjust the speed based on distance to wall
        float distance_error = distance - TARGET_DISTANCE;
        if (distance_error > 100.0f) distance_error = 100.0f;
        else if (distance_error < -100.0f) distance_error = -100.0f;

        // Trim is P-only, no integrator
        float trim = TRIM_ADJUST_KP * distance_error;
        output += trim;
    }

    controller->adjusted_speed_mps = clampMotorCommand(output);
}


static void updateTurnController(void)
{
    long left_counts  = readLeftEncoderCounts()  - drive_state.left_turn_start_counts;
    long right_counts = readRightEncoderCounts() - drive_state.right_turn_start_counts;

    // Use average of both wheels to be robust against one wheel slipping
    long counts_turned = (labs(left_counts) + labs(right_counts)) / 2;

    if (counts_turned >= TURN_90_TARGET_COUNTS)
    {
        // Turn complete — resume straight driving
        setLeftMotor(0.0f); //stop motors
        setRightMotor(0.0f);
        resetWheelSpeedController(&drive_state.left_wheel);
        resetWheelSpeedController(&drive_state.right_wheel);
        setLeftMotor(1.0f); //give motors a short forward momentum to reduce vibrations
        setRightMotor(1.0f);
        drive_state.mode = CONTROLLER_MODE_DRIVE_STRAIGHT;
        return;
    }

    if (drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90)
    {
        // Left wheel forward, right wheel backward
        setLeftMotor(TURN_SPEED);
        setRightMotor(-TURN_SPEED);
    }
    else
    {
        // Right wheel forward, left wheel backward
        setLeftMotor(-TURN_SPEED);
        setRightMotor(TURN_SPEED);
    }
}

void updateController(void)
{
    float measured_left_wheel_speed_mps = readLeftMotorSpeedMps();
    float measured_right_wheel_speed_mps = readRightMotorSpeedMps();

    unsigned int dist_mid = readMidSensorValue();
    unsigned int dist_left = readLeftSensorValue();
    unsigned int dist_right = readRightSensorValue();

    static int cnt = 0;
    cnt++;
    if (cnt >= 10) {
        cnt = 0;
        char uart_buffer[96];
        snprintf(uart_buffer, sizeof(uart_buffer),
            "left=%.3f right=%.3f target=%d ls=%u rs=%u fs=%u lcmd=%d rcmd=%d\r\n",
            (measured_left_wheel_speed_mps),
            (measured_right_wheel_speed_mps),
            getLeftTargetSpeedMmps(),
            dist_left,
            dist_right,
            dist_mid,
            getLeftMotorCommandPermille(),
            getRightMotorCommandPermille());
        writeUART(uart_buffer);
    }
        

    if (drive_state.mode == CONTROLLER_MODE_STOP)
    {
        stopDriveControl();
        return;
    }

    if (dist_mid > TURN_DISTANCE && drive_state.mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    {
        if (isWallLeft())
        {
            turnRight90();
        }
        else {
            turnLeft90();
        }
        return;
    }

    if (drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90 ||
        drive_state.mode == CONTROLLER_MODE_TURN_LEFT_90)
    {
        updateTurnController();
        return;
    }
    
    if (drive_state.mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    {
        updateWheelSpeedController(&drive_state.left_wheel, measured_left_wheel_speed_mps, dist_left);
        updateWheelSpeedController(&drive_state.right_wheel, measured_right_wheel_speed_mps, dist_right);

        setLeftMotor(drive_state.left_wheel.adjusted_speed_mps); 
        setRightMotor(drive_state.right_wheel.adjusted_speed_mps); 
    }
    
    
}

int getLeftTargetSpeedMmps(void)
{
    return (int)(drive_state.left_wheel.target_speed_mps * 1000.0f);
}

int getLeftMotorCommandPermille(void)
{
    return (int)(drive_state.left_wheel.adjusted_speed_mps * 1000.0f);
}

int getRightMotorCommandPermille(void)
{
    return (int)(drive_state.right_wheel.adjusted_speed_mps * 1000.0f);
}

int isTurnInProgress(void)
{
    return (drive_state.mode == CONTROLLER_MODE_TURN_LEFT_90) ||
           (drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90);
}

