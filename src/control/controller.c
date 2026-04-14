#include "controller.h"

#include "IOconfig.h"
#include "motors.h"
#include "adc.h"
#include "uart.h"
#include "explore.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

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

static float applyLowPass(float previous_value, float new_value, float alpha)
{
    return previous_value + alpha * (new_value - previous_value);
}

static float slewToward(float current_value, float target_value, float max_step)
{
    float delta = target_value - current_value;

    if (delta > max_step)
    {
        return current_value + max_step;
    }

    if (delta < -max_step)
    {
        return current_value - max_step;
    }

    return target_value;
}

static void resetWheelSpeedController(WheelSpeedController *controller)
{
    controller->integral_term_straight = 0.0f;
    controller->filtered_speed_mps = 0.0f;
    controller->filtered_trim_command = 0.0f;
    controller->adjusted_speed_mps = 0.0f;
}

void stopDriveControl(void)
{
    drive_state.mode = CONTROLLER_MODE_STOP;
    drive_state.left_wheel.target_speed_mps = 0.0f;
    drive_state.right_wheel.target_speed_mps = 0.0f;
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    brakeMotors();
    writeUART("STOP driving\n");
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
    return (readLeftSensorValue() > WALL_MIN_DISTANCE);
}

bool isWallRight(void)
{
    return (readRightSensorValue() > WALL_MIN_DISTANCE);
}

bool isWallFront(void)
{
    return (readMidSensorValue() > WALL_MIN_DISTANCE);
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

void turn180(void)
{
    drive_state.left_turn_start_counts = readLeftEncoderCounts();
    drive_state.right_turn_start_counts = readRightEncoderCounts();
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    drive_state.mode = CONTROLLER_MODE_TURN_180;
}

void driveStraight(void)
{
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
    setDriveSpeedMmps(DEFAULT_DRIVE_SPEED_MMPS);
}

static void updateWheelSpeedController(WheelSpeedController *controller, float measured_speed, float distance)
{
    float requested_trim = 0.0f;
    float output;

    controller->filtered_speed_mps = applyLowPass(controller->filtered_speed_mps,
                                                  measured_speed,
                                                  WHEEL_SPEED_FILTER_ALPHA);

    float speed_error = controller->target_speed_mps - controller->filtered_speed_mps;
    controller->integral_term_straight += WHEEL_SPEED_KI * speed_error * WHEEL_SPEED_SAMPLE_TIME_S;

    // Clamp integral to prevent windup
    if (controller->integral_term_straight > 0.5f)
        controller->integral_term_straight = 0.5f;
    if (controller->integral_term_straight < -0.5f)
        controller->integral_term_straight = -0.5f;

    output = WHEEL_SPEED_KP * speed_error + controller->integral_term_straight;

    if (distance > WALL_MIN_DISTANCE)
    {
        // if the sensor reading is valid (a wall is next to sensor)
        // adjust the speed based on distance to wall
        float distance_error = distance - TARGET_DISTANCE;
        // clamp distance error to [-100,100]
        if (distance_error > 100.0f)
            distance_error = 100.0f;
        else if (distance_error < -100.0f)
            distance_error = -100.0f;

        // Trim is P-only, no integrator
        requested_trim = TRIM_ADJUST_KP * distance_error;
    }

    controller->filtered_trim_command = applyLowPass(controller->filtered_trim_command,
                                                     requested_trim,
                                                     WALL_TRIM_FILTER_ALPHA);

    output += controller->filtered_trim_command;
    output = clampMotorCommand(output);
    controller->adjusted_speed_mps = slewToward(controller->adjusted_speed_mps,
                                                output,
                                                DRIVE_COMMAND_SLEW_PER_SAMPLE);
}

static void updateTurnController(void)
{
    long left_counts = readLeftEncoderCounts() - drive_state.left_turn_start_counts;
    long right_counts = readRightEncoderCounts() - drive_state.right_turn_start_counts;

    // Use average of both wheels to be robust against one wheel slipping
    long counts_turned = (labs(left_counts) + labs(right_counts)) / 2;

    if ((counts_turned >= TURN_90_TARGET_COUNTS && (drive_state.mode == CONTROLLER_MODE_TURN_LEFT_90 || drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90)) || (counts_turned >= TURN_180_TARGET_COUNTS && drive_state.mode == CONTROLLER_MODE_TURN_180))
    {
        // Turn complete — resume straight driving
        setLeftMotor(0.0f); // stop motors
        setRightMotor(0.0f);
        resetWheelSpeedController(&drive_state.left_wheel);
        resetWheelSpeedController(&drive_state.right_wheel);
        setLeftMotor(1.0f); // give motors a short forward momentum to reduce vibrations
        setRightMotor(1.0f);
        explore_resetStateDistances();
        driveStraight();
        return;
    }

    if (drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90 || drive_state.mode == CONTROLLER_MODE_TURN_180)
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
    if (cnt >= 10)
    {
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
        // writeUART(uart_buffer);
        snprintf(uart_buffer, sizeof(uart_buffer),
                 "Distance since start: %.3f, Rotations since start: %.3f",
                 getLeftDistanceMeters(),
                 getLeftRotations());
        // writeUART(uart_buffer);
    }

    if (drive_state.mode == CONTROLLER_MODE_STOP)
    {
        // stopDriveControl();
        return;
    }

    // if (dist_mid > TURN_DISTANCE && drive_state.mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    // {
    //     if (isWallLeft() && isWallRight())
    //     {
    //         turn180();
    //     }
    //     else if (isWallLeft())
    //     {
    //         turnRight90();
    //     }
    //     else
    //     {
    //         turnLeft90();
    //     }
    //     return;
    // }

    if (drive_state.mode == CONTROLLER_MODE_TURN_RIGHT_90 ||
        drive_state.mode == CONTROLLER_MODE_TURN_LEFT_90 ||
        drive_state.mode == CONTROLLER_MODE_TURN_180)
    {
        updateTurnController();
        return;
    }

    if (drive_state.mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    {

        updateWheelSpeedController(&drive_state.left_wheel, measured_left_wheel_speed_mps, dist_left);
        updateWheelSpeedController(&drive_state.right_wheel, measured_right_wheel_speed_mps, dist_right);

        static int drive_cnt = 0;
        drive_cnt++;
        if (drive_cnt >= 10)
        {
            drive_cnt = 0;
            char uart_buffer[96];
            snprintf(uart_buffer, sizeof(uart_buffer),
                     "speed left: %.3f, speed right: %.3f\n",
                     drive_state.left_wheel.adjusted_speed_mps,
                     drive_state.right_wheel.adjusted_speed_mps);
            // writeUART(uart_buffer);
        }

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

// returns pointer to the internal drive state, which can be read and modified
DriveControllerState *getDriveStatePtr(void)
{
    return &drive_state;
}
