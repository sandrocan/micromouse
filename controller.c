#include "controller.h"

#include <math.h>
#include "IOconfig.h"
#include "motors.h"

// Independent left/right PI gains. Keeping them separate makes it easy to tune
// each side later without touching the controller structure.
#define LEFT_SPEED_CONTROLLER_KP (1.6f)
#define LEFT_SPEED_CONTROLLER_KI (18.0f)
#define RIGHT_SPEED_CONTROLLER_KP (1.6f)
#define RIGHT_SPEED_CONTROLLER_KI (18.0f)

// The speed controller runs from the 10 ms timer interrupt.
#define SPEED_CONTROLLER_SAMPLE_TIME_S (0.01f)

// Wall-follow trim gains. The trim controller works on a normalized left/right
// balance error so it still behaves well when both side sensors rise or fall
// together while the robot is angled in the corridor.
#define WALL_TRIM_KP (180.0f)
#define WALL_TRIM_KI (35.0f)
#define WALL_TRIM_MAX_MMPS (220.0f)

// Small normalized balance errors are ignored so the robot does not twitch
// while it is already close to centered between the walls.
#define WALL_TRIM_BALANCE_DEADBAND (0.03f)

// Wall following should only run while both side walls are visible.
#define WALL_TRIM_SENSOR_PRESENT_THRESHOLD (100U)

// In-place 90 degree turns use the same wheel speed on both sides with opposite
// directions and stop once the average encoder travel reaches this count.
#define TURN_SPEED_MMPS (500)
#define TURN_90_TARGET_COUNTS (640)

typedef enum {
    CONTROLLER_MODE_STOP = 0,
    CONTROLLER_MODE_STRAIGHT,
    CONTROLLER_MODE_TURN_LEFT_90,
    CONTROLLER_MODE_TURN_RIGHT_90
} ControllerMode;

typedef struct {
    float kp;
    float ki;
    float target_mps;
    float integral;
    float command;
} SpeedPiController;

typedef struct {
    SpeedPiController left_speed;
    SpeedPiController right_speed;
    float base_target_mps;
    float trim_integral_mmps;
    float trim_output_mmps;
    long turn_start_left_counts;
    long turn_start_right_counts;
    unsigned int wall_follow_enabled;
    unsigned int wall_trim_active;
    ControllerMode mode;
} DriveControllerState;

static DriveControllerState drive_controller = {
    .left_speed = { LEFT_SPEED_CONTROLLER_KP, LEFT_SPEED_CONTROLLER_KI, 0.0f, 0.0f, 0.0f },
    .right_speed = { RIGHT_SPEED_CONTROLLER_KP, RIGHT_SPEED_CONTROLLER_KI, 0.0f, 0.0f, 0.0f },
    .base_target_mps = 0.0f,
    .trim_integral_mmps = 0.0f,
    .trim_output_mmps = 0.0f,
    .turn_start_left_counts = 0L,
    .turn_start_right_counts = 0L,
    .wall_follow_enabled = 1U,
    .wall_trim_active = 0U,
    .mode = CONTROLLER_MODE_STOP
};

static float clampUnit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }

    if (value < -1.0f) {
        return -1.0f;
    }

    return value;
}

static float clampSymmetric(float value, float limit)
{
    if (value > limit) {
        return limit;
    }

    if (value < -limit) {
        return -limit;
    }

    return value;
}

static long absoluteLong(long value)
{
    if (value < 0L) {
        return -value;
    }

    return value;
}

static void resetSpeedPiController(SpeedPiController *controller)
{
    controller->integral = 0.0f;
    controller->command = 0.0f;
}

static float updateSpeedPiController(SpeedPiController *controller, float measured_speed_mps)
{
    float error = controller->target_mps - measured_speed_mps;
    float proportional = controller->kp * error;

    controller->integral += controller->ki * error * SPEED_CONTROLLER_SAMPLE_TIME_S;
    controller->command = clampUnit(proportional + controller->integral);

    return controller->command;
}

static void resetWallTrimState(void)
{
    drive_controller.trim_integral_mmps = 0.0f;
    drive_controller.trim_output_mmps = 0.0f;
}

static void deactivateWallTrim(void)
{
    drive_controller.wall_trim_active = 0U;
    resetWallTrimState();
}

static void activateWallTrim(void)
{
    if (!drive_controller.wall_trim_active) {
        resetWallTrimState();
    }

    drive_controller.wall_trim_active = 1U;
}

void initController(void)
{
    drive_controller.base_target_mps = 0.0f;
    drive_controller.left_speed.target_mps = 0.0f;
    drive_controller.right_speed.target_mps = 0.0f;
    drive_controller.turn_start_left_counts = 0L;
    drive_controller.turn_start_right_counts = 0L;
    drive_controller.wall_follow_enabled = 1U;
    drive_controller.wall_trim_active = 1U;
    drive_controller.mode = CONTROLLER_MODE_STOP;

    resetSpeedPiController(&drive_controller.left_speed);
    resetSpeedPiController(&drive_controller.right_speed);
    resetWallTrimState();
    stopMotors();
}

void driveStraight(int speed_mmps)
{
    setStraightSpeedMmps(speed_mmps);
}

void turnLeft90(void)
{
    drive_controller.mode = CONTROLLER_MODE_TURN_LEFT_90;
    drive_controller.turn_start_left_counts = readLeftEncoderCounts();
    drive_controller.turn_start_right_counts = readRightEncoderCounts();
    resetSpeedPiController(&drive_controller.left_speed);
    resetSpeedPiController(&drive_controller.right_speed);
    deactivateWallTrim();
}

void turnRight90(void)
{
    drive_controller.mode = CONTROLLER_MODE_TURN_RIGHT_90;
    drive_controller.turn_start_left_counts = readLeftEncoderCounts();
    drive_controller.turn_start_right_counts = readRightEncoderCounts();
    resetSpeedPiController(&drive_controller.left_speed);
    resetSpeedPiController(&drive_controller.right_speed);
    deactivateWallTrim();
}

void setStraightSpeedMmps(int speed_mmps)
{
    drive_controller.base_target_mps = ((float)speed_mmps) / 1000.0f;
    drive_controller.mode =
        (speed_mmps == 0) ? CONTROLLER_MODE_STOP : CONTROLLER_MODE_STRAIGHT;

    // When the target changes, we keep the current PI state because this is the
    // normal operating path for step changes in desired speed.
}

void setWallFollowEnabled(int enabled)
{
    drive_controller.wall_follow_enabled = (enabled != 0U);
    if (drive_controller.wall_follow_enabled) {
        activateWallTrim();
    } else {
        deactivateWallTrim();
    }
}

void updateController(float measured_left_speed_mps,
                      float measured_right_speed_mps,
                      unsigned int left_sensor_value,
                      unsigned int right_sensor_value)
{
    float trim_mmps = 0.0f;
    long left_turn_delta;
    long right_turn_delta;
    long average_turn_counts;

    if (drive_controller.mode == CONTROLLER_MODE_STOP) {
        drive_controller.left_speed.target_mps = 0.0f;
        drive_controller.right_speed.target_mps = 0.0f;
        resetSpeedPiController(&drive_controller.left_speed);
        resetSpeedPiController(&drive_controller.right_speed);
        deactivateWallTrim();
        stopMotors();
        return;
    }

    if ((drive_controller.mode == CONTROLLER_MODE_TURN_LEFT_90)
        || (drive_controller.mode == CONTROLLER_MODE_TURN_RIGHT_90)) {
        left_turn_delta =
            absoluteLong(readLeftEncoderCounts() - drive_controller.turn_start_left_counts);
        right_turn_delta =
            absoluteLong(readRightEncoderCounts() - drive_controller.turn_start_right_counts);
        average_turn_counts = (left_turn_delta + right_turn_delta) / 2L;

        if (average_turn_counts >= TURN_90_TARGET_COUNTS) {
            drive_controller.mode = CONTROLLER_MODE_STOP;
            drive_controller.base_target_mps = 0.0f;
            drive_controller.left_speed.target_mps = 0.0f;
            drive_controller.right_speed.target_mps = 0.0f;
            resetSpeedPiController(&drive_controller.left_speed);
            resetSpeedPiController(&drive_controller.right_speed);
            deactivateWallTrim();
            stopMotors();
            return;
        }

        if (drive_controller.mode == CONTROLLER_MODE_TURN_LEFT_90) {
            drive_controller.left_speed.target_mps = -((float)TURN_SPEED_MMPS / 1000.0f);
            drive_controller.right_speed.target_mps = ((float)TURN_SPEED_MMPS / 1000.0f);
        } else {
            drive_controller.left_speed.target_mps = ((float)TURN_SPEED_MMPS / 1000.0f);
            drive_controller.right_speed.target_mps = -((float)TURN_SPEED_MMPS / 1000.0f);
        }

        setLeftMotor(updateSpeedPiController(&drive_controller.left_speed, measured_left_speed_mps));
        setRightMotor(updateSpeedPiController(&drive_controller.right_speed, measured_right_speed_mps));
        return;
    }

    if (drive_controller.wall_follow_enabled) {
        if ((left_sensor_value < WALL_TRIM_SENSOR_PRESENT_THRESHOLD)
            || (right_sensor_value < WALL_TRIM_SENSOR_PRESENT_THRESHOLD)) {
            deactivateWallTrim();
        } else {
            activateWallTrim();
        }

        if (drive_controller.wall_trim_active) {
            float sensor_sum = (float)left_sensor_value + (float)right_sensor_value;
            float wall_error;
            float proportional_trim_mmps;

            if (sensor_sum <= 0.0f) {
                wall_error = 0.0f;
            } else {
                // Positive error means the right wall looks farther away than the
                // left wall, so the robot should steer right. Dividing by the
                // sensor sum turns the raw difference into a corridor-balance
                // error instead of a distance-dependent value.
                wall_error =
                    ((float)right_sensor_value - (float)left_sensor_value) / sensor_sum;
            }

            if (fabsf(wall_error) <= WALL_TRIM_BALANCE_DEADBAND) {
                wall_error = 0.0f;
            }

            proportional_trim_mmps = WALL_TRIM_KP * wall_error;
            drive_controller.trim_integral_mmps +=
                WALL_TRIM_KI * wall_error * SPEED_CONTROLLER_SAMPLE_TIME_S;
            trim_mmps = proportional_trim_mmps + drive_controller.trim_integral_mmps;
            drive_controller.trim_output_mmps =
                clampSymmetric(trim_mmps, WALL_TRIM_MAX_MMPS);
        } else {
            resetWallTrimState();
        }
    } else {
        deactivateWallTrim();
    }

    drive_controller.left_speed.target_mps =
        drive_controller.base_target_mps - (drive_controller.trim_output_mmps / 1000.0f);
    drive_controller.right_speed.target_mps =
        drive_controller.base_target_mps + (drive_controller.trim_output_mmps / 1000.0f);

    setLeftMotor(updateSpeedPiController(&drive_controller.left_speed, measured_left_speed_mps));
    setRightMotor(updateSpeedPiController(&drive_controller.right_speed, measured_right_speed_mps));
}

int getControllerTargetMmps(void)
{
    return (int)(drive_controller.base_target_mps * 1000.0f);
}

int getControllerTrimMmps(void)
{
    return (int)(drive_controller.trim_output_mmps);
}

int getControllerLeftCommandPermille(void)
{
    return (int)(drive_controller.left_speed.command * 1000.0f);
}

int getControllerRightCommandPermille(void)
{
    return (int)(drive_controller.right_speed.command * 1000.0f);
}

int isWallFollowTrimActive(void)
{
    return (int)drive_controller.wall_trim_active;
}

int isControllerTurning(void)
{
    return (drive_controller.mode == CONTROLLER_MODE_TURN_LEFT_90)
        || (drive_controller.mode == CONTROLLER_MODE_TURN_RIGHT_90);
}
