#include "controller.h"

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

// Wall-follow trim gain for raw side sensor error values. The sign is applied
// in the controller update so positive sensor error steers away from the wall.
#define WALL_TRIM_KP (0.08f)
#define WALL_TRIM_MAX_MMPS (300.0f)

// Sensor ranges for the wall-follow mode selection.
#define WALL_SENSOR_WALL_MIN (750U)
#define WALL_SENSOR_WALL_MAX (2200U)
#define WALL_SENSOR_NONE_MAX (150U)
#define WALL_SENSOR_TARGET_SINGLE_WALL (1150.0f)
#define WALL_MODE_SWITCH_CONFIRM_FRAMES (3U)

// In-place 90 degree turns use the same wheel speed on both sides with opposite
// directions and stop once the average encoder travel reaches this count.
#define TURN_SPEED_MMPS (500)
#define TURN_90_TARGET_COUNTS (650)

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
    float trim_output_mmps;
    long turn_start_left_counts;
    long turn_start_right_counts;
    unsigned int wall_follow_enabled;
    unsigned int wall_mode_candidate_count;
    ControllerMode mode;
} DriveControllerState;

static DriveControllerState drive_controller = {
    .left_speed = { LEFT_SPEED_CONTROLLER_KP, LEFT_SPEED_CONTROLLER_KI, 0.0f, 0.0f, 0.0f },
    .right_speed = { RIGHT_SPEED_CONTROLLER_KP, RIGHT_SPEED_CONTROLLER_KI, 0.0f, 0.0f, 0.0f },
    .base_target_mps = 0.0f,
    .trim_output_mmps = 0.0f,
    .turn_start_left_counts = 0L,
    .turn_start_right_counts = 0L,
    .wall_follow_enabled = 1U,
    .wall_mode_candidate_count = 0U,
    .mode = CONTROLLER_MODE_STOP
};

typedef enum {
    WALL_MODE_NONE = 0,
    WALL_MODE_BOTH,
    WALL_MODE_LEFT_ONLY,
    WALL_MODE_RIGHT_ONLY
} WallFollowMode;

static WallFollowMode stable_wall_mode = WALL_MODE_NONE;
static WallFollowMode candidate_wall_mode = WALL_MODE_NONE;

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
    drive_controller.trim_output_mmps = 0.0f;
}

static WallFollowMode detectWallFollowMode(unsigned int left_sensor_value,
                                           unsigned int right_sensor_value)
{
    unsigned int left_wall_present =
        (left_sensor_value >= WALL_SENSOR_WALL_MIN)
        && (left_sensor_value <= WALL_SENSOR_WALL_MAX);
    unsigned int right_wall_present =
        (right_sensor_value >= WALL_SENSOR_WALL_MIN)
        && (right_sensor_value <= WALL_SENSOR_WALL_MAX);
    unsigned int left_wall_missing = (left_sensor_value < WALL_SENSOR_NONE_MAX);
    unsigned int right_wall_missing = (right_sensor_value < WALL_SENSOR_NONE_MAX);

    if (left_wall_present && right_wall_present) {
        return WALL_MODE_BOTH;
    }

    if (left_wall_present && right_wall_missing) {
        return WALL_MODE_LEFT_ONLY;
    }

    if (right_wall_present && left_wall_missing) {
        return WALL_MODE_RIGHT_ONLY;
    }

    return WALL_MODE_NONE;
}

static void updateWallModeLeds(WallFollowMode wall_mode)
{
    switch (wall_mode) {
        case WALL_MODE_BOTH:
            LED_RED = LEDON;
            LED_BLUE = LEDON;
            break;
        case WALL_MODE_LEFT_ONLY:
            LED_RED = LEDOFF;
            LED_BLUE = LEDON;
            break;
        case WALL_MODE_RIGHT_ONLY:
            LED_RED = LEDON;
            LED_BLUE = LEDOFF;
            break;
        default:
            LED_RED = LEDOFF;
            LED_BLUE = LEDOFF;
            break;
    }
}

static WallFollowMode updateStableWallMode(WallFollowMode detected_wall_mode)
{
    if (detected_wall_mode == stable_wall_mode) {
        candidate_wall_mode = detected_wall_mode;
        drive_controller.wall_mode_candidate_count = 0U;
        return stable_wall_mode;
    }

    if (detected_wall_mode != candidate_wall_mode) {
        candidate_wall_mode = detected_wall_mode;
        drive_controller.wall_mode_candidate_count = 1U;
        return stable_wall_mode;
    }

    if (drive_controller.wall_mode_candidate_count < WALL_MODE_SWITCH_CONFIRM_FRAMES) {
        drive_controller.wall_mode_candidate_count++;
    }

    if (drive_controller.wall_mode_candidate_count >= WALL_MODE_SWITCH_CONFIRM_FRAMES) {
        stable_wall_mode = candidate_wall_mode;
        drive_controller.wall_mode_candidate_count = 0U;
    }

    return stable_wall_mode;
}

void initController(void)
{
    drive_controller.base_target_mps = 0.0f;
    drive_controller.left_speed.target_mps = 0.0f;
    drive_controller.right_speed.target_mps = 0.0f;
    drive_controller.turn_start_left_counts = 0L;
    drive_controller.turn_start_right_counts = 0L;
    drive_controller.wall_follow_enabled = 1U;
    drive_controller.wall_mode_candidate_count = 0U;
    drive_controller.mode = CONTROLLER_MODE_STOP;
    stable_wall_mode = WALL_MODE_NONE;
    candidate_wall_mode = WALL_MODE_NONE;

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
    resetWallTrimState();
}

void turnRight90(void)
{
    drive_controller.mode = CONTROLLER_MODE_TURN_RIGHT_90;
    drive_controller.turn_start_left_counts = readLeftEncoderCounts();
    drive_controller.turn_start_right_counts = readRightEncoderCounts();
    resetSpeedPiController(&drive_controller.left_speed);
    resetSpeedPiController(&drive_controller.right_speed);
    resetWallTrimState();
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
    if (!drive_controller.wall_follow_enabled) {
        resetWallTrimState();
    }
}

void updateController(float measured_left_speed_mps,
                      float measured_right_speed_mps,
                      unsigned int left_sensor_value,
                      unsigned int right_sensor_value)
{
    float trim_mmps = 0.0f;
    float wall_error = 0.0f;
    long left_turn_delta;
    long right_turn_delta;
    long average_turn_counts;
    WallFollowMode wall_mode =
        updateStableWallMode(detectWallFollowMode(left_sensor_value, right_sensor_value));

    updateWallModeLeds(wall_mode);

    if (drive_controller.mode == CONTROLLER_MODE_STOP) {
        drive_controller.left_speed.target_mps = 0.0f;
        drive_controller.right_speed.target_mps = 0.0f;
        resetSpeedPiController(&drive_controller.left_speed);
        resetSpeedPiController(&drive_controller.right_speed);
        resetWallTrimState();
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
            resetWallTrimState();
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
        if (wall_mode == WALL_MODE_BOTH) {
            wall_error = (float)left_sensor_value - (float)right_sensor_value;
        } else if (wall_mode == WALL_MODE_LEFT_ONLY) {
            wall_error = (float)left_sensor_value - WALL_SENSOR_TARGET_SINGLE_WALL;
        } else if (wall_mode == WALL_MODE_RIGHT_ONLY) {
            wall_error = WALL_SENSOR_TARGET_SINGLE_WALL - (float)right_sensor_value;
        } else {
            wall_error = 0.0f;
        }

        trim_mmps = -WALL_TRIM_KP * wall_error;
        drive_controller.trim_output_mmps = clampSymmetric(trim_mmps, WALL_TRIM_MAX_MMPS);
    } else {
        resetWallTrimState();
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
    return (drive_controller.wall_follow_enabled != 0U)
        && (drive_controller.mode == CONTROLLER_MODE_STRAIGHT);
}

int isControllerTurning(void)
{
    return (drive_controller.mode == CONTROLLER_MODE_TURN_LEFT_90)
        || (drive_controller.mode == CONTROLLER_MODE_TURN_RIGHT_90);
}
