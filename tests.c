#include "tests.h"

#include "IOconfig.h"
#include "motors.h"
#include "selfdestruct.h"

// Entry point 1: proportional gain
#define RIGHT_PI_TEST_KP (1.6f)

// Entry point 2: integral gain
#define RIGHT_PI_TEST_KI (18.0f)

// Entry point 3: absolute target speeds in mm/s for the button test sequence
#define RIGHT_PI_TEST_TARGET_STATE_1_MMPS (600)
#define RIGHT_PI_TEST_TARGET_STATE_2_MMPS (350)
#define RIGHT_PI_TEST_TARGET_STATE_3_MMPS (950)

// Entry point 4: controller sample time in seconds
#define RIGHT_PI_TEST_SAMPLE_TIME_S (0.01f)

// Entry point 5: base speed for the wall-follow drive test in mm/s
#define WALL_FOLLOW_BASE_TARGET_MMPS (500)

// Entry point 6: wall-follow trim controller gains
#define WALL_FOLLOW_TRIM_KP (0.08f)
#define WALL_FOLLOW_TRIM_KI (0.0f)

// Entry point 7: limit for wall-follow trim in mm/s
#define WALL_FOLLOW_MAX_TRIM_MMPS (60)

// Entry point 8: disable wall trim if a side sensor changes too abruptly
#define WALL_FOLLOW_SENSOR_DELTA_DISABLE_THRESHOLD (60)

typedef struct {
    float target_mps;
    float integral;
    float command;
    unsigned int state;
} RightMotorPiTestState;

static RightMotorPiTestState right_motor_pi_test = {0};

typedef struct {
    float target_mps;
    float integral;
    float command;
    unsigned int state;
} LeftMotorPiTestState;

static LeftMotorPiTestState left_motor_pi_test = {0};

typedef struct {
    float base_target_mps;
    float left_target_mps;
    float right_target_mps;
    float left_integral;
    float right_integral;
    float left_command;
    float right_command;
    unsigned int enabled;
} DualMotorPiDriveTestState;

static DualMotorPiDriveTestState dual_motor_pi_drive_test = {0};

typedef struct {
    float base_target_mps;
    float left_target_mps;
    float right_target_mps;
    float left_integral;
    float right_integral;
    float trim_integral_mmps;
    float trim_output_mmps;
    float left_command;
    float right_command;
    unsigned int previous_left_sensor_value;
    unsigned int previous_right_sensor_value;
    unsigned int trim_enabled;
    unsigned int enabled;
} WallFollowDriveTestState;

static WallFollowDriveTestState wall_follow_drive_test = {0};
static unsigned int button_led_indicator_state = 0;

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

static void applyButtonLedIndicatorState(void)
{
    switch (button_led_indicator_state) {
        case 1:
            LED_RED = LEDON;
            LED_BLUE = LEDOFF;
            break;
        case 2:
            LED_RED = LEDOFF;
            LED_BLUE = LEDON;
            break;
        case 3:
            LED_RED = LEDON;
            LED_BLUE = LEDON;
            break;
        default:
            LED_RED = LEDOFF;
            LED_BLUE = LEDOFF;
            break;
    }
}

void initButtonLedIndicator(void)
{
    button_led_indicator_state = 0U;
    applyButtonLedIndicatorState();
    clearSelfDestructPressFlag();
}

void updateButtonLedIndicator(void)
{
    if (!selfDestructWasPressed()) {
        return;
    }

    clearSelfDestructPressFlag();
    button_led_indicator_state++;

    if (button_led_indicator_state > 3U) {
        button_led_indicator_state = 0U;
    }

    applyButtonLedIndicatorState();
}

int isButtonLedDriveEnabled(void)
{
    return (button_led_indicator_state != 0U);
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

static void resetRightMotorPiTest(void)
{
    right_motor_pi_test.integral = 0.0f;
    right_motor_pi_test.command = 0.0f;
}

static void applyRightMotorPiLedState(void)
{
    switch (right_motor_pi_test.state) {
        case 1:
            LED_RED = LEDON;
            LED_BLUE = LEDOFF;
            break;
        case 2:
            LED_RED = LEDOFF;
            LED_BLUE = LEDON;
            break;
        case 3:
            LED_RED = LEDON;
            LED_BLUE = LEDON;
            break;
        default:
            LED_RED = LEDOFF;
            LED_BLUE = LEDOFF;
            break;
    }
}

static void applyRightMotorPiTargetState(void)
{
    switch (right_motor_pi_test.state) {
        case 1:
            right_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_1_MMPS) / 1000.0f;
            break;
        case 2:
            right_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_2_MMPS) / 1000.0f;
            break;
        case 3:
            right_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_3_MMPS) / 1000.0f;
            break;
        default:
            right_motor_pi_test.target_mps = 0.0f;
            resetRightMotorPiTest();
            setRightMotor(0.0f);
            break;
    }
}

static void updateRightMotorPiButtonState(void)
{
    if (!selfDestructWasPressed()) {
        return;
    }

    clearSelfDestructPressFlag();

    right_motor_pi_test.state++;
    if (right_motor_pi_test.state > 3U) {
        right_motor_pi_test.state = 0U;
    }

    applyRightMotorPiTargetState();
    applyRightMotorPiLedState();
}

void initRightMotorPiTest(void)
{
    right_motor_pi_test.target_mps = 0.0f;
    right_motor_pi_test.state = 0;
    resetRightMotorPiTest();
    applyRightMotorPiTargetState();
    applyRightMotorPiLedState();
    clearSelfDestructPressFlag();
    setLeftMotor(0.0f);
    setRightMotor(0.0f);
}

void updateRightMotorPiTest(float measured_right_speed_mps)
{
    float error;
    float proportional;

    updateRightMotorPiButtonState();

    if (right_motor_pi_test.target_mps == 0.0f) {
        resetRightMotorPiTest();
        setLeftMotor(0.0f);
        setRightMotor(0.0f);
        return;
    }

    error = right_motor_pi_test.target_mps - measured_right_speed_mps;
    proportional = RIGHT_PI_TEST_KP * error;
    right_motor_pi_test.integral += RIGHT_PI_TEST_KI * error * RIGHT_PI_TEST_SAMPLE_TIME_S;

    right_motor_pi_test.command = clampUnit(proportional + right_motor_pi_test.integral);
    setLeftMotor(0.0f);
    setRightMotor(right_motor_pi_test.command);
}

int getRightMotorTargetMmps(void)
{
    return (int)(right_motor_pi_test.target_mps * 1000.0f);
}

int getRightMotorCommandPermille(void)
{
    return (int)(right_motor_pi_test.command * 1000.0f);
}

static void resetLeftMotorPiTest(void)
{
    left_motor_pi_test.integral = 0.0f;
    left_motor_pi_test.command = 0.0f;
}

static void applyLeftMotorPiLedState(void)
{
    switch (left_motor_pi_test.state) {
        case 1:
            LED_RED = LEDON;
            LED_BLUE = LEDOFF;
            break;
        case 2:
            LED_RED = LEDOFF;
            LED_BLUE = LEDON;
            break;
        case 3:
            LED_RED = LEDON;
            LED_BLUE = LEDON;
            break;
        default:
            LED_RED = LEDOFF;
            LED_BLUE = LEDOFF;
            break;
    }
}

static void applyLeftMotorPiTargetState(void)
{
    switch (left_motor_pi_test.state) {
        case 1:
            left_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_1_MMPS) / 1000.0f;
            break;
        case 2:
            left_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_2_MMPS) / 1000.0f;
            break;
        case 3:
            left_motor_pi_test.target_mps = ((float)RIGHT_PI_TEST_TARGET_STATE_3_MMPS) / 1000.0f;
            break;
        default:
            left_motor_pi_test.target_mps = 0.0f;
            resetLeftMotorPiTest();
            setLeftMotor(0.0f);
            break;
    }
}

static void updateLeftMotorPiButtonState(void)
{
    if (!selfDestructWasPressed()) {
        return;
    }

    clearSelfDestructPressFlag();

    left_motor_pi_test.state++;
    if (left_motor_pi_test.state > 3U) {
        left_motor_pi_test.state = 0U;
    }

    applyLeftMotorPiTargetState();
    applyLeftMotorPiLedState();
}

void initLeftMotorPiTest(void)
{
    left_motor_pi_test.target_mps = 0.0f;
    left_motor_pi_test.state = 0;
    resetLeftMotorPiTest();
    applyLeftMotorPiTargetState();
    applyLeftMotorPiLedState();
    clearSelfDestructPressFlag();
    setLeftMotor(0.0f);
    setRightMotor(0.0f);
}

void updateLeftMotorPiTest(float measured_left_speed_mps)
{
    float error;
    float proportional;

    updateLeftMotorPiButtonState();

    if (left_motor_pi_test.target_mps == 0.0f) {
        resetLeftMotorPiTest();
        setLeftMotor(0.0f);
        setRightMotor(0.0f);
        return;
    }

    error = left_motor_pi_test.target_mps - measured_left_speed_mps;
    proportional = RIGHT_PI_TEST_KP * error;
    left_motor_pi_test.integral += RIGHT_PI_TEST_KI * error * RIGHT_PI_TEST_SAMPLE_TIME_S;

    left_motor_pi_test.command = clampUnit(proportional + left_motor_pi_test.integral);
    setLeftMotor(left_motor_pi_test.command);
    setRightMotor(0.0f);
}

int getLeftMotorTargetMmps(void)
{
    return (int)(left_motor_pi_test.target_mps * 1000.0f);
}

int getLeftMotorCommandPermille(void)
{
    return (int)(left_motor_pi_test.command * 1000.0f);
}

static void resetDualMotorPiDriveTest(void)
{
    dual_motor_pi_drive_test.left_integral = 0.0f;
    dual_motor_pi_drive_test.right_integral = 0.0f;
    dual_motor_pi_drive_test.left_command = 0.0f;
    dual_motor_pi_drive_test.right_command = 0.0f;
}

static void applyDualMotorPiDriveLedState(void)
{
    if (dual_motor_pi_drive_test.enabled) {
        LED_RED = LEDON;
        LED_BLUE = LEDON;
    } else {
        LED_RED = LEDOFF;
        LED_BLUE = LEDOFF;
    }
}

static void updateDualMotorPiDriveButtonState(void)
{
    if (!selfDestructWasPressed()) {
        return;
    }

    clearSelfDestructPressFlag();
    dual_motor_pi_drive_test.enabled = !dual_motor_pi_drive_test.enabled;

    if (dual_motor_pi_drive_test.enabled) {
        dual_motor_pi_drive_test.base_target_mps = 0.50f;
        dual_motor_pi_drive_test.left_target_mps = 0.50f;
        dual_motor_pi_drive_test.right_target_mps = 0.50f;
    } else {
        dual_motor_pi_drive_test.base_target_mps = 0.0f;
        dual_motor_pi_drive_test.left_target_mps = 0.0f;
        dual_motor_pi_drive_test.right_target_mps = 0.0f;
        resetDualMotorPiDriveTest();
        stopMotors();
    }

    applyDualMotorPiDriveLedState();
}

void initDualMotorPiDriveTest(void)
{
    dual_motor_pi_drive_test.base_target_mps = 0.0f;
    dual_motor_pi_drive_test.left_target_mps = 0.0f;
    dual_motor_pi_drive_test.right_target_mps = 0.0f;
    dual_motor_pi_drive_test.enabled = 0;
    resetDualMotorPiDriveTest();
    applyDualMotorPiDriveLedState();
    clearSelfDestructPressFlag();
    stopMotors();
}

void updateDualMotorPiDriveTest(float measured_left_speed_mps, float measured_right_speed_mps)
{
    float left_error;
    float right_error;
    float left_proportional;
    float right_proportional;

    updateDualMotorPiDriveButtonState();

    if (!dual_motor_pi_drive_test.enabled) {
        resetDualMotorPiDriveTest();
        stopMotors();
        return;
    }

    left_error = dual_motor_pi_drive_test.left_target_mps - measured_left_speed_mps;
    right_error = dual_motor_pi_drive_test.right_target_mps - measured_right_speed_mps;
    left_proportional = RIGHT_PI_TEST_KP * left_error;
    right_proportional = RIGHT_PI_TEST_KP * right_error;

    dual_motor_pi_drive_test.left_integral += RIGHT_PI_TEST_KI * left_error * RIGHT_PI_TEST_SAMPLE_TIME_S;
    dual_motor_pi_drive_test.right_integral += RIGHT_PI_TEST_KI * right_error * RIGHT_PI_TEST_SAMPLE_TIME_S;

    dual_motor_pi_drive_test.left_command = clampUnit(left_proportional + dual_motor_pi_drive_test.left_integral);
    dual_motor_pi_drive_test.right_command = clampUnit(right_proportional + dual_motor_pi_drive_test.right_integral);

    setLeftMotor(dual_motor_pi_drive_test.left_command);
    setRightMotor(dual_motor_pi_drive_test.right_command);
}

int getDualMotorTargetMmps(void)
{
    return (int)(dual_motor_pi_drive_test.base_target_mps * 1000.0f);
}

int getDualLeftCommandPermille(void)
{
    return (int)(dual_motor_pi_drive_test.left_command * 1000.0f);
}

int getDualRightCommandPermille(void)
{
    return (int)(dual_motor_pi_drive_test.right_command * 1000.0f);
}

static void resetWallFollowDriveTest(void)
{
    wall_follow_drive_test.left_integral = 0.0f;
    wall_follow_drive_test.right_integral = 0.0f;
    wall_follow_drive_test.trim_integral_mmps = 0.0f;
    wall_follow_drive_test.trim_output_mmps = 0.0f;
    wall_follow_drive_test.left_command = 0.0f;
    wall_follow_drive_test.right_command = 0.0f;
    wall_follow_drive_test.trim_enabled = 0;
}

static unsigned int absoluteSensorDelta(unsigned int a, unsigned int b)
{
    if (a >= b) {
        return a - b;
    }

    return b - a;
}

static void applyWallFollowDriveLedState(void)
{
    if (wall_follow_drive_test.enabled) {
        LED_RED = LEDON;
        LED_BLUE = LEDON;
    } else {
        LED_RED = LEDOFF;
        LED_BLUE = LEDOFF;
    }
}

static void updateWallFollowDriveButtonState(void)
{
    if (!selfDestructWasPressed()) {
        return;
    }

    clearSelfDestructPressFlag();
    wall_follow_drive_test.enabled = !wall_follow_drive_test.enabled;

    if (wall_follow_drive_test.enabled) {
        wall_follow_drive_test.base_target_mps = ((float)WALL_FOLLOW_BASE_TARGET_MMPS) / 1000.0f;
        wall_follow_drive_test.trim_enabled = 1;
    } else {
        wall_follow_drive_test.base_target_mps = 0.0f;
        wall_follow_drive_test.left_target_mps = 0.0f;
        wall_follow_drive_test.right_target_mps = 0.0f;
        resetWallFollowDriveTest();
        stopMotors();
    }

    applyWallFollowDriveLedState();
}

void initWallFollowDriveTest(void)
{
    wall_follow_drive_test.base_target_mps = 0.0f;
    wall_follow_drive_test.left_target_mps = 0.0f;
    wall_follow_drive_test.right_target_mps = 0.0f;
    wall_follow_drive_test.previous_left_sensor_value = 0U;
    wall_follow_drive_test.previous_right_sensor_value = 0U;
    wall_follow_drive_test.enabled = 0;
    resetWallFollowDriveTest();
    applyWallFollowDriveLedState();
    clearSelfDestructPressFlag();
    stopMotors();
}

void updateWallFollowDriveTest(float measured_left_speed_mps,
                               float measured_right_speed_mps,
                               unsigned int left_sensor_value,
                               unsigned int right_sensor_value)
{
    float wall_error;
    float trim_proportional_mmps;
    float trim_total_mmps;
    float left_error;
    float right_error;
    float left_proportional;
    float right_proportional;
    unsigned int left_sensor_delta;
    unsigned int right_sensor_delta;

    updateWallFollowDriveButtonState();

    if (!wall_follow_drive_test.enabled) {
        resetWallFollowDriveTest();
        stopMotors();
        return;
    }

    left_sensor_delta = absoluteSensorDelta(left_sensor_value, wall_follow_drive_test.previous_left_sensor_value);
    right_sensor_delta = absoluteSensorDelta(right_sensor_value, wall_follow_drive_test.previous_right_sensor_value);
    wall_follow_drive_test.previous_left_sensor_value = left_sensor_value;
    wall_follow_drive_test.previous_right_sensor_value = right_sensor_value;

    wall_follow_drive_test.trim_enabled =
        (left_sensor_delta < WALL_FOLLOW_SENSOR_DELTA_DISABLE_THRESHOLD)
        && (right_sensor_delta < WALL_FOLLOW_SENSOR_DELTA_DISABLE_THRESHOLD);

    if (wall_follow_drive_test.trim_enabled) {
        wall_error = (float)right_sensor_value - (float)left_sensor_value;
        trim_proportional_mmps = WALL_FOLLOW_TRIM_KP * wall_error;
        wall_follow_drive_test.trim_integral_mmps +=
            WALL_FOLLOW_TRIM_KI * wall_error * RIGHT_PI_TEST_SAMPLE_TIME_S;
        trim_total_mmps = trim_proportional_mmps + wall_follow_drive_test.trim_integral_mmps;
        wall_follow_drive_test.trim_output_mmps =
            clampSymmetric(trim_total_mmps, (float)WALL_FOLLOW_MAX_TRIM_MMPS);
    } else {
        wall_follow_drive_test.trim_integral_mmps = 0.0f;
        wall_follow_drive_test.trim_output_mmps = 0.0f;
    }

    wall_follow_drive_test.left_target_mps =
        wall_follow_drive_test.base_target_mps - (wall_follow_drive_test.trim_output_mmps / 1000.0f);
    wall_follow_drive_test.right_target_mps =
        wall_follow_drive_test.base_target_mps + (wall_follow_drive_test.trim_output_mmps / 1000.0f);

    left_error = wall_follow_drive_test.left_target_mps - measured_left_speed_mps;
    right_error = wall_follow_drive_test.right_target_mps - measured_right_speed_mps;
    left_proportional = RIGHT_PI_TEST_KP * left_error;
    right_proportional = RIGHT_PI_TEST_KP * right_error;

    wall_follow_drive_test.left_integral += RIGHT_PI_TEST_KI * left_error * RIGHT_PI_TEST_SAMPLE_TIME_S;
    wall_follow_drive_test.right_integral += RIGHT_PI_TEST_KI * right_error * RIGHT_PI_TEST_SAMPLE_TIME_S;

    wall_follow_drive_test.left_command = clampUnit(left_proportional + wall_follow_drive_test.left_integral);
    wall_follow_drive_test.right_command = clampUnit(right_proportional + wall_follow_drive_test.right_integral);

    setLeftMotor(wall_follow_drive_test.left_command);
    setRightMotor(wall_follow_drive_test.right_command);
}

int getWallFollowBaseTargetMmps(void)
{
    return (int)(wall_follow_drive_test.base_target_mps * 1000.0f);
}

int getWallFollowTrimMmps(void)
{
    return (int)(wall_follow_drive_test.trim_output_mmps);
}

int getWallFollowLeftCommandPermille(void)
{
    return (int)(wall_follow_drive_test.left_command * 1000.0f);
}

int getWallFollowRightCommandPermille(void)
{
    return (int)(wall_follow_drive_test.right_command * 1000.0f);
}
