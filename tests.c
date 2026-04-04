#include "tests.h"

#include "IOconfig.h"
#include "motors.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>

#define PI_TEST_SAMPLE_TIME_S (0.01f)
#define PI_TEST_MAX_TARGET_SPEED_MPS (1.40f)
#define PI_TEST_MAX_OUTPUT_STEP (0.02f)
#define PI_TEST_DEBOUNCE_TICKS (3U)

static float percentageToTargetSpeed(float percentage)
{
    return percentage * PI_TEST_MAX_TARGET_SPEED_MPS;
}

typedef struct {
    float kp;
    float ki;
    float target_mps;
    float measured_left_mps;
    float measured_right_mps;
    float integrator;
    float output;
    unsigned int debounce_ticks;
    int last_raw_button;
    int debounced_button;
    unsigned int sequence_index;
    int initialized;
} TestPIController;

static TestPIController test_controller;

static float clampOutput(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }

    if (value < 0.0f) {
        return 0.0f;
    }

    return value;
}

static float limitOutputStep(float previous_output, float requested_output)
{
    float delta = requested_output - previous_output;

    if (delta > PI_TEST_MAX_OUTPUT_STEP) {
        return previous_output + PI_TEST_MAX_OUTPUT_STEP;
    }

    if (delta < -PI_TEST_MAX_OUTPUT_STEP) {
        return previous_output - PI_TEST_MAX_OUTPUT_STEP;
    }

    return requested_output;
}

static void resetTestControllerOutput(void)
{
    test_controller.integrator = 0.0f;
    test_controller.output = 0.0f;
}

static void advanceTestSequence(void)
{
    static const float sequence[] = { 0.30f, 0.70f, 0.0f, 0.50f, 0.0f };
    char uart_buffer[48];

    test_controller.target_mps = percentageToTargetSpeed(sequence[test_controller.sequence_index]);
    resetTestControllerOutput();
    snprintf(uart_buffer, sizeof(uart_buffer), "PI test mode: %.0f%%\r\n", sequence[test_controller.sequence_index] * 100.0f);
    writeUART(uart_buffer);
    test_controller.sequence_index++;

    if (test_controller.sequence_index >= (sizeof(sequence) / sizeof(sequence[0]))) {
        test_controller.sequence_index = 0;
    }
}

void test_PI_controller(float kp, float ki)
{
    if (test_controller.initialized) {
        return;
    }

    test_controller.kp = kp;
    test_controller.ki = ki;
    test_controller.target_mps = 0.0f;
    test_controller.measured_left_mps = 0.0f;
    test_controller.measured_right_mps = 0.0f;
    test_controller.integrator = 0.0f;
    test_controller.output = 0.0f;
    test_controller.debounce_ticks = 0;
    test_controller.last_raw_button = SELFDESTRUCT;
    test_controller.debounced_button = SELFDESTRUCT;
    test_controller.sequence_index = 0;
    test_controller.initialized = 1;

    setMotorStopLatch(0);
    setLeftMotor(0.0f);
    setRightMotor(0.0f);
}

void update_test_PI_controller(void)
{
    int raw_button;
    static int print_counter = 0;
    char uart_buffer[64];
    float error;
    float requested_output;

    if (!test_controller.initialized) {
        return;
    }

    raw_button = SELFDESTRUCT;

    if (raw_button == test_controller.last_raw_button) {
        if (test_controller.debounce_ticks < PI_TEST_DEBOUNCE_TICKS) {
            test_controller.debounce_ticks++;
        }
    } else {
        test_controller.debounce_ticks = 0;
        test_controller.last_raw_button = raw_button;
    }

    if ((test_controller.debounce_ticks >= PI_TEST_DEBOUNCE_TICKS) &&
        (raw_button != test_controller.debounced_button)) {
        test_controller.debounced_button = raw_button;

        if (test_controller.debounced_button) {
            advanceTestSequence();
        }
    }

    test_controller.measured_left_mps = readLeftMotorSpeedMps();
    // For this open one-direction PI test we only care about forward speed magnitude.
    test_controller.measured_right_mps = fabsf(readRightMotorSpeedMps());

    if (test_controller.target_mps <= 0.0f) {
        resetTestControllerOutput();
        setLeftMotor(0.0f);
        setRightMotor(0.0f);
        return;
    }

    error = test_controller.target_mps - test_controller.measured_right_mps;
    test_controller.integrator += error * PI_TEST_SAMPLE_TIME_S;
    requested_output = (test_controller.kp * error) + (test_controller.ki * test_controller.integrator);
    requested_output = clampOutput(requested_output);
    test_controller.output = limitOutputStep(test_controller.output, requested_output);

    setLeftMotor(0.0f);
    setRightMotor(test_controller.output);

    print_counter++;

    if (print_counter >= 10) {
        print_counter = 0;
        snprintf(uart_buffer, sizeof(uart_buffer), "left=%d right=%d mm/s\r\n",
                 (int)(test_controller.measured_left_mps * 1000.0f),
                 (int)(test_controller.measured_right_mps * 1000.0f));
        writeUART(uart_buffer);
    }
}

float get_test_left_speed_mps(void)
{
    return test_controller.measured_left_mps;
}

float get_test_right_speed_mps(void)
{
    return test_controller.measured_right_mps;
}
