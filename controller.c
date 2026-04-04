#include "controller.h"

#include "motors.h"

#define CONTROLLER_SAMPLE_TIME_S (0.01f)
#define CONTROLLER_OUTPUT_LIMIT (1.0f)
#define CONTROLLER_SPEED_DEADBAND_MPS (0.01f)
#define CONTROLLER_MAX_OUTPUT_STEP (0.02f)

typedef struct {
    float kp;
    float ki;
    float target_mps;
    float measured_mps;
    float integrator;
    float output;
} PIController;

static PIController left_controller;
static PIController right_controller;

static float clampControl(float value)
{
    if (value > CONTROLLER_OUTPUT_LIMIT) {
        return CONTROLLER_OUTPUT_LIMIT;
    }

    if (value < -CONTROLLER_OUTPUT_LIMIT) {
        return -CONTROLLER_OUTPUT_LIMIT;
    }

    return value;
}

static float limitOutputStep(float previous_output, float requested_output)
{
    float delta = requested_output - previous_output;

    if (delta > CONTROLLER_MAX_OUTPUT_STEP) {
        return previous_output + CONTROLLER_MAX_OUTPUT_STEP;
    }

    if (delta < -CONTROLLER_MAX_OUTPUT_STEP) {
        return previous_output - CONTROLLER_MAX_OUTPUT_STEP;
    }

    return requested_output;
}

static void resetControllerState(PIController *controller, float kp, float ki)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->target_mps = 0.0f;
    controller->measured_mps = 0.0f;
    controller->integrator = 0.0f;
    controller->output = 0.0f;
}

static float updatePIController(PIController *controller, float measured_speed)
{
    float error = controller->target_mps - measured_speed;
    float candidate_integrator = controller->integrator + error * CONTROLLER_SAMPLE_TIME_S;
    float unclamped_output = controller->kp * error + controller->ki * candidate_integrator;
    float clamped_output = clampControl(unclamped_output);

    controller->measured_mps = measured_speed;

    if ((controller->target_mps < CONTROLLER_SPEED_DEADBAND_MPS) &&
        (controller->target_mps > -CONTROLLER_SPEED_DEADBAND_MPS) &&
        (measured_speed < CONTROLLER_SPEED_DEADBAND_MPS) &&
        (measured_speed > -CONTROLLER_SPEED_DEADBAND_MPS)) {
        controller->integrator = 0.0f;
        controller->output = 0.0f;
        return 0.0f;
    }

    // Anti-windup: only accept the new integrator if the output was not saturated.
    if (unclamped_output == clamped_output) {
        controller->integrator = candidate_integrator;
    }

    clamped_output = limitOutputStep(controller->output, clamped_output);
    controller->output = clamped_output;
    return clamped_output;
}

void initLeftMotorController(float kp, float ki)
{
    resetControllerState(&left_controller, kp, ki);
}

void initRightMotorController(float kp, float ki)
{
    resetControllerState(&right_controller, kp, ki);
}

void setLeftMotorSpeedTarget(float target_mps)
{
    left_controller.target_mps = target_mps;
}

void setRightMotorSpeedTarget(float target_mps)
{
    right_controller.target_mps = target_mps;
}

void updateMotorControllers(void)
{
    float left_speed = readLeftMotorSpeedMps();
    float right_speed = readRightMotorSpeedMps();

    setLeftMotor(updatePIController(&left_controller, left_speed));
    setRightMotor(updatePIController(&right_controller, right_speed));
}

float getLeftMeasuredSpeedMps(void)
{
    return left_controller.measured_mps;
}

float getRightMeasuredSpeedMps(void)
{
    return right_controller.measured_mps;
}

float getLeftMotorSpeedTarget(void)
{
    return left_controller.target_mps;
}

float getRightMotorSpeedTarget(void)
{
    return right_controller.target_mps;
}

float getLeftControlEffort(void)
{
    return left_controller.output;
}

float getRightControlEffort(void)
{
    return right_controller.output;
}
