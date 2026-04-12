#include "motors.h"

#include "IOconfig.h"
#include "pwm.h"

static volatile long left_encoder_turns = 0;
static volatile long right_encoder_turns = 0;
static float left_motor_command = 0.0f;
static float right_motor_command = 0.0f;
static int motor_stop_latched = 0;

#define ENCODER_COUNTS_PER_WHEEL_REV (16.0f * 4.0f * 33.0f)
#define WHEEL_DIAMETER_M (0.06f)
#define TIMER_SAMPLE_TIME_S (0.01f)
#define MOTOR_SUPPLY_VOLTAGE_V (7.4f)
#define MOTOR_MAX_VOLTAGE_V (6.0f)
#define MOTOR_MAX_DUTY_CYCLE (MOTOR_MAX_VOLTAGE_V / MOTOR_SUPPLY_VOLTAGE_V)

// Project convention:
// positive motor command  => robot drives forward
// positive encoder counts => robot drives forward
//
// Mechanical direction:
// left motor forward  = counter-clockwise
// right motor forward = clockwise
#define LEFT_MOTOR_FORWARD_IN1_STATE (1)
#define LEFT_MOTOR_FORWARD_IN2_STATE (0)
#define RIGHT_MOTOR_FORWARD_IN1_STATE (1)
#define RIGHT_MOTOR_FORWARD_IN2_STATE (0)

#define LEFT_ENCODER_FORWARD_SIGN (1L)
#define RIGHT_ENCODER_FORWARD_SIGN (-1L)

static float clampDutyCycle(float speed)
{
    float duty_cycle;

    if (speed < 0.0f)
    {
        duty_cycle = -speed;
    }
    else
    {
        duty_cycle = speed;
    }

    if (duty_cycle > MOTOR_MAX_DUTY_CYCLE)
    {
        duty_cycle = MOTOR_MAX_DUTY_CYCLE;
    }

    return duty_cycle;
}

static float clampMotorCommand(float speed)
{
    if (speed > 1.0f)
    {
        return 1.0f;
    }

    if (speed < -1.0f)
    {
        return -1.0f;
    }

    return speed;
}

static long normalizeEncoderCounts(long raw_counts, long forward_sign)
{
    return forward_sign * raw_counts;
}

static void applyLeftMotorOutput(float speed)
{
    float duty_cycle = clampDutyCycle(speed);

    if (speed > 0.0f)
    {
        MOTOR_LEFT_IN1 = LEFT_MOTOR_FORWARD_IN1_STATE;
        MOTOR_LEFT_IN2 = LEFT_MOTOR_FORWARD_IN2_STATE;
    }
    else if (speed < 0.0f)
    {
        MOTOR_LEFT_IN1 = !LEFT_MOTOR_FORWARD_IN1_STATE;
        MOTOR_LEFT_IN2 = !LEFT_MOTOR_FORWARD_IN2_STATE;
    }
    else
    {
        MOTOR_LEFT_IN1 = 0;
        MOTOR_LEFT_IN2 = 0;
    }

    setDCMotorLeft(duty_cycle);
}

static void applyRightMotorOutput(float speed)
{
    float duty_cycle = clampDutyCycle(speed);

    if (speed > 0.0f)
    {
        MOTOR_RIGHT_IN1 = RIGHT_MOTOR_FORWARD_IN1_STATE;
        MOTOR_RIGHT_IN2 = RIGHT_MOTOR_FORWARD_IN2_STATE;
    }
    else if (speed < 0.0f)
    {
        MOTOR_RIGHT_IN1 = !RIGHT_MOTOR_FORWARD_IN1_STATE;
        MOTOR_RIGHT_IN2 = !RIGHT_MOTOR_FORWARD_IN2_STATE;
    }
    else
    {
        MOTOR_RIGHT_IN1 = 0;
        MOTOR_RIGHT_IN2 = 0;
    }

    setDCMotorRight(duty_cycle);
}

static void refreshMotorOutputs(void)
{
    if (motor_stop_latched)
    {
        applyLeftMotorOutput(0.0f);
        applyRightMotorOutput(0.0f);
        return;
    }

    applyLeftMotorOutput(left_motor_command);
    applyRightMotorOutput(right_motor_command);
}

void initEncoders(void)
{
    QEI1CONbits.QEIM = 0;
    QEI2CONbits.QEIM = 0;

    QEI1CONbits.QEISIDL = 1;
    QEI1CONbits.SWPAB = 1;
    QEI1CONbits.PCDOUT = 0;
    QEI1CONbits.TQGATE = 0;
    QEI1CONbits.POSRES = 0;
    QEI1CONbits.TQCS = 0;
    QEI1CONbits.UPDN_SRC = 0;
    MAX1CNT = 0xFFFF;
    POS1CNT = 0;
    left_encoder_turns = 0;
    IFS3bits.QEI1IF = 0;
    IEC3bits.QEI1IE = 1;
    IPC14bits.QEI1IP = 5;
    QEI1CONbits.QEIM = 0b111;

    QEI2CONbits.QEISIDL = 1;
    QEI2CONbits.SWPAB = 1;
    QEI2CONbits.PCDOUT = 0;
    QEI2CONbits.TQGATE = 0;
    QEI2CONbits.POSRES = 0;
    QEI2CONbits.TQCS = 0;
    QEI2CONbits.UPDN_SRC = 0;
    MAX2CNT = 0xFFFF;
    POS2CNT = 0;
    right_encoder_turns = 0;
    IFS4bits.QEI2IF = 0;
    IEC4bits.QEI2IE = 1;
    IPC18bits.QEI2IP = 5;
    QEI2CONbits.QEIM = 0b111;
}

long readLeftEncoderCounts(void)
{
    long raw_counts;

    _NSTDIS = 1;
    raw_counts = left_encoder_turns + POS1CNT;
    _NSTDIS = 0;

    return normalizeEncoderCounts(raw_counts, LEFT_ENCODER_FORWARD_SIGN);
}

long readRightEncoderCounts(void)
{
    long raw_counts;

    _NSTDIS = 1;
    raw_counts = right_encoder_turns + POS2CNT;
    _NSTDIS = 0;

    return normalizeEncoderCounts(raw_counts, RIGHT_ENCODER_FORWARD_SIGN);
}

float readLeftMotorSpeedMps(void)
{
    static long previous_counts = 0;
    long current_counts = readLeftEncoderCounts();
    long delta_counts = current_counts - previous_counts;
    float wheel_circumference_m = 3.14159265f * WHEEL_DIAMETER_M;
    float speed_mps;

    // counts_per_wheel_rev = 16 pulses * 4 quadrature edges * 33 gear ratio = 2112 counts/rev
    // wheel_circumference = pi * 0.06 m
    // speed = (delta_counts / counts_per_wheel_rev) * wheel_circumference / 0.01 s
    speed_mps = ((float)delta_counts / ENCODER_COUNTS_PER_WHEEL_REV) * wheel_circumference_m / TIMER_SAMPLE_TIME_S;
    previous_counts = current_counts;

    return speed_mps;
}

float readRightMotorSpeedMps(void)
{
    static long previous_counts = 0;
    long current_counts = readRightEncoderCounts();
    long delta_counts = current_counts - previous_counts;
    float wheel_circumference_m = 3.14159265f * WHEEL_DIAMETER_M;
    float speed_mps;

    // counts_per_wheel_rev = 16 pulses * 4 quadrature edges * 33 gear ratio = 2112 counts/rev
    // wheel_circumference = pi * 0.06 m
    // speed = (delta_counts / counts_per_wheel_rev) * wheel_circumference / 0.01 s
    speed_mps = ((float)delta_counts / ENCODER_COUNTS_PER_WHEEL_REV) * wheel_circumference_m / TIMER_SAMPLE_TIME_S;
    previous_counts = current_counts;

    return speed_mps;
}

void initMotors(void)
{
    left_motor_command = 0.0f;
    right_motor_command = 0.0f;
    motor_stop_latched = 0;
    refreshMotorOutputs();
}

void setLeftMotor(float speed)
{
    left_motor_command = clampMotorCommand(speed);
    refreshMotorOutputs();
}

void setRightMotor(float speed)
{
    right_motor_command = clampMotorCommand(speed);
    refreshMotorOutputs();
}

void setMotorStopLatch(int enabled)
{
    motor_stop_latched = (enabled != 0);
    refreshMotorOutputs();
}

void toggleMotorStopLatch(void)
{
    motor_stop_latched = !motor_stop_latched;
    refreshMotorOutputs();
}

int isMotorStopLatched(void)
{
    return motor_stop_latched;
}

void stopMotors(void)
{
    left_motor_command = 0.0f;
    right_motor_command = 0.0f;
    refreshMotorOutputs();
}

void brakeMotors(void)
{
    left_motor_command = 0.0f;
    right_motor_command = 0.0f;

    // Request active braking on the H-bridge instead of coasting.
    MOTOR_LEFT_IN1 = 1;
    MOTOR_LEFT_IN2 = 1;
    MOTOR_RIGHT_IN1 = 1;
    MOTOR_RIGHT_IN2 = 1;
    setDCMotorLeft(0.0f);
    setDCMotorRight(0.0f);
}

float getLeftDistanceMeters(void)
{
    long counts = readLeftEncoderCounts();
    float wheel_circumference_m = 3.14159265f * WHEEL_DIAMETER_M;

    return ((float)counts / ENCODER_COUNTS_PER_WHEEL_REV) * wheel_circumference_m;
}
float getRightDistanceMeters(void)
{
    long counts = readRightEncoderCounts();
    float wheel_circumference_m = 3.14159265f * WHEEL_DIAMETER_M;

    return ((float)counts / ENCODER_COUNTS_PER_WHEEL_REV) * wheel_circumference_m;
}

float getLeftRotations(void)
{
    long counts = readLeftEncoderCounts();
    return (float)counts / ENCODER_COUNTS_PER_WHEEL_REV;
}

float getRightRotations(void)
{
    long counts = readRightEncoderCounts();
    return (float)counts / ENCODER_COUNTS_PER_WHEEL_REV;
}

void __attribute__((__interrupt__, auto_psv)) _QEI1Interrupt(void)
{
    IFS3bits.QEI1IF = 0;

    if (POS1CNT < 32768)
    {
        left_encoder_turns += 0x10000L;
    }
    else
    {
        left_encoder_turns -= 0x10000L;
    }
}

void __attribute__((__interrupt__, auto_psv)) _QEI2Interrupt(void)
{
    IFS4bits.QEI2IF = 0;

    if (POS2CNT < 32768)
    {
        right_encoder_turns += 0x10000L;
    }
    else
    {
        right_encoder_turns -= 0x10000L;
    }
}
