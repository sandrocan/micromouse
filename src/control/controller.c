#include "controller.h"

#include "IOconfig.h"
#include "motors.h"
#include "adc.h"

#define WHEEL_SPEED_KP (1.6f)
#define WHEEL_SPEED_KI (18.0f)

// The wheel speed controller runs from the 10 ms timer interrupt.
#define WHEEL_SPEED_SAMPLE_TIME_S (0.01f)

// In-place 90 degree turns use equal wheel speeds in opposite directions and
// stop once the average encoder travel reaches this count.
#define TURN_SPEED_MMPS (500)
#define TURN_90_TARGET_COUNTS (640)

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
    float integral_term;
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

static long absoluteCountDelta(long value)
{
    if (value < 0L)
    {
        return -value;
    }

    return value;
}

static void resetWheelSpeedController(WheelSpeedController *controller)
{
    controller->integral_term = 0.0f;
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


void turnLeft90(void)
{
    drive_state.mode = CONTROLLER_MODE_TURN_LEFT_90;
    drive_state.left_turn_start_counts = readLeftEncoderCounts();
    drive_state.right_turn_start_counts = readRightEncoderCounts();
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
}

void turnRight90(void)
{
    drive_state.mode = CONTROLLER_MODE_TURN_RIGHT_90;
    drive_state.left_turn_start_counts = readLeftEncoderCounts();
    drive_state.right_turn_start_counts = readRightEncoderCounts();
    resetWheelSpeedController(&drive_state.left_wheel);
    resetWheelSpeedController(&drive_state.right_wheel);
}

static void updateWheelSpeedController(WheelSpeedController *controller, float measured_wheel_speed_mps)
{
    // Drive motors with same speed based on measured motor speed
    float speed_error = controller->target_speed_mps - measured_wheel_speed_mps;
    float proportional_term = WHEEL_SPEED_KP * speed_error;

    controller->integral_term += WHEEL_SPEED_KI * speed_error * WHEEL_SPEED_SAMPLE_TIME_S;
    controller->adjusted_speed_mps = clampMotorCommand((float) (proportional_term + controller->integral_term));
}

void updateController(void)
{
    float measured_left_wheel_speed_mps = readLeftMotorSpeedMps();
    float measured_right_wheel_speed_mps = readRightMotorSpeedMps();

    unsigned int dist_mid = readMidSensorValue();

    if (dist_mid > 500)
    {
        stopDriveControl();
        return;
    }

    if (drive_state.mode == CONTROLLER_MODE_STOP)
    {
        stopDriveControl();
        return;
    }

    updateWheelSpeedController(&drive_state.left_wheel, measured_left_wheel_speed_mps);
    updateWheelSpeedController(&drive_state.right_wheel, measured_right_wheel_speed_mps);
    
    setLeftMotor(drive_state.left_wheel.adjusted_speed_mps);
    setRightMotor(drive_state.right_wheel.adjusted_speed_mps);

    // if (drive_state.mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    // {
    //     // Set target speeds of the wheel speed controllers.
    //     drive_state.left_wheel.target_speed_mps = drive_state.drive_target_speed_mps;
    //     drive_state.right_wheel.target_speed_mps = drive_state.drive_target_speed_mps;

    //     float left_speed = updateWheelSpeedController(&drive_state.left_wheel, measured_left_wheel_speed_mps);
    //     float right_speed = updateWheelSpeedController(&drive_state.right_wheel, measured_right_wheel_speed_mps);

    //     // Adjust motor speed such that mouse stays centered based on sensor readings
    //     unsigned int dist_right = getRightSensorValue();
    //     unsigned int dist_left = getLeftSensorValue(); 

    //     if (dist_right < 500 || dist_left < 500)
    //     {
    //         return controller->motor_command;   // Case: Only single wall detected
    //         setLeftMotor(drive_state.left_);
    //         setRightMotor();
    //     }
    //     else
    //     {
    //         int diff = dist_right - dist_left;
    //         if (diff < 100 && diff > -100)
    //         {
    //             return controller->motor_command;
    //         }
    //         else if (diff < 0)
    //         {
                
    //         }
    //     }
        
        
    //     return;
    // }
    
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
