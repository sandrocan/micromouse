#include "sensors.h"
#include "adc.h"

int isWallLeft(void)
{
    return (readLeftSensorValue() >= WALL_THRESHOLD_LEFT);
}

int isWallRight(void)
{
    return (readRightSensorValue() >= WALL_THRESHOLD_RIGHT);
}

int isWallFront(void)
{
    return (readMidSensorValue() >= WALL_THRESHOLD_FRONT);
}
