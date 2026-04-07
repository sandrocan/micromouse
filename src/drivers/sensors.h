#ifndef SENSORS_H
#define SENSORS_H

// Wall detection thresholds per sensor.
// Higher ADC value = closer wall.  Calibrate these on the actual maze surface.
#define WALL_THRESHOLD_LEFT  (500U)
#define WALL_THRESHOLD_RIGHT (500U)
#define WALL_THRESHOLD_FRONT (1000U)

int isWallLeft(void);
int isWallRight(void);
int isWallFront(void);

#endif /* SENSORS_H */
