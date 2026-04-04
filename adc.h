
#ifndef ADC_H
#define	ADC_H

#include <xc.h>


void setupADC1();
void startADC1();
unsigned int readLeftSensorValue(void);
unsigned int readMidSensorValue(void);
unsigned int readRightSensorValue(void);

#endif	/* ADC_H */
