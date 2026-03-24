#ifndef DMA_H 
#define	DMA_H


#include <xc.h>
    
extern unsigned int adcData[32]__attribute__((space(dma)));


//add some defines to make accessing data more readable

#define SENSOR_RIGHT adcData[0] //AN0
#define SENSOR_MIDDLE adcData[1]  //AN1
#define SENSOR_LEFT adcData[1]  //AN8


void initDMA();



#endif	/* DMA_H */
