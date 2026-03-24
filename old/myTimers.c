#include "myTimers.h"
#include "IOconfig.h"
#include "myPWM.h"
#include "math.h"
#include "dma.h"
#include "serialComms.h"
#include "motorEncoders.h"
#include <stdio.h>

void initTimer1(unsigned int period)  
{
    //unsigned TimerControlValue;
    
    T1CON = 0;              // ensure Timer 1 is in reset state
    //TimerControlValue=T1CON;
 
    T1CONbits.TCKPS = 0b10; // FCY divide by 64: tick = 2.4us (Tcycle=37.5ns)
    T1CONbits.TCS = 0;      // select internal FCY clock source
    T1CONbits.TGATE = 0;    // gated time accumulation disabled
    TMR1 = 0;
    PR1 = period;           // set Timer 1 period register ()
    IFS0bits.T1IF = 0;      // reset Timer 1 interrupt flag
    IPC0bits.T1IP = 4;      // set Timer1 interrupt priority level to 4
    IEC0bits.T1IE = 1;      // enable Timer 1 interrupt
    T1CONbits.TON = 0;      // leave timer disabled initially
}

void startTimer1(void) 
{
    T1CONbits.TON = 1; //
 
}
void initTimer1InMS(unsigned int timeInMS)
{
    unsigned long time_in_cycles = timeInMS * 26667ul;
    if(time_in_cycles < 1 * 65535)
    {
        T1CONbits.TCKPS = 0b00;
        PR1 = (unsigned int) time_in_cycles;
    }
    else if (time_in_cycles < 8 * 65535)
    {
        T1CONbits.TCKPS = 0b01;
        PR1 = (unsigned int) time_in_cycles/8;
    }
    else if (time_in_cycles < 64 * 65535)
    {
        T1CONbits.TCKPS = 0b10;
        PR1 = (unsigned int) time_in_cycles/64;
    }
    else if (time_in_cycles < 256 * 65535)
    {
        T1CONbits.TCKPS = 0b11;
        PR1 = (unsigned int) time_in_cycles/256;
    }
    else
    {
        return;
    }
    
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;           // reset Timer 1 interrupt flag 
    
    float val_sensor = (float) TEST_SENSOR;
    drive_motor(val_sensor);
    
    //IO9 and IO10 connected to encoder
    //VSS = GND
    char uart [64];
    long position;
    //position = getPositionInCounts_1();
    position = getVelocityInCountsPerSample_1();
    LED7 =~ LED7;
    sprintf(uart, "Position: %ld\r\n",position);
    putsUART1(uart);
    
}
