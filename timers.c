#include "timers.h"
#include "IOconfig.h"
#include "uart.h"
#include "pwm.h"
#include <math.h>

static void updateLEDGreenSine(void)
{
    static float phase = 0.0f;
    const float step = 0.06283185f; // 2*pi/100 -> 1 s period at 10 ms interrupt
    float duty_cycle = 0.5f + 0.5f * sinf(phase);

    setDCLEDGreen(duty_cycle);
    phase += step;

    if (phase >= 6.28318531f) {
        phase -= 6.28318531f;
    }
}

void initTimer1ms(float timeinms)
{
    T1CON = 0;  //Turn all bits off
    
    //Fcyc = 80.0MHz
    //tcyc = 12.5ns
    
    //period = prescaler/Fcyc
    //N is the maximum number of ticks: 2¹⁵-1 = 65535
    //The counter cannot exceed this number, so the maximum time reachable is:
    //tmax = N * period * 1000 (the 1000 comes from transforming to ms)
    //tmax = (N * prescaler * 1000) / Fcyc
    
    //We want to keep the prescaler as low as possible to get numbers as high
    //as possible to get less rounding errors.
    //The four prescalers of the encoder are 1, 8, 64, 256

    float tmax_8 = 5.50;
    float tmax_64 = 51.20;
    float tmax_256 = 208.50;
    
    int prescaler = 0;
    
    if (timeinms <= tmax_8)
    {
        T1CONbits.TCKPS = 0b01;
        prescaler = 8;
    }
    
    else if (timeinms <= tmax_64)
    {
        T1CONbits.TCKPS = 0b10;
        prescaler = 64;
    }
    
    else if (timeinms <= tmax_256)
    {
        T1CONbits.TCKPS = 0b11;
        prescaler = 256;
    }
    
    else
    {
        return;
    }
    
    T1CONbits.TCS = 0;      // select internal FCY clock source    
    T1CONbits.TGATE = 0;    // gated time accumulation disabled
    TMR1 = 0;
    //PR1 = (80000/prescaler) * timeinms -1;
    PR1 = (uint16_t)( ((80000.00f * (float)timeinms) / (float)prescaler) + 0.5f ) - 1;
    IFS0bits.T1IF = 0;      // reset Timer 1 interrupt flag
    IPC0bits.T1IP = 4;      // set Timer1 interrupt priority level to 4
    IEC0bits.T1IE = 1;      // enable Timer 1 interrupt
    T1CONbits.TON = 0;      // leave timer disabled initially 
}

void startTimer1(void) 
{
    T1CONbits.TON = 1; //
 
}


void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;           // reset Timer 1 interrupt flag 
    //updateLEDGreenSine();

    static int cnt = 0;
    cnt++;

    if (cnt>100) {
        cnt=0;
        LED_BLUE = !LED_BLUE;
    }
}
