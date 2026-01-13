
#include "myPWM.h"
#include <xc.h>
void setupPWM()
{
        /* PWM1H1 *, configured to 1kHz, based on fcyc = 26.666 MIPS, Tcycle=37.5nsec/
         * 1ms/37.5nsec = 26666.666 ==> 26666 (fits in 15 bits)
         * of course, we could use a pre-scaler and end up somewhere else
         */
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    P1TCONbits.PTCKPS = 0b00; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    P1TPER = MYPWM_MAX/2; //15 bit register
    PWM1CON1bits.PMOD1 = 1; // set PWM unit 1 to independent mode
    
    // Map PWM2H1 to RP14 (LED5) own code, maybe trash
    //RPOR7bits.RP14R = 0b01010;   // 01010 = PWM2H1 code
    
    PWM1CON1bits.PEN1H = 1; // enable  PWM driver PWM1H1 (LED5)
    PWM1CON1bits.PEN2H = 1; // enable PWM driver PMW2H1 (LED7)
    PWM1CON1bits.PEN3H = 1; // disable PWM driver
    PWM1CON1bits.PEN1L = 0; // disable PWM driver 
    PWM1CON1bits.PEN2L = 0; // disable PWM driver
    PWM1CON1bits.PEN3L = 0; // disable PWM driver

    P1TCONbits.PTEN = 1; // Switch on PWM generator
    P1DC1 = .1*MYPWM_MAX; //to get 100% DC, you need to write twice the PER Value (2*26666)
    P1DC2 = .1*MYPWM_MAX;
    P1DC3 = .1*MYPWM_MAX;
}

void setDCPWM1(float dc)
{
    P1DC1 = (1-dc) * MYPWM_MAX;
}

void setDCPWM2(float dc){
    P1DC2 = (1-dc) * MYPWM_MAX;
}
void setDCPWM3(float dc){
    P1DC3 = (1-dc) * MYPWM_MAX;
}
