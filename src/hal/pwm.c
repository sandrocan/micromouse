#include "pwm.h"

void setupPWM() {

    //PWM1 configured at 4kHz: 0.25ms/12.5ns = 20000

    P1TCONbits.PTEN = 0; // Switch off PWM generator
    P2TCONbits.PTEN = 0;

    P1TCONbits.PTCKPS = 0b00; //No prescaler used!
    P2TCONbits.PTCKPS = 0b00;

    P1TPER = PWM_MAX/2; //15 bit register
    P2TPER = PWM_MAX/2;

    PWM1CON1bits.PMOD1 = 1; // set PWM units to independent mode
    PWM1CON1bits.PMOD3 = 0; // complementary mode for PWM1 channel 3 (Blaulicht)
    PWM2CON1bits.PMOD1 = 1;
    
    PWM1CON1bits.PEN1H = 1; // enable PWM driver PWM1H1 (Motor left)
    PWM1CON1bits.PEN2H = 1; // enable PWM driver PMW1H2 (Motor right)
    PWM1CON1bits.PEN3H = 1; // enable PWM driver PWM1H3 (RB10 / EXTRA5)
    PWM1CON1bits.PEN1L = 0; // disable PWM driver
    PWM1CON1bits.PEN2L = 0; // disable PWM driver
    PWM1CON1bits.PEN3L = 1; // enable PWM driver PWM1L3 (RB11 / EXTRA3)

    PWM2CON1bits.PEN1H = 1; // enable PWM driver PWM2H1 (LED_GREEN)
    PWM2CON1bits.PEN1L = 1; // (EXTRA 7)



    P1TCONbits.PTEN = 1; // Switch on PWM generator
    P2TCONbits.PTEN = 1;
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
    P2DC1 = 0;
}


void setDCMotorLeft (float dc)
{
    P1DC1 = dc * PWM_MAX;
}


void setDCMotorRight(float dc){
    P1DC2 = dc * PWM_MAX;
}


void setDCLEDGreen(float dc){
    P2DC1 = dc * PWM_MAX;
}

void setDCExtra(float dc){
    P1DC3 = dc * PWM_MAX;
}

