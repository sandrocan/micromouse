 #include <xc.h>
 #include "IOconfig.h"

 void setupIO() {

    //ANALOG PINS: A0, A1, A8, rest digital
    AD1PCFGL=0xFEFC;

    //LEDs
    TRISCbits.TRISC6 = 0;
    TRISAbits.TRISA10 = 0;
    TRISAbits.TRISA7 = 0;    

    //Motor Left (PWM, ENC, INPUT)
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
    TRISBbits.TRISB14 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;

    //Motor Right (PWM, ENC, INPUT)
    TRISCbits.TRISC8 = 1;
    TRISCbits.TRISC9 = 1;
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;    

    //Sensoren
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISCbits.TRISC2 = 1;

    //UART
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 1;

    //SELFDESTRUCT
    TRISBbits.TRISB8 = 1;

    /// PIN MAPPING
    // Unlock OSCCON
    __builtin_write_OSCCONL(OSCCON & 0xbf);

    RPINR18bits.U1RXR = 3; //RP3 is U1 RX
    RPOR1bits.RP2R = 0b00011; //RP2 is U1 TX

    //PERIPHERAL Motor Left Encoder A
    RPINR14bits.QEA1R = 16; 
    //PERIPHERAL Motor Left Encoder B
    RPINR14bits.QEB1R = 17;

    //PERIPHERAL Motor Right Encoder A
    RPINR16bits.QEA2R = 24; 
    //PERIPHERAL Motor Right Encoder B
    RPINR16bits.QEB2R = 25;

    //lock OSCCON
     __builtin_write_OSCCONL(OSCCON | 0x40); 

    //Short **DIRTY** Delay nach Lenz'scher Art
    for (volatile int i=0; i<30000; i++);
 }