/*  FILE: main.c
 *  Authors: 
 *  
 *  Micromouse
 *  Wintersemester 2025
 */

#ifndef IOCONFIG_H
#define IOCONFIG_H

// #define LED_GREEN LATCbits.LATC6
#define LED_BLUE LATAbits.LATA10
#define LED_RED LATAbits.LATA7

#define LEDON 0
#define LEDOFF 1

#define PWM_LEFT LATBbits.LATB14
#define PWM_RIGHT LATBbits.LATB12

#define MOTOR_LEFT_IN1 LATCbits.LATC4
#define MOTOR_LEFT_IN2 LATCbits.LATC5

#define MOTOR_RIGHT_IN1 LATBbits.LATB5
#define MOTOR_RIGHT_IN2 LATBbits.LATB6

#define UART_TX LATBbits.LATB2

#define UART_RX PORTBbits.RB3


#define ENC_A_RIGHT PORTCbits.RC8
#define ENC_B_RIGHT PORTCbits.RC9

#define ENC_A_LEFT PORTCbits.RC0
#define ENC_B_LEFT PORTCbits.RC1

//defined in dma.h
//#define SENSOR_RIGHT PORTAbits.RA0
//#define SENSOR_MIDDLE PORTAbits.RA1
//#define SENSOR_LEFT PORTCbits.RC2

#define SELFDESTRUCT !PORTBbits.RB8


//#define EXTRA7 LATCbits.LATC7
//#define EXTRA5 LATBbits.LATB10
//#define EXTRA3 LATBbits.LATB11
//#define EXTRA1 LATBbits.LATB13

void setupIO();

#endif  /* IOCONFIG_H */