/*  FILE: uart.c
 *  Authors: 
 *  
 *  Micromouse
 *  Wintersemester 2025
 */

#ifndef UART_H
#define UART_H

#include <xc.h>

void setupUART(void);
void writeUART(const char* buffer);
int initBluetooth(void);

#endif /* UART_H */
