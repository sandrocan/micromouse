
#include <xc.h>
#include <string.h>
#include "IOconfig.h"
#include "stdbool.h"


/*
*	set-up the serial port
*   here we aim to achieve a data transfer rate of 57.6 kbit/s,
*   based on Fcycle=26.6666Mhz 
*   BaudRate=Fcycle/(16*(BRG+1))
*   ==> BRG=Fcy/(16*BaudRate) - 1 = 26.666Mhz/(16*57600) - 1 = 28.23
*   ==> choose 28 ==> BaudRate= 57.474  kbit/s, which is ~ 1% off.
 * 
 * for standard communication speed of 9600 kbit/s
 * choose 173 (factor 6)
*/
void setupUART1(void)
{
	U1MODEbits.UARTEN=0; //switch the uart off during set-up
	U1BRG=28; // baud rate register
	U1MODEbits.LPBACK=0; // in loopback mode for test! TODO: set to no loop-back (=0) after test 
	
	U1MODEbits.WAKE=0; //do not wake up on serial port activity

	U1MODEbits.ABAUD=0; //no auto baud rate detection
	U1MODEbits.PDSEL=0; //select 8 bits date, no parity
	U1MODEbits.STSEL=0; //one stop bit
    U1MODEbits.BRGH = 0; // No High Speed Mode


	IFS0bits.U1RXIF=0; //reset the receive interrupt flag
	IFS0bits.U1TXIF=0; //reset the transmission interrupt flag
    
	IPC2bits.U1RXIP=3; //set the RX interrupt priority
	IPC3bits.U1TXIP=5; //set the TX interrupt priority

	U1STAbits.URXISEL=0; //generate a receive interrupt as soon as a character has arrived
	U1STAbits.UTXEN=1; //enable the transmission of data

	IEC0bits.U1RXIE=1; //enable the receive interrupt
	IEC0bits.U1TXIE=0; //disable the transmit interrupt

	//FINALLY, 
	U1MODEbits.UARTEN=1; //switch the uart on

  	U1STAbits.UTXEN=1; //enable transmission
	
    
//   	U1MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
//	U1STA  = 0x0440; /* Reset status register and enable TX & RX*/

	
	
}


//volatile bool tx_request = false;
typedef struct {
    const char * ptr;
    unsigned int remaining;
} uart_tx_t;

uart_tx_t uart_tx_state;


void uart_send_string(const char *str) {
    uart_tx_state.ptr = str;
    uart_tx_state.remaining = strlen(str);
    
    if(uart_tx_state.remaining > 0) {
        U1TXREG = *uart_tx_state.ptr++;
        uart_tx_state.remaining--;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{	
	unsigned int rxData; // a local buffer to copy the data into

	/**Set the UART2 receiving interrupt flag to zero*/
 
	IFS0bits.U1RXIF=0;

    //LED4=~LED4;
    // = LEDON;
    //LED5=~LED5;
    //LED7=~LED7;
    //static float idx = 0;
    //setDCPWM1(idx);
    
    //static float t = 0.0;
    //t += 0.001; //wenn interupt intervall 1ms
    //set PWM to sin(2*pi*t) (shiftet, dass (-1,1) in (0,1) geshiftet wird, da setDCPWM1 prozent der Zeit an nimmt)
    //float sin_f = sin(2*3.14159*t);
    //sin_f = (sin_f + 1)/2;
    //setDCPWM1(sin_f);
	


	
	//we should now read out the data
	rxData=U1RXREG;
    
    //and copy it back out to UART
    U1TXREG=rxData;
    //uart_send_string("Test");
        //wait until the character is gone...

	//we should also clear the overflow bit if it has been set (i.e. if we were to slow to read out the fifo)
	U1STAbits.OERR=0; //we reset it all the time
	//some notes on this from the data sheet
	/*
	If the FIFO is full (four characters) and a fifth character is fully received into the UxRSR register,
	the overrun error bit, OERR (UxSTA<1>), will be set. The word in UxRSR will be kept, but further
	transfers to the receive FIFO are inhibited as long as the OERR bit is set. The user must clear
	the OERR bit in software to allow further data to be received.
	If it is desired to keep the data received prior to the overrun, the user should first read all five
	characters, then clear the OERR bit. If the five characters can be discarded, the user can simply
	clear the OERR bit. This effectively resets the receive FIFO and all prior received data is lost.

	The data in the receive FIFO should be read prior to clearing the OERR bit. The
	FIFO is reset when OERR is cleared, which causes all data in the buffer to be lost.
	*/

}


void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{	
	//unsigned int rxData; // a local buffer to copy the data into
   // long i;
	/**Set the UART2 receiving interrupt flag to zero*/
    //if (uart_tx_state.remaining > 0){
    //    U1TXREG = *uart_tx_state.ptr++;
    //    uart_tx_state.remaining--;
    //}
 
	IFS0bits.U1TXIF=0;
    //LED7=~LED7;
}
//TODO Hausaufgabe: Background send: pass string that sends it out in the background (funktion soll sofort returnen)
//Ansatz immer wenn tranmit flag gesetzt neuen byte rein laden

//sscanf um Daten einzulesen zB <75>
//Nur daten lesen wenn '<' kommt







/***************************************************************************
* Function Name     : putsUART1                                            *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer (till NULL character)       *
* Parameters        : unsigned int * address of the string buffer to be    *  
*                     transmitted                                          *
* Return Value      : None                                                 *
***************************************************************************/

void putsUART1(char *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

    if(U1MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer != '\0') 
        {
            while(U1STAbits.UTXBF); /* wait if the buffer is full */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
    else
    {
        while(*temp_ptr != '\0')
        {
            while(U1STAbits.UTXBF);  /* wait if the buffer is full */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
}