#include "uart.h"

static void uartDelayCycles(unsigned long cycles)
{
    volatile unsigned long i;

    for (i = 0; i < cycles; i++) {
        Nop();
    }
}

static void uartDelayMs(unsigned int ms)
{
    while (ms-- > 0U) {
        uartDelayCycles(2000UL);
    }
}

static void flushUartRx(void)
{
    while (U1STAbits.URXDA) {
        (void)U1RXREG;
    }

    U1STAbits.OERR = 0;
}

static int waitForUartResponse(const char *expected, unsigned int timeout_ms)
{
    unsigned int match_index = 0;

    while (timeout_ms-- > 0U) {
        while (U1STAbits.URXDA) {
            char rx = (char)U1RXREG;

            if (rx == expected[match_index]) {
                match_index++;
                if (expected[match_index] == '\0') {
                    return 1;
                }
            } else {
                match_index = (rx == expected[0]) ? 1U : 0U;
            }
        }

        uartDelayMs(1);
    }

    return 0;
}

static int sendBluetoothCommand(const char *command, const char *expected, unsigned int timeout_ms)
{
    flushUartRx();
    writeUART(command);
    return waitForUartResponse(expected, timeout_ms);
}

void setupUART() {

    ///Setup UART with baudrate=115200

    U1MODEbits.UARTEN = 0; //switch the uart off during set-up
    U1BRG = 21; // BRG = Fcycle / (16*115200) -1 ~= 21
    U1MODEbits.LPBACK=0;

    U1MODEbits.WAKE=0; //do not wake up on serial port activity

    U1MODEbits.ABAUD=0; //no auto baud rate detection
    U1MODEbits.PDSEL=0; //select 8 bits data, no parity
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

    U1MODEbits.UARTEN=1; //switch the uart on

    U1STAbits.UTXEN=1; //enable the transmission of data
}


void writeUART(const char *buffer) {
    const char * temp_ptr = buffer;

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

int initBluetooth(void)
{
    unsigned int rx_interrupt_enabled = IEC0bits.U1RXIE;

    IEC0bits.U1RXIE = 0;
    uartDelayMs(500);

    if (!sendBluetoothCommand("$$$", "CMD>", 500)) {
        IEC0bits.U1RXIE = rx_interrupt_enabled;
        return 0;
    }

    if (!sendBluetoothCommand("SS,C0\r", "AOK", 500)) {
        IEC0bits.U1RXIE = rx_interrupt_enabled;
        return 0;
    }

    if (!sendBluetoothCommand("SN,Micromouse\r", "AOK", 500)) {
        IEC0bits.U1RXIE = rx_interrupt_enabled;
        return 0;
    }

    if (!sendBluetoothCommand("R,1\r", "Reboot", 1000)) {
        IEC0bits.U1RXIE = rx_interrupt_enabled;
        return 0;
    }

    uartDelayMs(500);
    IEC0bits.U1RXIE = rx_interrupt_enabled;
    return 1;
}





void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{	
	unsigned int rxData; // a local buffer to copy the data into

	IFS0bits.U1RXIF=0;

	// //we should now read out the data
	// rxData=U1RXREG;
    
    // //and copy it back out to UART
    // U1TXREG=rxData;

	//we should also clear the overflow bit if it has been set (i.e. if we were to slow to read out the fifo)
	U1STAbits.OERR=0; //we reset it all the time
}


void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{	
	IFS0bits.U1TXIF=0;
}
