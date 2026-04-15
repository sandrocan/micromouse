#include "selfdestruct.h"
#include "controller.h"
#include <stdbool.h>

#include "IOconfig.h"

#define SELFDESTRUCT_DEBOUNCE_DELAY_CYCLES (60000U)
#define SELFDESTRUCT_START_DELAY_CYCLES (50000000UL)

static volatile bool started = false;

static void debounceDelay(void)
{
    volatile unsigned int i;

    for (i = 0; i < SELFDESTRUCT_DEBOUNCE_DELAY_CYCLES; i++)
    {
    }
}

void initSelfDestructInterrupt(void)
{
    _CNIE = 0;
    _CNIF = 0;
    _CNIP = 5;

    _CN22PUE = 1;
    _CN22IE = 1;

    // Read the port once so the CN mismatch condition is cleared before enabling the interrupt.
    (void)PORTB;

    _CNIF = 0;
    _CNIE = 1;
}

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void)
{
    bool button_pressed;
    bool was_started;

    debounceDelay();
    button_pressed = SELFDESTRUCT;

    if (button_pressed)
    {
        was_started = started;
        started = !started;

        if (!was_started)
        {
            __delay32(SELFDESTRUCT_START_DELAY_CYCLES);
            driveStraight();
        }
        else
        {
            stopDriveControl();
        }
    }

    (void)PORTB;
    IFS1bits.CNIF = 0;
}

bool isStarted(void)
{
    return started;
}
