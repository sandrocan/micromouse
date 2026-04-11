#include "selfdestruct.h"
#include "controller.h"
#include <stdbool.h>

#include "IOconfig.h"

#define SELFDESTRUCT_DEBOUNCE_DELAY_CYCLES (60000U)

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

    debounceDelay();
    button_pressed = SELFDESTRUCT;

    if (button_pressed)
    {
        started = !started;

        if (started)
        {
            setDriveSpeedMmps(600);
        }
        else
        {
            setDriveSpeedMmps(0);
        }
    }

    (void)PORTB;
    IFS1bits.CNIF = 0;
}

bool isStarted(void)
{
    return started;
}
