#include "selfdestruct.h"

#include "IOconfig.h"

#define SELFDESTRUCT_DEBOUNCE_DELAY_CYCLES (60000U)

static volatile unsigned int selfdestruct_press_flag = 0;
static volatile unsigned long selfdestruct_press_count = 0;
static volatile unsigned int selfdestruct_ready_for_press = 0;

static void debounceDelay(void)
{
    volatile unsigned int i;

    for (i = 0; i < SELFDESTRUCT_DEBOUNCE_DELAY_CYCLES; i++) {
    }
}

void initSelfDestructInterrupt(void)
{
    selfdestruct_press_flag = 0;
    selfdestruct_press_count = 0;
    selfdestruct_ready_for_press = 0;

    _CNIE = 0;
    _CNIF = 0;
    _CNIP = 5;

    _CN22PUE = 1;
    _CN22IE = 1;

    // Read the port once so the CN mismatch condition is cleared before enabling the interrupt.
    (void)PORTB;
    selfdestruct_ready_for_press = !SELFDESTRUCT;

    _CNIF = 0;
    _CNIE = 1;
}

unsigned int selfDestructWasPressed(void)
{
    return selfdestruct_press_flag;
}

void clearSelfDestructPressFlag(void)
{
    selfdestruct_press_flag = 0;
}

unsigned long getSelfDestructPressCount(void)
{
    return selfdestruct_press_count;
}

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void)
{
    unsigned int pressed_now;

    IFS1bits.CNIF = 0;
    (void)PORTB;

    debounceDelay();
    pressed_now = SELFDESTRUCT;

    if (!selfdestruct_ready_for_press) {
        if (!pressed_now) {
            selfdestruct_ready_for_press = 1;
        }
        return;
    }

    if (pressed_now) {
        selfdestruct_press_flag = 1;
        selfdestruct_press_count++;
        selfdestruct_ready_for_press = 0;
    }
}
