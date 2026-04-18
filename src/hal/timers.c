#include "timers.h"
#include "IOconfig.h"
#include "adc.h"
#include "controller.h"
#include "motors.h"
#include "uart.h"
#include "pwm.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "explore.h"
#include "selfdestruct.h"

#define TIMER2_ISR_HZ 4000U

enum
{
    MELODY_TICKS_PER_MS = TIMER2_ISR_HZ / 1000U
};

typedef struct
{
    unsigned int frequencyHz;
    unsigned int durationTicks;
} MelodyNote;

static const MelodyNote goalMelody[] = {
    {1047U, 120U * MELODY_TICKS_PER_MS}, // C6
    {1319U, 120U * MELODY_TICKS_PER_MS}, // E6
    {1568U, 140U * MELODY_TICKS_PER_MS}, // G6
    {1319U, 120U * MELODY_TICKS_PER_MS}, // E6
    {1047U, 120U * MELODY_TICKS_PER_MS}, // C6
    {1319U, 120U * MELODY_TICKS_PER_MS}, // E6
    {1568U, 180U * MELODY_TICKS_PER_MS}, // G6
    {0U, 70U * MELODY_TICKS_PER_MS},     // short pause
    {880U, 120U * MELODY_TICKS_PER_MS},  // A5
    {1047U, 120U * MELODY_TICKS_PER_MS}, // C6
    {1319U, 160U * MELODY_TICKS_PER_MS}, // E6
    {1568U, 260U * MELODY_TICKS_PER_MS}, // G6 hold
};

static bool buzzerOutputHigh = false;
static bool goalMelodyActive = false;
static uint8_t goalMelodyIndex = 0;
static unsigned int goalMelodyTicksRemaining = 0;
static unsigned int buzzerPhaseAccumulator = 0;

static void setBuzzerOutput(bool enabled)
{
    activateBuzzer(enabled ? 1.0f : 0.0f);
    buzzerOutputHigh = enabled;
}

static void loadGoalMelodyNote(uint8_t noteIndex)
{
    if (noteIndex >= (sizeof(goalMelody) / sizeof(goalMelody[0])))
    {
        goalMelodyActive = false;
        setBuzzerOutput(false);
        return;
    }

    goalMelodyIndex = noteIndex;
    goalMelodyTicksRemaining = goalMelody[noteIndex].durationTicks;
    buzzerPhaseAccumulator = 0;
    setBuzzerOutput(false);
}

void startGoalMelody(void)
{
    goalMelodyActive = true;
    loadGoalMelodyNote(0);
}

static void updateGoalMelody(void)
{
    const MelodyNote *note;

    if (!goalMelodyActive)
    {
        setBuzzerOutput(false);
        return;
    }

    note = &goalMelody[goalMelodyIndex];

    if (note->frequencyHz == 0U)
    {
        setBuzzerOutput(false);
    }
    else
    {
        buzzerPhaseAccumulator += (unsigned int)(2U * note->frequencyHz);

        if (buzzerPhaseAccumulator >= TIMER2_ISR_HZ)
        {
            buzzerPhaseAccumulator -= TIMER2_ISR_HZ;
            setBuzzerOutput(!buzzerOutputHigh);
        }
    }

    if (goalMelodyTicksRemaining > 0U)
    {
        goalMelodyTicksRemaining--;
    }

    if (goalMelodyTicksRemaining == 0U)
    {
        loadGoalMelodyNote((uint8_t)(goalMelodyIndex + 1U));
    }
}

static void updateLEDGreenSine(void)
{
    static float phase = 0.0f;
    const float step = 0.06283185f; // 2*pi/100 -> 1 s period at 10 ms interrupt
    float duty_cycle = 0.5f + 0.5f * sinf(phase);

    setDCLEDGreen(duty_cycle);
    phase += step;

    if (phase >= 6.28318531f)
    {
        phase -= 6.28318531f;
    }
}

void initTimer1ms(float timeinms)
{
    T1CON = 0; // Turn all bits off

    // Fcyc = 80.0MHz
    // tcyc = 12.5ns

    // period = prescaler/Fcyc
    // N is the maximum number of ticks: 2¹⁵-1 = 65535
    // The counter cannot exceed this number, so the maximum time reachable is:
    // tmax = N * period * 1000 (the 1000 comes from transforming to ms)
    // tmax = (N * prescaler * 1000) / Fcyc

    // We want to keep the prescaler as low as possible to get numbers as high
    // as possible to get less rounding errors.
    // The four prescalers of the encoder are 1, 8, 64, 256

    float tmax_8 = 5.50;
    float tmax_64 = 51.20;
    float tmax_256 = 208.50;

    int prescaler = 0;

    if (timeinms <= tmax_8)
    {
        T1CONbits.TCKPS = 0b01;
        prescaler = 8;
    }

    else if (timeinms <= tmax_64)
    {
        T1CONbits.TCKPS = 0b10;
        prescaler = 64;
    }

    else if (timeinms <= tmax_256)
    {
        T1CONbits.TCKPS = 0b11;
        prescaler = 256;
    }

    else
    {
        return;
    }

    T1CONbits.TCS = 0;   // select internal FCY clock source
    T1CONbits.TGATE = 0; // gated time accumulation disabled
    TMR1 = 0;
    // PR1 = (80000/prescaler) * timeinms -1;
    PR1 = (uint16_t)(((80000.00f * (float)timeinms) / (float)prescaler) + 0.5f) - 1;
    IFS0bits.T1IF = 0; // reset Timer 1 interrupt flag
    IPC0bits.T1IP = 4; // set Timer1 interrupt priority level to 4
    IEC0bits.T1IE = 1; // enable Timer 1 interrupt
    T1CONbits.TON = 0; // leave timer disabled initially
}

void startTimer1(void)
{
    T1CONbits.TON = 1; //
}

void initTimer2ms(float timeinms)
{
    T2CON = 0; // Turn all bits off

    // Fcyc = 40.0MHz
    // tcyc = 12.5ns

    // period = prescaler/Fcyc
    // N is the maximum number of ticks: 2¹⁵-1 = 65535
    // The counter cannot exceed this number, so the maximum time reachable is:
    // tmax = N * period * 1000 (the 1000 comes from transforming to ms)
    // tmax = (N * prescaler * 1000) / Fcyc

    // We want to keep the prescaler as low as possible to get numbers as high
    // as possible to get less rounding errors.
    // The four prescalers of the encoder are 1, 8, 64, 256

    float tmax_8 = 5.50;
    float tmax_64 = 51.20;
    float tmax_256 = 208.50;
    int prescaler = 1;

    if (timeinms <= 1)
    {
        T2CONbits.TCKPS = 0b00;
    }
    else if (timeinms <= tmax_8)
    {
        T2CONbits.TCKPS = 0b01;
        prescaler = 8;
    }
    else if (timeinms <= tmax_64)
    {
        T2CONbits.TCKPS = 0b10;
        prescaler = 64;
    }
    else if (timeinms <= tmax_256)
    {
        T2CONbits.TCKPS = 0b11;
        prescaler = 256;
    }
    else
    {
        return;
    }

    T2CONbits.TCS = 0;   // select internal FCY clock source
    T2CONbits.TGATE = 0; // gated time accumulation disabled
    TMR2 = 0;
    // PR2 = (80000/prescaler) * timeinms -1;
    PR2 = (uint16_t)(((80000.00f * (float)timeinms) / (float)prescaler) + 0.5f) - 1;
    IFS0bits.T2IF = 0; // reset Timer 2 interrupt flag
    IPC1bits.T2IP = 4; // set Timer2 interrupt priority level to 4
    IEC0bits.T2IE = 1; // enable Timer 2 interrupt
    T2CONbits.TON = 0; // leave timer disabled initially
}

void startTimer2(void)
{
    T2CONbits.TON = 1; //
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; // reset Timer 1 interrupt flag

    if (isStarted())
    {
        explore_estimateCellCenter();
    }

    // unsigned val = readMidSensorValue();
    // char buf[16];
    // snprintf(buf, sizeof(buf), "Sensor = %d\n", val);
    // writeUART(buf);
    updateController();

    static int blaulicht_cnt = 0;
    blaulicht_cnt++;
    if (blaulicht_cnt >= 15)
    { // 17 * 10ms = 170ms -> ~3 Hz
        blaulicht_cnt = 0;
        if (P1DC3 == 0)
        {
            P1DC3 = PWM_MAX; // PWM1H3 (RB10) HIGH, PWM1L3 (RB11) LOW
        }
        else
        {
            P1DC3 = 0; // PWM1H3 (RB10) LOW, PWM1L3 (RB11) HIGH
        }
    }
}

void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0; // reset Timer 2 interrupt flag

    if (goalMelodyActive)
    {
        updateGoalMelody();
    }
}
