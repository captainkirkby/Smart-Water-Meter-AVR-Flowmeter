/*
 * Frequency Counter
 * Counts at 16MHz with a resolution of 62.5ns
 * Displays result on an LCD
 *
 * Author: Dylan Kirkby
 * Date: 12/2/16
 */

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "debug.h"

/********************************************
 *  Change these values to change operation *
 ********************************************/

#define UPDATES_PER_MINUTE (100)
#define UART_BAUD_RATE (9600)
#define DEBUG LEVEL_0

/********************************************
 *  Utility #define statements              *
 ********************************************/

#define CYCLES_PER_OVERFLOW (1UL << 16)
#define MILIHERTZ_PER_CYCLE (16000000000ULL)
#define STR_LEN (50)
#define SEC_PER_MIN (60)
#define TOV_PER_SEC (244)
#define DISPLAY_MUX_CONSTANT ((UPDATES_PER_MINUTE / SEC_PER_MIN) * TOV_PER_SEC)

/********************************************
 *  Calibration *
 ********************************************/

// See spreadsheet for experimental Hz to GPH determination
// https://tinyurl.com/zdstpnf
#define MILLIHZ_TO_MILLIGPH (6.109 * millihertz + 0.000128)

typedef enum {
    WAITING,
    COUNTING,
    DISPLAYING
} State;


/********************************************
 *  Globals                                 *
 ********************************************/

// Volatile shared memory
volatile uint16_t displayMuxCount;
volatile uint32_t overflows;
volatile uint32_t cyclesElapsed;
volatile State state;

// Nonvolatile globals
char displayString[STR_LEN];
uint8_t charsPrinted;

/********************************************
 *  Function Prototypes                     *
 ********************************************/

void init(void);
void initInputs(void);
void initINT0(void);
void initTimer(void);
void loop(void);

/********************************************
 *  Function Definitions                    *
 ********************************************/

int main(void) {
    init();
    
    while(1) {
        loop();
    }
}

void init(void) {
    initInputs();
    initINT0();
    initTimer();
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU)); 
    if (DEBUG != LEVEL_0) {
        uart_puts("Flow Meter\nDylan Kirkby\n\n");
    }
    displayMuxCount = 0;
    sei();
}

void initInputs(void) {
    // Set PD2 (INT0 pin) as an input (no pull-up)
    DDRD &= ~(1 << PD2);

    // Set PD3 as an output
    DDRD |= (1 << PD3);
}

void initINT0(void) {
    // Rising edge of INT0 (PD2) generates interrupt request
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);     // EICRA = 0bXXXXXX10

    // Enable INT0 Interrupt
    EIMSK |= (1 << INT0);       // EIMSK = 0bXXXXXXX1
}

void initTimer(void) {
    // Set Prescaler to 1
    TCCR1B |= (1 << CS10);
    TCCR1B &= ~(1 << CS12);     // TCCR1B = 0bXXXXX0X1

    // Enable Overflow Interrupt
    TIMSK1 |= (1 << TOIE1);
}

// 16000000 cycles = 1.000 Hz = 1,000 mHz
// 1600000 cycles = 10.000 Hz = 10,000 mHz
// 160000 cycles = 100.000 Hz = 100,000 mHz
// 16000 cycles = 1000.000 Hz = 1000,000 mHz
// frequency (mHz) = 16,000,000,000 / cycles 

void loop(void) {
    State nextState;
    uint32_t millihertz;
    uint32_t milliGPH;
    uint32_t integerPart;
    uint32_t fractionalPart;

    if (state == DISPLAYING) {
        // Display Data
        millihertz = MILIHERTZ_PER_CYCLE / cyclesElapsed;

        milliGPH = MILLIHZ_TO_MILLIGPH;

        if (DEBUG == LEVEL_0 && (uart_getc() != UART_NO_DATA)) {
            // uart_putc(rx);
            integerPart = milliGPH / 1000;
            fractionalPart = milliGPH % 1000;
            sprintf(displayString, "%lu.", integerPart);
            uart_puts(displayString);
            charsPrinted += strlen(displayString);
            sprintf(displayString, "%03lu GPH\n", fractionalPart);
            uart_puts(displayString);
            charsPrinted += strlen(displayString);
        }

        if (!DEBUG == LEVEL_0 && displayMuxCount == DISPLAY_MUX_CONSTANT) {
            integerPart = milliGPH / 1000;
            fractionalPart = milliGPH % 1000;
            sprintf(displayString, "%lu.", integerPart);
            uart_puts(displayString);
            charsPrinted += strlen(displayString);
            sprintf(displayString, "%03lu GPH\n", fractionalPart);
            uart_puts(displayString);
            charsPrinted += strlen(displayString);

            if (DEBUG == LEVEL_2) {
                integerPart = millihertz / 1000;
                fractionalPart = millihertz % 1000;
                sprintf(displayString, "%lu.", integerPart);
                uart_puts(displayString);
                sprintf(displayString, "%03lu Hz\n", fractionalPart);
                uart_puts(displayString);

                sprintf(displayString, "%lu\n", cyclesElapsed);
                uart_puts(displayString);
                sprintf(displayString, "%lu\n", overflows);
                uart_puts(displayString);                   
            }

            // Reset count
            displayMuxCount = 0;
            
        }

        nextState = WAITING;
    }
    else {
        nextState = state;
    }

    state = nextState;
}

/********************************************
 *  Interrupt Service Routines              *
 ********************************************/

// Timer Overflow Interrupt (occurs once every 2^16 cycles = .004096 seconds = 244 times per second)
ISR(TIMER1_OVF_vect) {
    ++overflows;
    // Update the display UPDATES_PER_MINUTE times per minute
    displayMuxCount = (displayMuxCount >= DISPLAY_MUX_CONSTANT) ? displayMuxCount : displayMuxCount + 1;
}

// Falling Edge Interrupt
ISR(INT0_vect) {
    State nextState;
    
    if (state == WAITING) {
        // Got a falling edge, start counting
        // Reset Timer to zero
        TCNT1 = 0;
        // Reset overflow count
        overflows = 0;
        // Reset overflow interrupt flag by writing a one to TOV
        TIFR1 |= (1 << TOV1);

        nextState = COUNTING;

        // Toggle PD3
        PORTD ^= (1 << PD3);
    }
    else if (state == COUNTING) {
        cyclesElapsed = TCNT1 + overflows * CYCLES_PER_OVERFLOW;
        // Got a falling edge, stop counting and display results
        nextState = DISPLAYING;

        // Toggle PD3
        PORTD ^= (1 << PD3);
    }
    else if (state == DISPLAYING) {

    }
    else {
        // Unknown state
        nextState = WAITING;
    }
    
    state = nextState;


}
