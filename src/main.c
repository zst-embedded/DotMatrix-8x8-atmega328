/**
 * Column
 * - source current (+)
 * - PD0 - PD7
 * - left to right column-pin-1 ~ column-pin-8
 * - currently on screen is left to right, need to reverse order
 *
 * Row (serial side)
 * - sink current through resistor (-)
 * - PB0 - PB7
 * - left to right row-pin-1 ~ row-pin-8
 *
 * With serial at right
 * Current setup,
 * x-select/col = Left to right
 * y-select/row = top to bottom
 *
 *
 * Buttons (Active LOW)
 * - PC0
 * - PC1
 * - PC5 - Disable display
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
//#include "zst-avr-lib.h"
#include "font5x8.h"

// define ports
#define DDR_ROW DDRB
#define PORT_ROW PORTB
#define DDR_COL DDRD
#define PORT_COL PORTD

// Multiplexing
volatile uint8_t mux_row = 0;
volatile uint8_t mux_colData[8];

// Time
volatile uint8_t time_digit[6];
volatile uint8_t hours = 12, minutes = 30, seconds = 00;
volatile uint8_t hoursOld = 98, minutesOld = 98;
/*#define forceUpdateTime() do { \
    hoursOld = 99;\
    minutesOld = 99;\
} while(0)
// to force an update, set minutesOld to 99*/

// Text Clock
#define MOVEMENT_WIDTH 5    // LED width of the chars
#define MOVEMENT_SPEED 100  // Delay between frames in msec

// Interupt and buttons
void (*interruptFunction)(void) = 0;
bool turnOffDisplay = false;
bool timeChangerButtonIncrement, timeChangerButtonDecrement, timeChangerButtonMode;

// Function prototypes
void helloWorldTest(void);
void dotTest(void);
void binaryClock(void);
void textClock(void);
void timeChanger(void);
void forceUpdateClock(void);
void showText(const char input[]);


uint8_t reverse(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

int main(void) {
    DDR_COL = 0xFF;
    DDR_ROW = 0xFF;

    /* Multiplexing: Set up Timer 0 CTC*/
    // Set CTC compare value @ 256 prescale & 8MHz
    //     (8_000_000 / 256) * 1 - 1 = 31250 - 1
    //     1/31250 sec = 0.000032 seconds per increment
    //     31250 * 2 * 10^-3 = 62.5 --> for 2 msec
    OCR0A = 75;
    TIMSK0 |= _BV(OCIE0A); // Set Timer 0 Compare Match A Interupt
    TCCR0A |= _BV(WGM01); // Set Timer 0 to Mode 2: CTC on OCR0A
    TCCR0B |= _BV(CS02); // Set Prescaler to 256 and start timer

    /* Clock: Set up Timer 1 CTC*/
    OCR1A = 31249; // Set CTC compare value to 1sec at 8MHz internal @ 256 pre-scale: (8000000 / 256) * 1  - 1
    TIMSK1 |= _BV(OCIE1A); // Set Timer 1 Compare Match A Interupt
    TCCR1B |= _BV(WGM12); // Set Timer 1 to Mode 4: CTC on OCR1A
    TCCR1B |= _BV(CS12); // Set Prescaler to 256 and start timer

    /* Buttons: Pin Change Interupt*/
    DDRC &= ~( _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB5) ); // PC0-5 as input
    PCICR |= _BV(PCIE1); // set PCIE1 / PCINT[14:8] to enable PCMSK0 scan
    PCMSK1 |= _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) |_BV(PCINT13); // set PCINT8/9 to trigger an interrupt on state change
    PORTC |= ( _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB5) ); // Internal Pull-Up Resistor

    /* Power Reduction: */
    PRR |= (1<<PRTWI) | (1<<PRTIM2) | (1<<PRSPI) | (1<<PRUSART0)| (1<<PRADC);
    MCUSR = 0;
    wdt_disable();

    sei();

    for(;;) {
        if (interruptFunction) { // If pointer to a function exists (!= 0)
            interruptFunction();
            interruptFunction = 0;
        }
        binaryClock();
        _delay_ms(498);
    }
    return 0;
}

ISR (TIMER0_COMPA_vect) {
    if (turnOffDisplay && interruptFunction == 0) {
        PORT_ROW = ~0;
        PORT_COL = 0;
        return;
    }

    PORT_ROW = ~_BV(mux_row);
    PORT_COL = mux_colData[mux_row];
    mux_row++;
    if (mux_row >= 8)
        mux_row = 0;
}

ISR (TIMER1_COMPA_vect) {
    seconds++;
    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) hours = 0;
        }
    }

    // Always update seconds because it always changes.
    time_digit[4] = seconds / 10;
    time_digit[5] = seconds % 10;

    if (minutesOld != minutes) {
        time_digit[2] = minutes / 10;
        time_digit[3] = minutes % 10;

        // if minutes is the same, hour should be the same
        if (hoursOld != hours) {
            time_digit[0] = hours / 10;
            time_digit[1] = hours % 10;
        }
    }

    hoursOld = hours;
    minutesOld = minutes;
}

void forceUpdateClock() {
    if (minutes >= 60) minutes = 0; // without this, in timeChanger, the minutes can exceed 60
    if (hours >= 24) hours = 0; // without this, in timeChanger, the hours can exceed 60
    time_digit[0] = hours / 10;
    time_digit[1] = hours % 10;
    time_digit[2] = minutes / 10;
    time_digit[3] = minutes % 10;
    time_digit[4] = seconds / 10;
    time_digit[5] = seconds % 10;
    hoursOld = hours;
    minutesOld = minutes;
}

ISR (PCINT1_vect) {
    if (interruptFunction == &timeChanger) {
        if ((PINC & (1 << PC0)) == 0) timeChangerButtonIncrement = true;
        if ((PINC & (1 << PC5)) == 0) timeChangerButtonDecrement = true;
        if ((PINC & (1 << PC1)) == 0) timeChangerButtonMode = true;
        return;
    }

    //TODO turnOffDisplay = ( (PINC & (1 << PC2)) == 0 );

    if ( (PINC & (1 << PC0)) == 0) {
        interruptFunction = &textClock;
    }
    if ( (PINC & (1 << PC1)) == 0) {
        interruptFunction = &timeChanger;
    }
    if ( (PINC & (1 << PC5)) == 0 ) {
        interruptFunction = &dotTest;
    }

}

void dotTest() {
    for (uint8_t select_row = 0; select_row < 8; select_row++) {
        for (uint8_t col = 0; col < 8; col++) {
            for (uint8_t row = 0; row < 8; row++) {
                mux_colData[row] = select_row == row ? _BV(col) : 0;
            }
            _delay_ms(50);
        }
    }
}

void textClock() {
    // http://stackoverflow.com/questions/10820377/c-format-char-array-like-printf
    char input[12];
    sprintf(input, " %02d:%02d:%02d", hours, minutes, seconds);
    showText(input);
}

void showText(const char input[]) {
    uint8_t a, b, x;
    uint8_t eachChar, movement;
    for (eachChar = 0; input[eachChar] != '\0'; eachChar++) { // FOR EACH CHAR
        for (movement = 0; movement < MOVEMENT_WIDTH; movement++) {
            for (x = 0; x < 8; x++) { // MULTIPLEX 8 ROWS
                a = pgm_read_byte_near(font + input[eachChar] * 8 + x);
                b = pgm_read_byte_near(font + input[eachChar + 1] * 8 + x);
                a <<= movement; // shift after reading the word into `a` because we want to clip off top 8 bits of the 16 bit word we read.
                b >>= (MOVEMENT_WIDTH - movement);
                //mux_colData[7-x] = a | b;
                mux_colData[x] = reverse(a | b);
            }
            _delay_ms(MOVEMENT_SPEED);
        }
    }
}

void timeChanger() {
    // (PINC & (1 << PC1) // mode changer
    // (PINC & (1 << PC0) // time increment

    uint8_t flashingFrame = 0U;
    uint8_t mode = 0, col;
    bool buttonIncrement, buttonDecrement, buttonMode;
    while (1) {
        flashingFrame = ~flashingFrame;
        for (col = 0; col < 8; col ++)
            mux_colData[col] = flashingFrame;

        buttonIncrement = (PINC & (1 << PC0)) == 0 || timeChangerButtonIncrement;
        buttonDecrement = (PINC & (1 << PC5)) == 0 || timeChangerButtonDecrement;
        buttonMode      = (PINC & (1 << PC1)) == 0 || timeChangerButtonMode;
        timeChangerButtonIncrement = timeChangerButtonDecrement = timeChangerButtonMode = false;
        // reset it so we don't go into an infinite increment
        // check booleans in addition to pin. We may "miss" the button press during the msec delays;

        if (mode == 0) {
            // Hour Change
            mux_colData[0] = time_digit[0];
            mux_colData[1] = time_digit[1];
            mux_colData[2] = 0;
            if (buttonIncrement) hours++;
            else if (buttonDecrement) {
                if (hours == 0) hours = 23;
                else hours--;
            }
        } else if (mode == 1) {
            // Min Change
            mux_colData[2] = 0;
            mux_colData[3] = time_digit[2];
            mux_colData[4] = time_digit[3];
            mux_colData[5] = 0;
            if (buttonIncrement) minutes++;
            else if (buttonDecrement) {
                if (minutes == 0) minutes = 59;
                else minutes--;
            }
        } else if (mode == 2) {
            // Sec Reset
            mux_colData[5] = 0;
            mux_colData[6] = time_digit[4];
            mux_colData[7] = time_digit[5];
            if (buttonIncrement) seconds = 0;
            else if (buttonDecrement) seconds = 30;
        } else {
            return;
        }

        if (buttonIncrement || buttonDecrement) {
            forceUpdateClock();
        }

        _delay_ms(250);
        if (buttonMode) { // change mode
            mode++;
            _delay_ms(500);
        }
        _delay_ms(250);
    }
}



void binaryClock() {
    if (turnOffDisplay) return;
    // hour
    mux_colData[0] = time_digit[0];
    mux_colData[1] = time_digit[1];
    // min
    mux_colData[3] = time_digit[2];
    mux_colData[4] = time_digit[3];
    // sec
    mux_colData[6] = time_digit[4];
    mux_colData[7] = time_digit[5];

    mux_colData[2] = mux_colData[5] = 0;
    // reset these rows, else previous data will still remain on screen.
}


//#define DEBUG
#ifdef DEBUG

void helloWorldTest() {
    showText(" HELLO WORLD! ");
}

void diagonalTest() {
    for (uint8_t h = 0; h < 8; h++) {
        for (uint8_t x = 0; x < 8; x++) {
            mux_colData[x] = _BV((h+x)%8);
        }
        _delay_ms(500);
    }
}


#endif


/*
 * shiftOutLatch(
                        &PORTD, PD5, //SR-PIN14
                        &PORTD, PD6, //SR-PIN11
                        &PORTD, PD7, //SR-PIN12
                        0,
                        a | b
                );

        */

/**
 * Column
 * - source current (+)
 * - Shift register
 * - PD5 (11) data
 * - PD6 (12) clk
 * - PD7 (13) latch
 * - away from serial
 * - left to right column-pin-1 ~ column-pin-8
 *
 * Row
 * - sink current through resistor (-)
 * - PB0 - PB7
 * - left to right row-pin-1 ~ row-pin-8
 *
 *
 */