#ifndef __ZST_LIB__
#define __ZST_LIB__


/*********************************************************************************/
/* MACROS FOR BIT MANIPULATION  */
#ifndef _BV
  #define _BV(x) (1UL << (x))
#endif

#define sbi(value, bit) ((value) |= _BV(bit))
#define cbi(value, bit) ((value) &= ~_BV(bit))

#define readBit(value, bit) (((value) >> (bit)) & 0x01)
#define toggleBit(value, bit) ((value) ^= _BV(bit))
#define writeBit(value, bit, bitvalue) (bitvalue ? sbi(value, bit) : cbi(value, bit))


/*********************************************************************************/
/* MACROS FOR AVR  */
#ifndef _NOP // avr-libc defines _NOP() since 1.6.2
    #define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif


/*********************************************************************************/
/* FUNCTIONS FOR AVR  */

uint8_t flipu4bit(const uint8_t c) {
    if (c == 0) return c;
    return (c & 0b0001) << 3 |
           (c & 0b0010) << 1 |
           (c & 0b0100) >> 1 |
           (c & 0b1000) >> 3;
}


/*********************************************************************************/
/**
 * SHIFT REGISTER HELPER FUNCTION
 *
 * -> Connections (Serial):
 *     dataPin  ->  SER  / DS       = 74HC595 PIN 14 (Data in)
 *     clockPin -> SRCLK / SH_CP    = 74HC595 PIN 11 (Shift register clock)
 *     latchPin ->  RCLK / ST_CP    = 74HC595 PIN 12 (Latch / Storage clock)
 *     bitOrder -> 0 (LSB_FIRST) or 1 (MSB_FIRST)
 *
 * -> Connections (Others):
 *     VCC -> PIN 16
 *     GND -> PIN 8
 *     OE  -> PULL DOWN (Active low)
 *     CLR -> PULL UP
 *
 * -> Reference:
 *     http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_port_pass.html
 *     http://www.ti.com/lit/ds/symlink/sn74hc595.pdf
 *     http://www.nxp.com/documents/data_sheet/74HC_HCT595.pdf
 *     https://www.sparkfun.com/datasheets/IC/SN74HC595.pdf
 */
void shiftOut(volatile uint8_t * dataPort, uint8_t dataPin,
              volatile uint8_t * clockPort, uint8_t clockPin,
              uint8_t bitOrder, uint8_t val) {
    for (uint8_t i = 8; i; i--) {
        if (bitOrder) { // MSB_FIRST
            writeBit(*dataPort, dataPin, (val & _BV(7)) );
            val <<= 1;
        } else { // LSB first
            writeBit(*dataPort, dataPin, (val & 1) );
            val >>= 1;
        }
        sbi(*clockPort, clockPin);
        cbi(*clockPort, clockPin);
    }
}

void shiftOutLatch(volatile uint8_t * dataPort, uint8_t dataPin,
              volatile uint8_t * clockPort, uint8_t clockPin,
              volatile uint8_t * latchPort, uint8_t latchPin,
              uint8_t bitOrder, uint8_t val) {
    cbi(*latchPort, latchPin);
    shiftOut(dataPort, dataPin, clockPort, clockPin, bitOrder, val);
    sbi(*latchPort, latchPin);
}

/*
 * for (uint8_t i = 0b00000001; i < 256; i++) {
            shiftOutLatch(
                    &PORTD, PD5, //SR-PIN14
                    &PORTD, PD6, //SR-PIN11
                    &PORTD, PD7, //SR-PIN12
                    1, i);
            _delay_ms(500);
        }
 */

/*
// From arduino library
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
	uint8_t i;

	for (i = 0; i < 8; i++)  {
		if (bitOrder == LSBFIRST)
			digitalWrite(dataPin, !!(val & (1 << i)));
		else	
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);		
	}
}*/
/*********************************************************************************/


#endif //#ifndef __ZST_LIB__