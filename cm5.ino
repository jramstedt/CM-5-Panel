#pragma GCC optimize("O3")

#include <Arduino.h>

#include "pin.h"
#include "font.h"
#include "rtc.h"

#define CLONE_PANEL

#define M_A 17
#define M_B 16
#define M_C 15
#define M_D 14

#define M_R 7

#define M_OE 6
#define M_STR 8
#define M_SCK 9

#define ROTARY_BUTTON 10
#define ROTARY_A 11
#define ROTARY_B 12

#define RTC_INT 2

pin pinA, pinB, pinC, pinD, pinR, pinOE, pinSTR, pinSCK;
pin pinBtn, pinRotA, pinRotB;
pin pinRtcInt;

#pragma region CM5 screen

#define NUM_ROWS 32            /* unique rows */
#define NUM_ROWS_DISPLAYED 106 /* total rows in front panel display */

#ifdef CLONE_PANEL
#define NUM_PANELS 4
#define NUM_DATA_ROWS NUM_ROWS
#else
#define NUM_PANELS 4
#define NUM_DATA_ROWS NUM_PANELS * NUM_ROWS
#endif

#define VIRTUAL_PANEL_COUNT (NUM_DATA_ROWS / NUM_ROWS)

#define RNUM_SEED 0xBAD /* :-) */

/* Uninitialized bits, displayed briefly at the start of mode 7
 */
PROGMEM const uint16_t rows_glitch[] = {
    0x8F10, 0x9112, 0x9314, 0x9516, 0x18E9, 0x5899, 0x38D9, 0x78B9,
    0x9F20, 0xA122, 0xA324, 0xA526, 0x14E5, 0x5495, 0x34D5, 0x74B5,
    0xAF30, 0xB132, 0xB334, 0xB536, 0x1CED, 0x5C9D, 0x3CDD, 0x7CBD,
    0xBF40, 0xC142, 0xC344, 0xC546, 0x12E3, 0x5293, 0x32D3, 0x72B3};

/* Note: rows[0] is the top row; most significant bit is at left;
 * a zero bit corresponds to a lit LED
 */
uint16_t rows[NUM_DATA_ROWS];

/* A Galois LFSR requires a different seed to produce the same output
 */
uint16_t rnum = 0x917D; //RNUM_SEED;

#pragma endregion

/*
  The rotary switch settings are as follows:

  (0) freeze leds
  (1) fast copy leds from pm to led panel
  (2) freeze leds
  (3) copy from pm and display interleaved
  (4) freeze leds
  (5) random and pleasing
  (6) freeze leds
  (7) interleaved display of random and pleasing
  (8) freeze leds
  (9) all leds on
  (A) all leds off
  (B) continuous blink leds on then off
  (C) continuous test mode
  (D) display pm loop connnectivity on LEDs
  (E) freeze
  (F) freeze
*/
const uint8_t supportedModes[] = {0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB};

uint16_t modeCounter = 2;
uint8_t mode = supportedModes[modeCounter];
bool rotAPrev = 0;

bcdTime time = (bcdTime) {
  .seconds = 0x11,
  .minutes = 0x11,
  .hour = 0x11
};
bool timeDirty = true;

// the setup function runs once when you press reset or power the board
void setup() {
  pinA = setupPin(M_A);
  pinB = setupPin(M_B);
  pinC = setupPin(M_C);
  pinD = setupPin(M_D);
  pinR = setupPin(M_R);
  pinOE = setupPin(M_OE);
  pinSTR = setupPin(M_STR);
  pinSCK = setupPin(M_SCK);

  pinBtn = setupPin(ROTARY_BUTTON, INPUT_PULLUP);
  pinRotA = setupPin(ROTARY_A, INPUT_PULLUP);
  pinRotB = setupPin(ROTARY_B, INPUT_PULLUP);
  pinRtcInt = setupPin(RTC_INT, INPUT_PULLUP);

  SET_LOW(pinA);
  SET_LOW(pinB);
  SET_LOW(pinC);
  SET_LOW(pinD);
  SET_HIGH(pinR);
  SET_HIGH(pinOE);
  SET_LOW(pinSTR);
  SET_LOW(pinSCK);

  // Enable external interrupt from RTC
  EICRA = 0 << ISC11 | 0 << ISC10 | 1 << ISC01 | 0 << ISC00;  // Falling edge on INT0
  EIMSK = 0 << INT1 | 1 << INT0;                              // INT0_vect

  // 16000000 hz / 128 / 16 / 65 = 120,19 Hz
  TCCR2A = 1 << WGM21;                        // CTC
  TCCR2B = 1 << CS22 | 0 << CS21 | 1 << CS20; // Clock / 128
  TCNT2 = 0;                                  // Clear counter
  OCR2A = 65;                                 // Compare value
  TIMSK2 = 1 << OCIE2A;                       // TIMER2_COMPA_vect

  rotAPrev = GET(pinRotA);

  if(rtcBegin()) {
    rtcEnable1Hz();

    if (rtcLostTime())
      rtcWriteTime(&time);
  }

  reset();
}

/* https://en.wikipedia.org/wiki/Linear_feedback_shift_register
 * Primitive polynomial: x^16 + x^15 + x^13 + x^4 + 1
 */
static uint16_t get_random_bit_galois(void) {
	uint16_t out_bit = rnum & 1;
	uint16_t rand_bit = (rnum | (rnum >> 2)) & 1;

	rnum >>= 1;
	rnum ^= (-out_bit) & 0xD008;

	return rand_bit;
}

void reset() {
  /* Initial state: all but 3 LEDs lit */
  memset(rows, 0, sizeof(rows));
  for (uint8_t panel = 0; panel < VIRTUAL_PANEL_COUNT; ++panel)
    rows[panel * NUM_ROWS] = 0x9400;

  _delay_ms(600);

  for (uint8_t panel = 0; panel < VIRTUAL_PANEL_COUNT; ++panel)
    memcpy_P(rows + (panel * NUM_ROWS), rows_glitch, sizeof(rows_glitch)); // Initialize rows with glitch pattern
}

void loop() {
  unsigned long nextLoop = millis() + 200;

  if (mode == 0x05) { /* "random and pleasing" */
    #define HALF_NUM_DATA_ROWS (NUM_DATA_ROWS >> 1)
    for (uint8_t row = 0; row < HALF_NUM_DATA_ROWS; row++) {
      for (uint8_t column = 0; column < 16; column++) {
        uint16_t bit_lower = get_random_bit_galois();
        uint16_t bit_upper = get_random_bit_galois();

        rows[row] <<= 1;
        rows[row] |= bit_upper;

        rows[row + HALF_NUM_DATA_ROWS] <<= 1;
        rows[row + HALF_NUM_DATA_ROWS] |= bit_lower;
      }
    }
  } else if (mode == 0x07) { /* "interleaved display of random and pleasing" */
    for (int8_t row = NUM_DATA_ROWS - 1; row >= 0; --row) {
      uint16_t bit = get_random_bit_galois();

      if (row & 4) // Shift in groups of four rows
        rows[row] = (bit << 15) | (rows[row] >> 1);
      else
        rows[row] = (rows[row] << 1) | bit;
    }
  } else if (mode == 0x09) { /* "all leds on" */
    memset(rows, 0x00, sizeof(rows));
  } else if (mode == 0x0A) { /* "all leds off" */
    memset(rows, 0xFF, sizeof(rows));
  } else if (mode == 0x0B) { /* "continuous blink leds on then off" */
    uint8_t led = rows[3] & 0x01 ? 0x00 : 0xFF;
    memset(rows, led, sizeof(rows));
  } else {
    // Freeze leds
  }

  while (millis() < nextLoop) {
    int8_t rotaryDelta = 0;

    #pragma region Rotary Encoder
    bool buttonPressed = IS_LOW(pinBtn);
    bool rotA = GET(pinRotA);
    if (rotA == true && rotAPrev == false) {
      if (GET(pinRotB) == true)
        rotaryDelta = 1;
      else
        rotaryDelta = -1;
    }
    rotAPrev = rotA;
    #pragma endregion

    if (rotaryDelta != 0) {
      if (buttonPressed) {
        reset();
      } else {
        modeCounter += rotaryDelta;
        mode = supportedModes[modeCounter % sizeof(supportedModes)];
      }
    }

    if (timeDirty)
      rtcReadTime(&time);

    yield();
  }
}

/*
 *  CM5 data is horizontal rows
 *  led panel is vertical rows
 * 
 *  Writes one single led panel row from CM5 data rows.
 */
__attribute__((optimize("unroll-loops")))
void writeRows() {
  static uint8_t ledRow;

#define cm5column ledRow

  SET_HIGH(pinOE);
  SET_LOW(pinSTR);

  uint16_t columnMask = 1 << cm5column;
  uint8_t cm5panelorigin = 0;
  for (int8_t cm5panel = NUM_PANELS - 1; cm5panel >= 0; --cm5panel) {
#ifndef CLONE_PANEL
    cm5panelorigin = cm5panel * NUM_ROWS;
#endif

    for (int8_t cm5row = 0; cm5row < NUM_ROWS; ++cm5row) { // 32 pixels per matrix panel row
      SET_LOW(pinSCK);
      SET_TO(pinR, (rows[cm5panelorigin | cm5row] & columnMask) != 0);
      SET_HIGH(pinSCK);
    }
  }

  SET_TO(pinA, (ledRow & 0x01) != 0);
  SET_TO(pinB, (ledRow & 0x02) != 0);
  SET_TO(pinC, (ledRow & 0x04) != 0);
  SET_TO(pinD, (ledRow & 0x08) != 0);

  SET_HIGH(pinSTR);
  SET_LOW(pinOE);

  ledRow = ++ledRow & 0x0F;
}

ISR(TIMER2_COMPA_vect) {
  writeRows();
}

ISR(INT0_vect) {
  timeDirty = true;
}
