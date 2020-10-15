#pragma GCC optimize("O3")

#include <Arduino.h>

#include "pin.h"
#include "font.h"
#include "rtc.h"

//#define CLONE_PANEL

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

#define NUM_CLOCK_ROWS 8
#define NUM_ROWS 32            /* unique rows */
#define NUM_ROWS_DISPLAYED 106 /* total rows in front panel display */

#ifdef CLONE_PANEL
#define NUM_PANELS 4
#define NUM_DATA_ROWS NUM_ROWS
#define CLOCK_LINE_OFFSET = 0
#else
#define NUM_PANELS 4
#define NUM_DATA_ROWS (NUM_PANELS * NUM_ROWS)
#define CLOCK_LINE_OFFSET ((NUM_PANELS - 1) * NUM_ROWS)
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
bool buttonPrev = false;
uint32_t lastPressed;

#define UI_SET_MODE 0
#define UI_SET_HOURS 1
#define UI_SET_MINUTES 2
uint8_t uiMode = UI_SET_MODE;

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

  if(rtcBegin()) {
    rtcEnable1Hz();

    if (rtcLostTime())
      rtcWriteTime(&time);
  }

  reset();
  
  rotAPrev = GET(pinRotA) != 0;
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

uint8_t bcd2dec (uint8_t bcd) {
  return (bcd & 0x0F) + ((bcd & 0xF0) >> 4) * 10;
}

uint8_t dec2bcd (uint8_t dec) {
  return ((dec % 10) & 0x0F) | (((dec / 10) << 4) & 0xF0);
}

void drawTime (uint16_t mask = 0xFFFF) {
  uint32_t minLo = pgm_read_dword_near(&font[time.minutes & 0x0F]);
  uint32_t minHi = pgm_read_dword_near(&font[(time.minutes & 0xF0) >> 4]);
  uint32_t hLo = pgm_read_dword_near(&font[time.hour & 0x0F]);
  uint32_t hHi = pgm_read_dword_near(&font[(time.hour & 0xF0) >> 4]);

  // character is 4x8 pixels, four characters MMSS
  for (uint8_t line = 0; line < NUM_CLOCK_ROWS; ++line) {
    // move one column from font (line) to lsb of each byte
    uint32_t minLoCol = (minLo >> line) & 0x01010101;
    uint32_t minHiCol = (minHi >> line) & 0x01010101;
    uint32_t hLoCol = (hLo >> line) & 0x01010101;
    uint32_t hHiCol = (hHi >> line) & 0x01010101;

    // pack lsb from each byte to sequential bits, also mirrors the order, and moves straight to correct place
    minLoCol = (minLoCol >> 24 | minLoCol >> 15 | minLoCol >> 6 | minLoCol << 3) & 0x0000000F;
    minHiCol = (minHiCol >> 20 | minHiCol >> 11 | minHiCol >> 2 | minHiCol << 7) & 0x000000F0;
    hLoCol = (hLoCol >> 16 | hLoCol >> 7 | hLoCol << 2 | hLoCol << 11) & 0x00000F00;
    hHiCol = (hHiCol >> 12 | hHiCol >> 3 | hHiCol << 6 | hHiCol << 15) & 0x0000F000;
    
    rows[line + CLOCK_LINE_OFFSET] = ~((hHiCol | hLoCol | minHiCol | minLoCol) & mask);
  }
}

void loop() {
  unsigned long nextLoop = millis() + 200;

  if (mode == 0x05) { /* "random and pleasing" */
    #define HALF_NUM_DATA_ROWS (NUM_DATA_ROWS >> 1)
    for (uint8_t row = 0; row < HALF_NUM_DATA_ROWS; row++) {
      uint8_t upperRow = row;
      uint8_t lowerRow = row + HALF_NUM_DATA_ROWS;

      bool drawUpper = upperRow < CLOCK_LINE_OFFSET || upperRow >= CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS;
      bool drawLower = lowerRow < CLOCK_LINE_OFFSET || lowerRow >= CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS;

      for (uint8_t column = 0; column < 16; column++) {
        uint16_t bit_lower = get_random_bit_galois();
        uint16_t bit_upper = get_random_bit_galois();

        if (drawUpper) {
          rows[upperRow] <<= 1;
          rows[upperRow] |= bit_upper;
        }

        if (drawLower) {
          rows[lowerRow] <<= 1;
          rows[lowerRow] |= bit_lower;
        }
      }
    }
  } else if (mode == 0x07) { /* "interleaved display of random and pleasing" */
    for (int8_t row = NUM_DATA_ROWS - 1; row >= 0; --row) {
      if (row >= CLOCK_LINE_OFFSET && row < CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS) continue;
      
      uint16_t bit = get_random_bit_galois();

      if (row & 4) // Shift in groups of four rows
        rows[row] = (bit << 15) | (rows[row] >> 1);
      else
        rows[row] = (rows[row] << 1) | bit;
    }
  } else if (mode == 0x09) { /* "all leds on" */
    memset(&rows[0], 0x00, sizeof(*rows) * CLOCK_LINE_OFFSET); // Before Clock
    memset(&rows[CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS], 0x00, sizeof(rows) - sizeof(*rows) * (CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS)); // After Clock
  } else if (mode == 0x0A) { /* "all leds off" */
    memset(&rows[0], 0xFF, sizeof(*rows) * CLOCK_LINE_OFFSET); // Before Clock
    memset(&rows[CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS], 0xFF, sizeof(rows) - sizeof(*rows) * (CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS)); // After Clock
  } else if (mode == 0x0B) { /* "continuous blink leds on then off" */
    uint8_t led = rows[3] & 0x01 ? 0x00 : 0xFF;
    memset(&rows[0], led, sizeof(*rows) * CLOCK_LINE_OFFSET); // Before Clock
    memset(&rows[CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS], led, sizeof(rows) - sizeof(*rows) * (CLOCK_LINE_OFFSET+NUM_CLOCK_ROWS)); // After Clock
  } else {
    // Freeze leds
  }

  while (millis() < nextLoop) {
    int8_t rotaryDelta = 0;

    #pragma region Rotary Encoder
    bool buttonPressed = IS_LOW(pinBtn);
    if (buttonPressed) lastPressed = millis();
    
    bool rotA = GET(pinRotA) != 0;
    if (rotA == true && rotAPrev == false) {
      if (GET(pinRotB) != 0)
        rotaryDelta = 1;
      else
        rotaryDelta = -1;
    }
    #pragma endregion

    if (uiMode == UI_SET_MODE) {
      if (rotaryDelta != 0) {
        if (buttonPressed) {
          reset();
        } else {
          modeCounter += rotaryDelta;
          mode = supportedModes[modeCounter % sizeof(supportedModes)];
        }
      } else if (!buttonPressed && buttonPrev && (millis() - lastPressed) < 200 ) {
        uiMode = UI_SET_HOURS;
      }
    } else if (uiMode == UI_SET_HOURS) {
      if (rotaryDelta != 0) {
        uint8_t hour = (bcd2dec(time.hour) + rotaryDelta) % 24;
        time.hour = dec2bcd(hour);
        rtcWriteTime(&time);
      }

      if (millis() & 0x100)
        drawTime();
      else
        drawTime(0x00FF);

      if (!buttonPressed && buttonPrev) uiMode = UI_SET_MINUTES;

      timeDirty = false; // Don't update time struct!
    } else if (uiMode == UI_SET_MINUTES) {
      if (rotaryDelta != 0) {
        uint8_t minutes = (bcd2dec(time.minutes) + rotaryDelta) % 60;
        time.minutes = dec2bcd(minutes);
        rtcWriteTime(&time);
      }

      if (millis() & 0x100)
        drawTime();
      else
        drawTime(0xFF00);

      if (!buttonPressed && buttonPrev) uiMode = UI_SET_MODE;

      timeDirty = false; // Don't update time struct!
    }

    if (timeDirty) {
      timeDirty = false;
      rtcReadTime(&time);
      drawTime();
    }

    rotAPrev = rotA;
    buttonPrev = buttonPressed;
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
#ifdef CLONE_PANEL
  for (int8_t cm5panel = NUM_PANELS - 1; cm5panel >= 0; --cm5panel) {
#endif
    for (uint8_t cm5row = 0; cm5row < NUM_DATA_ROWS; ++cm5row) {
      SET_LOW(pinSCK);
      SET_TO(pinR, (rows[cm5row] & columnMask) != 0);
      SET_HIGH(pinSCK);
    }
#ifdef CLONE_PANEL
  }
#endif

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
