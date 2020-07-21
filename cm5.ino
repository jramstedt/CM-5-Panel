#pragma GCC optimize("O3")

#include <Arduino.h>
#include <assert.h>

#include <wiring_private.h>
static void turnOffPWM(uint8_t timer);

#define CLONE_PANEL

#define M_A 2
#define M_B 3
#define M_C 4
#define M_D 5

#define M_R 7

#define M_OE 6
#define M_STR 8
#define M_SCK 9

#define MODE_BUTTON 10

#pragma region pin struct
struct pin {
  volatile uint8_t *out;
  volatile uint8_t *in;
  uint8_t bit;
};

#define SET_TO(p, x) (*p.out ^= (-(x) ^ *p.out) & p.bit)
#define SET_HIGH(p) (*p.out |= p.bit)
#define SET_LOW(p) (*p.out &= ~p.bit)
#define IS_HIGH(p) ((*p.in & p.bit) ? true : false)
#define IS_LOW(p) ((*p.in & p.bit) ? false : true)

pin setupPin(uint8_t pinNumber, uint8_t pinMode = OUTPUT) {
  uint8_t timer = digitalPinToTimer(pinNumber);
  if (timer != NOT_ON_TIMER) turnOffPWM(timer);

  uint8_t port = digitalPinToPort(pinNumber);
  assert(port != NOT_A_PIN);

  uint8_t bit = digitalPinToBitMask(pinNumber);

  volatile uint8_t *out = portOutputRegister(port);
  volatile uint8_t *in = portInputRegister(port);
  volatile uint8_t *reg = portModeRegister(port);

  if (pinMode == INPUT) {
    *reg &= ~bit;
    *out &= ~bit;
  } else if (pinMode == INPUT_PULLUP) {
    *reg &= ~bit;
    *out |= bit;
  } else { // Set as output
    *reg |= bit;
  }

  return (pin) {
      .out = out,
      .in = in,
      .bit = bit
  };
}
#pragma endregion

pin pinA, pinB, pinC, pinD, pinR, pinOE, pinSTR, pinSCK;
pin pinBtn;

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

uint16_t rnum = RNUM_SEED;

#pragma endregion

uint8_t mode = 7;

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

  pinBtn = setupPin(MODE_BUTTON, INPUT_PULLUP);

  SET_LOW(pinA);
  SET_LOW(pinB);
  SET_LOW(pinC);
  SET_LOW(pinD);
  SET_HIGH(pinR);
  SET_HIGH(pinOE);
  SET_LOW(pinSTR);
  SET_LOW(pinSCK);

  // 16000000 hz / 128 / 16 / 65 = 120,19 Hz
  TCCR2A = 1 << WGM21;                        // CTC
  TCCR2B = 1 << CS22 | 0 << CS21 | 1 << CS20; // Clock / 128
  TCNT2 = 0;                                  // Clear counter
  OCR2A = 65;                                 // Compare value
  TIMSK2 = 1 << OCIE2A;                       // TIMER2_COMPA_vect

  reset();
}

static uint16_t get_random_bit(void) {
  /* https://en.wikipedia.org/wiki/Linear_feedback_shift_register
   * Primitive polynomial: x^16 + x^15 + x^13 + x^4 + 1
   */
  uint16_t lfsr_bit = ((rnum >> 0) ^ (rnum >> 1) ^ (rnum >> 3) ^ (rnum >> 12)) & 1;

  uint16_t rand_bit = (rnum | (rnum >> 2)) & 1;

  rnum = (lfsr_bit << 15) | (rnum >> 1);

  return rand_bit;
}

void reset() {
#define VIRTUAL_PANEL_COUNT (NUM_DATA_ROWS / NUM_ROWS)

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

  if (IS_LOW(pinBtn)) {
    mode = mode == 7 ? 5 : 7;
    return reset();
  }

  if (mode == 5) {
    #define HALF_NUM_DATA_ROWS (NUM_DATA_ROWS >> 1)
    for (uint8_t row = 0; row < HALF_NUM_DATA_ROWS; row++) {
      for (uint8_t column = 0; column < 16; column++) {
        uint16_t bit_lower = get_random_bit();
        uint16_t bit_upper = get_random_bit();

        rows[row] <<= 1;
        rows[row] |= bit_upper;

        rows[row + HALF_NUM_DATA_ROWS] <<= 1;
        rows[row + HALF_NUM_DATA_ROWS] |= bit_lower;
      }
    }
  } else {
    for (int8_t row = NUM_DATA_ROWS - 1; row >= 0; --row) {
      uint16_t bit = get_random_bit();

      if (row & 4) // Shift in groups of four rows
        rows[row] = (bit << 15) | (rows[row] >> 1);
      else
        rows[row] = (rows[row] << 1) | bit;
    }
  }

  while (millis() < nextLoop)
    yield();
}

/*
 *  CM5 data is horizontal rows
 *  led panel is vertical rows
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

ISR(TIMER2_COMPA_vect)
{
  writeRows();
}

static void turnOffPWM(uint8_t timer)
{
  switch (timer)
  {
#if defined(TCCR1A) && defined(COM1A1)
  case TIMER1A:
    cbi(TCCR1A, COM1A1);
    break;
#endif
#if defined(TCCR1A) && defined(COM1B1)
  case TIMER1B:
    cbi(TCCR1A, COM1B1);
    break;
#endif
#if defined(TCCR1A) && defined(COM1C1)
  case TIMER1C:
    cbi(TCCR1A, COM1C1);
    break;
#endif

#if defined(TCCR2) && defined(COM21)
  case TIMER2:
    cbi(TCCR2, COM21);
    break;
#endif

#if defined(TCCR0A) && defined(COM0A1)
  case TIMER0A:
    cbi(TCCR0A, COM0A1);
    break;
#endif

#if defined(TCCR0A) && defined(COM0B1)
  case TIMER0B:
    cbi(TCCR0A, COM0B1);
    break;
#endif
#if defined(TCCR2A) && defined(COM2A1)
  case TIMER2A:
    cbi(TCCR2A, COM2A1);
    break;
#endif
#if defined(TCCR2A) && defined(COM2B1)
  case TIMER2B:
    cbi(TCCR2A, COM2B1);
    break;
#endif

#if defined(TCCR3A) && defined(COM3A1)
  case TIMER3A:
    cbi(TCCR3A, COM3A1);
    break;
#endif
#if defined(TCCR3A) && defined(COM3B1)
  case TIMER3B:
    cbi(TCCR3A, COM3B1);
    break;
#endif
#if defined(TCCR3A) && defined(COM3C1)
  case TIMER3C:
    cbi(TCCR3A, COM3C1);
    break;
#endif

#if defined(TCCR4A) && defined(COM4A1)
  case TIMER4A:
    cbi(TCCR4A, COM4A1);
    break;
#endif
#if defined(TCCR4A) && defined(COM4B1)
  case TIMER4B:
    cbi(TCCR4A, COM4B1);
    break;
#endif
#if defined(TCCR4A) && defined(COM4C1)
  case TIMER4C:
    cbi(TCCR4A, COM4C1);
    break;
#endif
#if defined(TCCR4C) && defined(COM4D1)
  case TIMER4D:
    cbi(TCCR4C, COM4D1);
    break;
#endif

#if defined(TCCR5A)
  case TIMER5A:
    cbi(TCCR5A, COM5A1);
    break;
  case TIMER5B:
    cbi(TCCR5A, COM5B1);
    break;
  case TIMER5C:
    cbi(TCCR5A, COM5C1);
    break;
#endif
  }
}