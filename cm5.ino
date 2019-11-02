#include <Arduino.h>
#include <assert.h>

#define M_A 2
#define M_B 3
#define M_C 4
#define M_D 5

#define M_R 6

#define M_OE 7
#define M_STR 8
#define M_SCK 9

#pragma region pin struct
struct pin {
  volatile uint8_t *out;
  uint8_t bit;
};

#define SET_HIGH(p) ((*p.out) |= (p.bit))
#define SET_LOW(p) ((*p.out) &= (~p.bit))

pin setupPin (uint8_t pinNumber) {
  uint8_t port = digitalPinToPort(pinNumber);
  assert(port != NOT_A_PIN);

  uint8_t bit = digitalPinToBitMask(pinNumber);
  
  volatile uint8_t *out = portOutputRegister(port);
  volatile uint8_t *reg = portModeRegister(port);

  *reg |= bit;  // Set as output

  return (pin){
    .out = out,
    .bit = bit
  };
}
#pragma endregion

pin pinA, pinB, pinC, pinD, pinR, pinOE, pinSTR, pinSCK;

#pragma region CM5 screen

#define NUM_ROWS 32		/* unique rows */
#define NUM_ROWS_DISPLAYED 106	/* total rows in front panel display */

#define RNUM_SEED 0xBAD  /* :-) */

/* Uninitialized bits, displayed briefly at the start of mode 7
 */
PROGMEM const uint16_t rows_glitch[NUM_ROWS] = {
	0x8F10, 0x9112, 0x9314, 0x9516, 0x18E9, 0x5899, 0x38D9, 0x78B9,
	0x9F20, 0xA122, 0xA324, 0xA526, 0x14E5, 0x5495, 0x34D5, 0x74B5,
	0xAF30, 0xB132, 0xB334, 0xB536, 0x1CED, 0x5C9D, 0x3CDD, 0x7CBD,
	0xBF40, 0xC142, 0xC344, 0xC546, 0x12E3, 0x5293, 0x32D3, 0x72B3
};

/* Note: rows[0] is the top row; most significant bit is at left;
 * a zero bit corresponds to a lit LED
 */
uint16_t rows[NUM_ROWS];

uint16_t rnum = RNUM_SEED;

#pragma endregion


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinA = setupPin(M_A);
  pinB = setupPin(M_B);
  pinC = setupPin(M_C);
  pinD = setupPin(M_D);
  pinR = setupPin(M_R);
  pinOE = setupPin(M_OE);
  pinSTR = setupPin(M_STR);
  pinSCK = setupPin(M_SCK);

  SET_LOW(pinA);
  SET_LOW(pinB);
  SET_LOW(pinC);
  SET_LOW(pinD);
  SET_HIGH(pinR);
  SET_HIGH(pinOE);
  SET_LOW(pinSTR);
  SET_LOW(pinSCK);

  TCCR2A = 1 << WGM21; // CTC
  TCCR2B = 1 << CS22; // Clock / 64
  TCNT2  = 0; // Clear counter
  OCR2A = 255; // Compare value
  TIMSK2 = 1 << OCIE2A; // TIMER2_COMPA_vect

  /* Initial state: all but 3 LEDs lit */
  memset(rows, 0, sizeof(rows));
	rows[0] = 0x9400;

  _delay_ms(600);

  /* Initialize rows with glitch pattern */
	memcpy(rows, pgm_read_byte(rows_glitch), sizeof(rows));
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

// the loop function runs over and over again forever
void loop() {
  for (int8_t row = NUM_ROWS - 1; row >= 0; --row) {
    uint16_t bit = get_random_bit();

    if (row & 4) // Shift in groups of four rows
      rows[row] = (bit << 15) | (rows[row] >> 1);
    else
      rows[row] = (rows[row] << 1) | bit;
  }

  _delay_ms(200);
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

  for (uint8_t cm5row = 0; cm5row < NUM_ROWS; ++cm5row) { // 32 pixels per matrix panel row
    SET_LOW(pinSCK);
    if (rows[cm5row] & (1 << cm5column)) SET_HIGH(pinR); else SET_LOW(pinR);
    SET_HIGH(pinSCK);
  }

  if (ledRow & 0x01) SET_HIGH(pinA); else SET_LOW(pinA);
  if (ledRow & 0x02) SET_HIGH(pinB); else SET_LOW(pinB);
  if (ledRow & 0x04) SET_HIGH(pinC); else SET_LOW(pinC);
  if (ledRow & 0x08) SET_HIGH(pinD); else SET_LOW(pinD);
  
  SET_HIGH(pinSTR);
  SET_LOW(pinOE);

  ledRow = ++ledRow & 0x0F;
}

ISR(TIMER2_COMPA_vect)
{
  writeRows();
}
