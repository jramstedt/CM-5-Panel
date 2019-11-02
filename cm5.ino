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

pin pinA;
pin pinB;
pin pinC;
pin pinD;
pin pinR;
pin pinOE;
pin pinSTR;
pin pinSCK;

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
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);                       // wait for a second
}

uint8_t row;

void writeRow() {
  SET_HIGH(pinOE);
  SET_LOW(pinSTR);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 8; ++j) {
      SET_HIGH(pinSCK);

      if ((row & 0x01) ^ (j & 0x01)) SET_HIGH(pinR); else SET_LOW(pinR);

      SET_LOW(pinSCK);
    }
  }

  if (row & 0x01) SET_HIGH(pinA); else SET_LOW(pinA);
  if (row & 0x02) SET_HIGH(pinB); else SET_LOW(pinB);
  if (row & 0x04) SET_HIGH(pinC); else SET_LOW(pinC);
  if (row & 0x08) SET_HIGH(pinD); else SET_LOW(pinD);
  
  SET_HIGH(pinSTR);
  SET_LOW(pinOE);

  row = ++row & 0x0F;
}

ISR(TIMER2_COMPA_vect)
{
  writeRow();
}
