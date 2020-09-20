#include <Arduino.h>
#include <assert.h>

#include <wiring_private.h>
static void turnOffPWM(uint8_t timer);

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
#define GET(p) (*p.in & p.bit)

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
