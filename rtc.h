#include <Arduino.h>
#include <Wire.h>

/**
 * https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
 */

struct bcdTime {
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hour;
};

#define DS3231 0b1101000

#define SECONDS 0x00
#define MINUTES 0x01
#define HOURS 0x02
#define WEEKDAY 0x02
#define DAY 0x03
#define MONTH 0x05
#define YEAR 0x06

#define ALARM1_SECONDS 0x07
#define ALARM1_MINUTES 0x08
#define ALARM1_HOUR 0x09
#define ALARM1_DAY 0x0A

#define ALARM2_MINUTES 0x0B
#define ALARM2_HOUR 0x0C
#define ALARM2_DAY 0x0D

#define CONTROL 0x0E // Control Register (0Eh)
#define STATUS 0x0F // Status Register (0Fh)

#define AGING 0x10 // Aging Offset (10h)
#define TEMP_MSB 0x11 // Temperature Register (Upper Byte) (11h)
#define TEMP_LSB 0x12 // Temperature Register (Lower Byte) (12h)

static int8_t lastError = 0; // -1 read error

static bool rtcBegin() {
  Wire.begin();

  // Test transmission
  Wire.beginTransmission(DS3231);
  return Wire.endTransmission() == 0;
}

static uint8_t rtcReadRegister(uint8_t reg) {
  Wire.beginTransmission(DS3231);
  Wire.write(reg);
  Wire.endTransmission();

  if (Wire.requestFrom(DS3231, 1) == 1)
    return Wire.read();

  lastError = -1;
  return 0;
}

static uint8_t rtcWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(DS3231);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static void rtcReadTime(bcdTime *const time) {
  Wire.beginTransmission(DS3231);
  Wire.write(0x00); // Start of timekeeping registers
  Wire.endTransmission();

  if(Wire.requestFrom(DS3231, sizeof(*time)) == sizeof(*time)) {
    Wire.readBytes((uint8_t *)time, sizeof(*time));
    return;
  }

  lastError = -1;
}

static void rtcWriteTime(const bcdTime *const time) {
  Wire.beginTransmission(DS3231);
  Wire.write(0x00); // Start of timekeeping registers
  Wire.write((uint8_t *)time, sizeof(*time));
  Wire.endTransmission();

  uint8_t statusRegister = rtcReadRegister(STATUS);
  statusRegister &= ~0b10001000; // Clear OSF (Start oscillator), EN32kHz (Disable 32kHz Output)
  rtcWriteRegister(STATUS, statusRegister);
}

static bool rtcLostTime() {
  return rtcReadRegister(STATUS) >> 7; // Bit 7: Oscillator Stop Flag (OSF)
}

static void rtcEnable1Hz() {
  uint8_t controlRegister = rtcReadRegister(CONTROL);
  controlRegister &= ~0b00011100; // Clear RS2, RS1, INTCN (Enable 1Hz square wave)
  rtcWriteRegister(CONTROL, controlRegister);
}
