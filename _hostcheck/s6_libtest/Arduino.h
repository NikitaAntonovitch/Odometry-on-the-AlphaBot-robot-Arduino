// Host mock Arduino core for LAB2 stage6 tests (no hardware).
// Provides: virtual clock, pin model, TLC1543 protocol simulator (so the REAL
// TRSensors.cpp bit-bang code runs unmodified), Serial capture, EEPROM array.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <deque>

// ---------- virtual clock ----------
inline unsigned long g_ms = 0;
inline unsigned long g_serialAutoAdvanceMs = 0; // set to 1 around 'g' countdown tests
inline unsigned long g_millisQueries = 0;
inline unsigned long millis() {
  g_millisQueries++;
  if (g_serialAutoAdvanceMs) g_ms += g_serialAutoAdvanceMs;
  return g_ms;
}
inline void delay(unsigned long ms) { g_ms += ms; }
inline void delayMicroseconds(unsigned int us) { (void)us; }

// ---------- pins ----------
static const uint8_t A0 = 14, A1 = 15, A2 = 16, A3 = 17;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

inline uint8_t g_pinDir[32] = {0};
inline uint8_t g_pinLevel[32] = {0};
inline uint8_t g_pinPwm[32] = {0};
inline unsigned long g_tlc_passCount = 0;   // CS falls = 6 per AnalogRead

inline void pinMode(uint8_t p, uint8_t m) { if (p < 32) g_pinDir[p] = m; }

// ---------- TLC1543 simulator ----------
// Model: during pass j the chip shifts out the result of the conversion made
// during pass j-1 (one-behind pipeline). CS fall starts a pass, CS rise ends it
// (conversion of the latched address completes -> result ready next pass).
// DataOut delivers 10 bits MSB-first per pass. Result: sensor_values[i] = raw[i].
inline int g_tlc_raw[6] = {0, 0, 0, 0, 0, 0};   // programmed per-channel raw ADC
inline unsigned g_tlc_output = 0;               // result available to shift out
inline unsigned g_tlc_bitIdx = 0;
inline unsigned g_tlc_addrBits = 0;
inline unsigned g_tlc_curAddr = 0;

inline void digitalWrite(uint8_t p, uint8_t v) {
  if (p >= 32) return;
  g_pinLevel[p] = v;
  if (p == 10 && v == LOW) {           // CS fall: start of a pass
    g_tlc_bitIdx = 0; g_tlc_addrBits = 0; g_tlc_curAddr = 0;
    g_tlc_passCount++;
  } else if (p == 10 && v == HIGH) {   // CS rise: conversion of this pass done
    g_tlc_output = (unsigned)g_tlc_raw[g_tlc_curAddr & 0x07];
  } else if (p == 12) {                // address line, MSB first, 4 bits per pass
    if (g_tlc_addrBits < 4) {
      g_tlc_curAddr = (g_tlc_curAddr << 1) | (v ? 1u : 0u);
      g_tlc_addrBits++;
    }
  }
}
inline int digitalRead(uint8_t p) {
  if (p == 11) {                       // DataOut: MSB-first bit of g_tlc_output
    unsigned idx = (g_tlc_bitIdx < 10) ? g_tlc_bitIdx : 9;
    int bit = (int)((g_tlc_output >> (9 - idx)) & 1u);
    if (g_tlc_bitIdx < 10) g_tlc_bitIdx++;
    return bit;
  }
  return (p < 32) ? g_pinLevel[p] : 0;
}
inline void analogWrite(uint8_t p, int v) {
  if (p >= 32) return;
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  g_pinPwm[p] = (uint8_t)v;
  g_pinLevel[p] = v > 0 ? HIGH : LOW;
}

#define F(x) x
inline int min(int a, int b) { return a < b ? a : b; }
inline int max(int a, int b) { return a > b ? a : b; }

// ---------- Serial mock: TX captured, RX queued ----------
struct SerialMock {
  std::string tx;
  std::deque<char> rx;
  void begin(unsigned long) {}
  size_t available() { if (g_serialAutoAdvanceMs) g_ms += g_serialAutoAdvanceMs; return rx.size(); }
  int read() { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return (unsigned char)c; }
  int peek() { return rx.empty() ? -1 : (unsigned char)rx.front(); }
  void print(const char* s) { tx += s; }
  void print(char c) { tx += c; }
  void print(unsigned char v) { char b[16]; sprintf(b, "%u", (unsigned)v); tx += b; }
  void print(signed char v) { char b[16]; sprintf(b, "%d", (int)v); tx += b; }
  void print(int v) { char b[16]; sprintf(b, "%d", v); tx += b; }
  void print(unsigned int v) { char b[16]; sprintf(b, "%u", v); tx += b; }
  void print(long v) { char b[32]; sprintf(b, "%ld", v); tx += b; }
  void print(unsigned long v) { char b[32]; sprintf(b, "%lu", v); tx += b; }
  void print(double v, int digits = 2) { char b[64]; sprintf(b, "%.*f", digits, v); tx += b; }
  void println() { tx += "\r\n"; }
  template <typename T> void println(const T& v) { print(v); tx += "\r\n"; }
  void println(double v, int digits) { print(v, digits); tx += "\r\n"; }
  void flush() {}
};
inline SerialMock Serial;

// ---------- EEPROM mock: 1024-byte array, AVR-faithful type sizes ----------
// On AVR: int/unsigned int = 2 bytes, long/unsigned long/float = 4 bytes.
// The generic templates below use HOST sizes (int=4), which would corrupt the
// sketch's v5 layout on the host only. Specializations keep the mock faithful.
struct EEPROMMock {
  uint8_t mem[1024] = {0};
  template <typename T> T& get(int addr, T& t) { memcpy(&t, mem + addr, sizeof(T)); return t; }
  template <typename T> const T& put(int addr, const T& t) { memcpy(mem + addr, &t, sizeof(T)); return t; }
  // AVR-faithful specializations for 2-byte types
  int& get(int addr, int& t) { uint16_t v; memcpy(&v, mem + addr, 2); t = (int)(int16_t)v; return t; }
  const int& put(int addr, const int& t) { uint16_t v = (uint16_t)(int16_t)t; memcpy(mem + addr, &v, 2); return t; }
  unsigned int& get(int addr, unsigned int& t) { uint16_t v; memcpy(&v, mem + addr, 2); t = (unsigned int)v; return t; }
  const unsigned int& put(int addr, const unsigned int& t) { uint16_t v = (uint16_t)t; memcpy(mem + addr, &v, 2); return t; }
  uint8_t& get(int addr, uint8_t& t) { t = mem[addr]; return t; }
  const uint8_t& put(int addr, const uint8_t& t) { mem[addr] = t; return t; }
  uint8_t read(int addr) { return mem[addr]; }
  void write(int addr, uint8_t v) { mem[addr] = v; }
  void update(int addr, uint8_t v) { mem[addr] = v; }
};
inline EEPROMMock EEPROM;

// ---------- Arduino-style auto-prototypes (the IDE generates these for .ino) ----------
void runCalibration();
void warmupCalibrate();
bool calibrationLoadFromEEPROM();
void calibrationSaveToEEPROM();
void calibrationClearEEPROM();
void pidReset();
int pidStep(int error);
void applyCorrection(int uIn);
int deadBandCompensate(int pwm);
void motorL(int pwm);
void motorR(int pwm);
void driveWheels(int leftSpeed, int rightSpeed);
void motorsStop();
void motorSelfTest();
int readError();
bool lineLost(int e);
bool isCrossing();
void blinkCrossingLed();
void applySearchMovement(int side);
void stateMachineStep(int e);
void recordTick(int16_t t, int16_t e, int16_t u, int8_t st);
void dumpBuffer();
void tickController();
void handleLine(const char* buf);
void readSerialCommands();
void printValues(const char* label, unsigned int* v);
void setup();
void loop();
