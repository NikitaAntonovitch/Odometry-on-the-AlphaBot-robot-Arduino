// mock arduino core + virtual alphabot for host-side simulation of the lab1 sketch (not for the robot)
// the sketch compiles UNCHANGED; pin writes are intercepted by the differential-drive model below
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <cstdio>
using std::min; using std::max;

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define RISING 2
#define FALLING 3
#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : 255))

// interrupt control: host is single threaded (isrs run inline inside stepPhysics),
// so these are no-ops; SREG is modelled as a variable so the sketch's save/restore code compiles
inline uint8_t hostSREG = 0;
#define SREG hostSREG
inline void noInterrupts() {}
inline void interrupts() {}

// F() casts the literal so print(const __FlashStringHelper*) can recover it
class __FlashStringHelper;
#define F(x) ((const __FlashStringHelper*)(x))
typedef void (*isr_t)();

namespace sim {
  void           setTime(unsigned long ms);
  unsigned long  now();
  void           stepPhysics(unsigned long dt);
  void           inject(unsigned long atMs, const char* s);
  void           out(const std::string& s);
  void           simOut(const std::string& s);
  void           onPinWrite(uint8_t pin, uint8_t val);
  void           onAnalogWrite(uint8_t pin, int val);
  void           pinPwmSet(uint8_t pin, int val);
  void           pinDigSet(uint8_t pin, int val);
  int            pinLevel(uint8_t pin);
  int            pinPwm(uint8_t pin);
  void           attachIsr(uint8_t intNum, isr_t fn, int mode);
  void           resetRobot();
  void           dumpPose();
}

inline void pinMode(uint8_t, uint8_t) {}
// pin model mirrors the Arduino core: analogWrite takes over the pin, digitalWrite
// HIGH after PWM = full on, LOW = off (this is what coast() relies on)
inline void digitalWrite(uint8_t p, uint8_t v) {
  sim::onPinWrite(p, v ? HIGH : LOW);
  if (p < 32) sim::pinPwmSet(p, v ? 255 : 0);
}
inline int  digitalRead(uint8_t p) { return sim::pinLevel(p); }
inline void analogWrite(uint8_t p, int v) {
  sim::onAnalogWrite(p, v);
  if (p < 32) sim::pinDigSet(p, v > 0 ? HIGH : LOW);
}
inline unsigned long millis() { return sim::now(); }
inline unsigned long micros() { return sim::now() * 1000UL; }
inline void delay(unsigned long ms) { sim::setTime(sim::now() + ms); sim::stepPhysics(ms); }
inline void attachInterrupt(uint8_t intNum, isr_t fn, int mode) { sim::attachIsr(intNum, fn, mode); }
inline void detachInterrupt(uint8_t) {}

struct SerialMock {
  void begin(unsigned long) {}
  int  available();
  int  read();
  int  peek();                       // stage 5: variant-A busy-window lookahead, like HardwareSerial
  float parseFloat();
  void print(const char* s)               { sim::out(s); }
  void print(const __FlashStringHelper* p){ sim::out(reinterpret_cast<const char*>(p)); }
  void print(char c)                      { sim::out(std::string(1, c)); }
  void print(int v)                       { sim::out(std::to_string(v)); }
  void print(unsigned int v)              { sim::out(std::to_string(v)); }
  void print(long v)                      { sim::out(std::to_string(v)); }
  void print(unsigned long v)             { sim::out(std::to_string(v)); }
  void print(double v, int digits = 2)    { char b[40]; snprintf(b, sizeof b, "%.*f", digits, v); sim::out(b); }
  void println()                          { sim::out("\n"); }
  void println(const char* s)             { print(s);     sim::out("\n"); }
  void println(const __FlashStringHelper* p) { print(p);  sim::out("\n"); }
  void println(char c)                    { print(c);     sim::out("\n"); }
  void println(int v)                     { print(v);     sim::out("\n"); }
  void println(unsigned int v)            { print(v);     sim::out("\n"); }
  void println(long v)                    { print(v);     sim::out("\n"); }
  void println(unsigned long v)           { print(v);     sim::out("\n"); }
  void println(double v, int digits = 2)  { print(v, digits); sim::out("\n"); }
  operator bool() { return true; }
};
extern SerialMock Serial;

// ===================== virtual alphabot =====================
namespace sim {

// nominal parameters; the real values get measured at the academy and only constants change
static const double WHEEL_D_MM    = 65.0;   // alphabot wheel diameter, nominal
static const double TRACK_MM      = 160.0;  // wheel track, nominal (stage 4 calibrates the real one)
static const int    SLOTS         = 20;     // encoder disc slots, nominal - count on the real disc!
static const int    EDGES         = 2;      // react to both edges -> N_rev = 2*S
inline int    DEADBAND_PWM  = 75;     // floor-measured under-load dead band (14.10: wheels first rolled at 75)
static const double MAX_MM_S      = 300.0;  // wheel speed at pwm=255
static const double ASYM_R        = 1.06;   // right wheel 6% faster at the same pwm (stage 1 measurement)
static const double NOISE_FRAC    = 0.03;   // speed noise fraction, deterministic pattern
static const double RUNON_COAST_S = 0.35;   // run-on time constant when coasting
static const double RUNON_BRAKE_S = 0.05;   // run-on when braking (winding shorted)
static const long   COAST_MS      = 2000;   // mirror of the sketch's coast window for scenario timing

struct Wheel {
  double x_mm   = 0.0;   // distance travelled (odometry truth)
  double v_mm_s = 0.0;   // current speed
  long   ticks  = 0;     // encoder count (signed, direction from rotation sense)
  double phase  = 0.0;   // fractional ticks accumulated -> edge scheduling
  bool   level  = false; // current signal level to the opto-interrupter
};

// NOTE: all mutable state is `inline` (C++17) so the .ino TU and sim_main TU share ONE copy.
// `static` here would silently give each translation unit its own copy - the sim would read
// a different world than the sketch writes. This exact bug cost an afternoon.
inline Wheel wl, wr;
inline double heading = 0.0;         // rad, pose of the chassis
inline double poseX = 0.0, poseY = 0.0;
inline double lastNoise = 0.0;
inline unsigned long simMs = 0;

// serial plumbing
struct SchedIn { unsigned long atMs; std::string data; size_t pos = 0; };
inline std::vector<SchedIn> scheduled;
inline std::deque<char> rxQueue;
inline std::string txLog;

inline void setTime(unsigned long ms) { simMs = ms; }
inline unsigned long now() { return simMs; }
inline void inject(unsigned long atMs, const char* s) { scheduled.push_back({atMs, std::string(s)}); }
inline void out(const std::string& s) { txLog += s; }
inline void simOut(const std::string& s) { std::printf("[sim] %s\n", s.c_str()); }

// pin capture (the sketch writes, the robot reacts)
inline int pinDig[32] = {0};
inline int pinPwmV[32] = {0};

inline void onPinWrite(uint8_t p, uint8_t v) { if (p < 32) pinDig[p] = v ? HIGH : LOW; }
inline void onAnalogWrite(uint8_t p, int v)  { if (p < 32) pinPwmV[p] = v; }
inline void pinPwmSet(uint8_t p, int v)      { if (p < 32) pinPwmV[p] = v; }
inline void pinDigSet(uint8_t p, int v)      { if (p < 32) pinDig[p] = v; }
inline int  pinLevel(uint8_t p) { return (p < 32) ? pinDig[p] : 0; }
inline int  pinPwm(uint8_t p)   { return (p < 32) ? pinPwmV[p] : 0; }

// isr registry for stage 2+ (INT0=D2, INT1=D3); mode stored so edge emulation can honour it
inline isr_t isr0 = 0, isr1 = 0;
inline int   isr0Mode = CHANGE, isr1Mode = CHANGE;
inline void attachIsr(uint8_t intNum, isr_t fn, int mode) {
  if (intNum == 0) { isr0 = fn; isr0Mode = mode; }
  if (intNum == 1) { isr1 = fn; isr1Mode = mode; }
}

// one shared edge emitter for driving AND hand-turning: every whole count of wheel travel
// toggles the signal level (2S edges/rev); the isr fires according to the attached edge mode.
// small epsilon absorbs float residue on exact whole-revolution boundaries (real discs have
// real edges - this is purely a host double-precision artifact)
inline void emitEdges(Wheel& w, double dx, isr_t isr, int mode) {
  const double k = 3.14159265358979 * WHEEL_D_MM / (SLOTS * EDGES);   // mm per count, N_rev = 2S
  const double eps = 1e-6;   // fp residue across long hand-turn chains must not eat an edge
  w.phase += dx / k;
  for (;;) {
    int dir = 0;
    if      (w.phase >=  1.0 - eps) dir = +1;
    else if (w.phase <= -1.0 + eps) dir = -1;
    else break;
    w.phase -= dir;
    w.ticks += dir;
    bool was = w.level;                    // every edge toggles the square wave, both directions
    w.level = !w.level;
    bool rising = !was, falling = was;     // transition type seen by the interrupt
    if (isr && (mode == CHANGE || (mode == RISING && rising) || (mode == FALLING && falling))) isr();
  }
}

// stage 2 bench: force a wheel to rotate by hand exactly n revolutions (positive = forward sense)
// fires the same edge events as driving, so hand-turn checks exercise the real isr path.
// paced at 120 ms per edge (5 rpm hand speed) so the sketch's isr debounce window is honoured -
// real hand rotation is orders of magnitude slower than chatter
inline void handTurn(double revL, double revR) {
  const double k = 3.14159265358979 * WHEEL_D_MM / (SLOTS * EDGES);
  double dL = revL * 3.14159265358979 * WHEEL_D_MM, dR = revR * 3.14159265358979 * WHEEL_D_MM;
  long counts = (long)std::lround(std::max(std::abs(dL), std::abs(dR)) / k);  // round, not floor
  double step = k;                                                      // one count per step
  double remL = dL, remR = dR;                // remaining travel per wheel
  long doneL = 0, doneR = 0;
  for (long i = 0; i < counts; i++) {
    double sL = (doneL < std::labs((long)std::lround(dL / k))) ? (dL > 0 ? step : -step) : 0.0;
    double sR = (doneR < std::labs((long)std::lround(dR / k))) ? (dR > 0 ? step : -step) : 0.0;
    emitEdges(wl, sL, isr0, isr0Mode);
    emitEdges(wr, sR, isr1, isr1Mode);
    doneL += (sL != 0.0); doneR += (sR != 0.0);
    simMs += 10;                              // pace the bench turn: 10 ms per count = 10 ms/edge
  }
}

// stage 2 bench: standstill vibration - the disc wiggles back and forth across one slot edge.
// deliberately asymmetric (settles slightly past the edge each time, like a real loose disc),
// so a naive counter nets a false count per oscillation - exactly what the freeze window must stop
inline void jitterWheel(int oscillations) {
  // sub-count wiggle MUST keep phase carry across oscillations (handTurn quantizes each
  // call to whole counts and would erase the asymmetry): emit raw dx with residual phase.
  // left wheel only - the vibration test targets one channel, mirroring a loose disc on it
  const double k = 3.14159265358979 * WHEEL_D_MM / (SLOTS * EDGES);
  const double out_ = +1.3 * k;                // forward crossing, 1.3 counts
  const double back = -0.7 * k;                // backward crossing, 0.7 counts -> nets +0.6/osc
  for (int i = 0; i < oscillations; i++) {
    emitEdges(wl, out_, isr0, isr0Mode);
    emitEdges(wl, back, isr0, isr0Mode);
  }
}

// wheel model: pwm + brake/coast state -> speed, distance, encoder edges
// fwdIn/revIn are the two direction inputs in "forward" order for that channel:
// channel A nominal forward = IN1=1,IN2=0; channel B nominal forward = IN3=0,IN4=1 (guide's table)
inline void stepWheel(Wheel& w, int enDig, int enPwm, int fwdIn, int revIn, double dt, double asym, bool braking, double noise) {
  int dir = 0;
  if (fwdIn != revIn) dir = fwdIn ? +1 : -1;   // one input high, one low = drive
  int duty = (enDig == HIGH) ? enPwm : 0;      // en low = coast, motor open
  duty = min(duty, 255);
  double target = 0.0;
  if (dir != 0 && duty > DEADBAND_PWM) {
    target = dir * (duty / 255.0) * MAX_MM_S * asym * (1.0 + noise);
  }
  // first order approach to target speed; time constant depends on stop mode
  double tau = 0.12;                            // spool up/down while driven
  if (dir == 0 || duty == 0) tau = braking ? RUNON_BRAKE_S : RUNON_COAST_S;
  w.v_mm_s += (target - w.v_mm_s) * (dt / tau);
  if (tau <= dt) w.v_mm_s = target;
  double dx = w.v_mm_s * dt;
  w.x_mm += dx;

  emitEdges(w, dx, (&w == &wl) ? isr0 : isr1, (&w == &wl) ? isr0Mode : isr1Mode);
}

inline bool channelBraking() {
  // dynamic braking on both channels: EN high + both inputs low (matches the sketch's brake())
  return pinDig[5] == HIGH && pinDig[6] == HIGH && pinDig[14] == LOW && pinDig[15] == LOW &&
         pinDig[16] == LOW && pinDig[17] == LOW;
}

inline bool SIGN_FLIP = true;   // real-robot wiring: bridge "forward" rolls the robot backward
                                // (floor-verified 14.10). The sketch corrects via SIGN_L/R=-1,
                                // so the model must also negate: a nominal-fwd pin pattern
                                // produces backward travel. Level flip keeps fwd/rev pairing.
inline void stepPhysics(unsigned long dtMs) {
  double dt = dtMs / 1000.0;
  if (dt <= 0) return;
  lastNoise = 0.5 * lastNoise + 0.5 * sin(simMs * 0.013);   // deterministic low-freq noise
  bool braking = channelBraking();
  int d14 = pinDig[14], d15 = pinDig[15], d16 = pinDig[16], d17 = pinDig[17];
  if (SIGN_FLIP) {  // see the sketch's SIGN_L/R comment: invert the level of every direction input
    d14 = !d14; d15 = !d15; d16 = !d16; d17 = !d17;
  }
  stepWheel(wl, pinDig[5],  pinPwmV[5], d14, d15, dt, 1.0,    braking, NOISE_FRAC * lastNoise);
  stepWheel(wr, pinDig[6],  pinPwmV[6], d17, d16, dt, ASYM_R, braking, NOISE_FRAC * lastNoise);

  // chassis pose from per-wheel motion (differential drive)
  double dsL = wl.v_mm_s * dt, dsR = wr.v_mm_s * dt;
  double ds  = 0.5 * (dsL + dsR);
  double dth = (dsR - dsL) / TRACK_MM;
  poseX += ds * cos(heading + dth * 0.5);
  poseY += ds * sin(heading + dth * 0.5);
  heading += dth;
}

inline void resetRobot() {
  wl = Wheel(); wr = Wheel();
  heading = poseX = poseY = 0;
  lastNoise = 0;
}

inline void dumpPose() {
  char buf[160];
  snprintf(buf, sizeof buf,
    "[sim] pose x=%.1fmm y=%.1fmm heading=%.1fdeg | ticks L=%ld R=%ld",
    poseX, poseY, heading * 180.0 / 3.14159265358979, wl.ticks, wr.ticks);
  simOut(buf);
}

} // namespace sim

// ----- SerialMock member implementations -----
inline int SerialMock::available() {
  for (auto& s : sim::scheduled)
    if (s.atMs <= sim::simMs && s.pos < s.data.size())
      sim::rxQueue.push_back(s.data[s.pos++]);
  return (int)sim::rxQueue.size();
}
inline int SerialMock::read() {
  if (sim::rxQueue.empty()) return -1;
  char c = sim::rxQueue.front(); sim::rxQueue.pop_front();
  return (unsigned char)c;
}
inline int SerialMock::peek() {              // stage 5: look, do not consume
  if (sim::rxQueue.empty()) return -1;
  return (unsigned char)sim::rxQueue.front();
}
inline float SerialMock::parseFloat() {
  // enough for a simple stage-5 sketch; the stage-1 sketch never calls it
  bool neg = false; bool started = false; double v = 0; double frac = 0, scale = 0.1; bool inFrac = false;
  for (;;) {
    int c = read();
    if (c < 0) break;
    char ch = (char)c;
    if (ch == '-' && !started) { neg = true; started = true; continue; }
    if (ch >= '0' && ch <= '9') {
      started = true;
      if (inFrac) { frac += (ch - '0') * scale; scale *= 0.1; }
      else v = v * 10 + (ch - '0');
      continue;
    }
    if (ch == '.' && !inFrac) { inFrac = true; started = true; continue; }
    break;  // first non-numeric char stops the parse (real parseFloat waits for timeout; simplified here)
  }
  return (float)((neg ? -1.0 : 1.0) * (v + frac));
}
