// iss lab2 - stage 4: full PID (P+I+D, hand-written - no PID library) + RAM trace buffer
// builds on stage 3 unchanged: calibration, readError()/lineLost() (SIGNED int),
// fixed-period 50 Hz millis loop, dead-man's switch g+y, clip-only applyCorrection()
// adds: I term with iLimit clamp, D term raw-or-smoothed, pidReset(), TICK diagnostic,
// record/dump trace buffer for the three school plots
// stage 3 working values from school go on the next line (globals still start at 0):
// // school results: kp=___ baseSpeed=___
// motor layer from LAB1 stage7: ENA=5 IN1=A0 IN2=A1, ENB=6 IN3=A2 IN4=A3, SIGN_L=SIGN_R=-1
// PWM_MIN=85 (LAB1 dead band 75 + margin), LED on D8 (D13 = TLC1543 clock, library-owned)
// RAM note: kickoff's 150x12 B buffer does NOT fit - every Serial literal lives in RAM
// unless wrapped in F(), and base usage alone is ~1 KB. Fixed by: all literals F(),
// row packed to 6 B (t,e as int16; L,R derivable as base+-u and recomputed at dump),
// 160 rows = 960 B = 3.2 s window at 50 Hz

#include <TRSensors.h>

static const unsigned long SERIAL_BAUD = 9600;
static const uint8_t NUM_SENSORS = 5;
static const int SETPOINT = 2000;
static const int LOST_LIMIT = 2000;
static bool white_line = false;   // set from Stage 1 polarity measurement; flip with 'w' if unsure at school

static const int PWM_MAX = 255;
static const int PWM_MIN = 85;                    // LAB1 floor-measured dead band + margin
static const unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz; period is ABSORBED into ki/kd
                                                  // (recorded convention - changing the period
                                                  //  invalidates the tuned settings)

// tunables (all zero = nothing moves, assignment safety default)
static float kp = 0.0f;
static float ki = 0.0f;
static float kd = 0.0f;
static int   baseSpeed = 0;
static float iLimit = 1000.0f;     // integral clamp; tunable live
static bool  smoothError = false;  // D on raw (false) or smoothed (true) error
static bool  driving = false;
static bool  dryRun   = true;
static bool  trace   = false;      // print per-tick to Serial (user-toggled only)
static bool  record  = false;      // record per-tick into RAM buffer

// motor pins copied from LAB1 stage7 pin map (frozen values)
static const uint8_t PIN_ENA = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // left pwm + dir
static const uint8_t PIN_ENB = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // right pwm + dir
static const uint8_t PIN_LED = 8;
static const int8_t SIGN_L = -1;   // floor-verified 14.10: bridge-fwd rolls backward
static const int8_t SIGN_R = -1;

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int rawValues[NUM_SENSORS];
static int lastLinePos = 0;

// ===== controller state (names kept for stage 5) =====
static float integral = 0.0f;
static int   prevError = 0;          // for D
static int   smoothedError = 0;     // for optional D smoothing
static unsigned long prevTickMs = 0; // measured-period diagnostic

// ===== stage 2 functions, unchanged =====
// SIGNED int — never change this type, the lost-line sign depends on it
int readError() {
  lastLinePos = trs.readLine(sensorValues, white_line);
  return lastLinePos - SETPOINT;
}

bool lineLost(int e) {
  return e > LOST_LIMIT || e < -LOST_LIMIT;
}

// ===== PID =====
void pidReset() {
  integral = 0.0f;
  prevError = 0;
  smoothedError = 0;
  Serial.println(F("PIDRESET"));
}

// ONE control step at the fixed period. Period absorbed into ki/kd, not explicit.
int pidStep(int error) {
  // P: plain gain on the current error
  float p = kp * (float)error;

  // I: accumulate, then clamp - without the clamp a lost-line stall would wind it up
  integral += ki * (float)error;
  if (integral >  iLimit) integral =  iLimit;
  if (integral < -iLimit) integral = -iLimit;

  // D: differentiate raw error by default; the IIR (3/4 old + 1/4 new) is light
  // enough not to delay the anticipation, but trades some D effect for noise
  int errForD = error;
  if (smoothError) {
    smoothedError = (3 * smoothedError + error) / 4;
    errForD = smoothedError;
  }
  float d = kd * (float)(errForD - prevError);

  // prevError stores the RAW error so the raw-D case always compares like-with-like;
  // the smoothed value is carried in smoothedError instead
  prevError = error;

  long u = (long)(p + integral + d);
  if (u >  PWM_MAX) u =  PWM_MAX;
  if (u < -PWM_MAX) u = -PWM_MAX;
  return (int)u;
}

// ===== correction split (stage 3 logic, clip-only) =====
// expected direction: line RIGHT -> error>0 -> u>0 -> L=base+u > R=base-u -> turns right
void applyCorrection(int u) {
  int leftSpeed  = baseSpeed + u;
  int rightSpeed = baseSpeed - u;

  // clip-only: clamp each wheel into [-255,255], leave the other alone
  if (leftSpeed  >  PWM_MAX) leftSpeed  =  PWM_MAX;
  if (leftSpeed  < -PWM_MAX) leftSpeed  = -PWM_MAX;
  if (rightSpeed >  PWM_MAX) rightSpeed =  PWM_MAX;
  if (rightSpeed < -PWM_MAX) rightSpeed = -PWM_MAX;

  // dead band guard: below pwmMin a wheel buzzes but does not turn
  if (baseSpeed < PWM_MIN) {
    motorsStop();
    if (trace) Serial.println(F("GUARD baseSpeed < pwmMin, motors held"));
    return;
  }

  if (!driving || dryRun) {
    if (trace) {
      Serial.print(F("DRY u=")); Serial.print(u);
      Serial.print(F(" L=")); Serial.print(leftSpeed);
      Serial.print(F(" R=")); Serial.println(rightSpeed);
    }
    motorsStop();
    return;
  }

  driveWheels(leftSpeed, rightSpeed);
}

// ===== motor layer (LAB1 pattern incl. the floor-verified wiring sign flip) =====
void motorL(int pwm) {
  pwm *= SIGN_L;
  digitalWrite(PIN_IN1, pwm >= 0);
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX));
}
void motorR(int pwm) {
  pwm *= SIGN_R;
  digitalWrite(PIN_IN3, pwm <  0);
  digitalWrite(PIN_IN4, pwm >= 0);
  analogWrite(PIN_ENB, min(abs(pwm), PWM_MAX));
}
void driveWheels(int leftSpeed, int rightSpeed) {
  motorL(leftSpeed);
  motorR(rightSpeed);
}
void motorsStop() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// ===== trace buffer (RAM) =====
// row packed to 6 B: t and e fit int16 (t wraps at 32.7 s, window is 3.2 s; e +-2001).
// L/R are NOT stored - they are base+-u, recomputed at dump time (valid while base
// is unchanged between the run and the dump, the normal workflow)
static const unsigned int BUF_SIZE = 160;   // 160 ticks = 3.2 s at 50 Hz, 960 B
struct TraceRow { int16_t t; int16_t e; int16_t u; };
static TraceRow buf[BUF_SIZE];
static unsigned int bufHead = 0;   // next write index
static bool bufFull = false;       // wrapped around at least once

void recordTick(int16_t t, int16_t e, int16_t u) {
  buf[bufHead] = { t, e, u };
  bufHead++;
  if (bufHead >= BUF_SIZE) {   // wraparound: oldest data is overwritten first
    bufHead = 0;
    bufFull = true;
  }
}

void dumpBuffer() {
  Serial.println(F("# t,e,u,L,R"));
  unsigned int n = bufFull ? BUF_SIZE : bufHead;
  // oldest first: after a wrap the oldest row sits at bufHead
  for (unsigned int i = 0; i < n; i++) {
    TraceRow& row = buf[bufFull ? (bufHead + i) % BUF_SIZE : i];
    Serial.print(row.t); Serial.print(',');
    Serial.print(row.e); Serial.print(',');
    Serial.print(row.u); Serial.print(',');
    if (row.t < 0) {
      // sentinel row (lost line): L,R printed as 0 per the stage 4 spec
      Serial.print(F("0,0"));
    } else {
      Serial.print(baseSpeed + row.u); Serial.print(',');
      Serial.print(baseSpeed - row.u);
    }
    Serial.println();
  }
  Serial.print(F("# DUMP_DONE rows=")); Serial.println(n);
}

// ===== one control tick at the fixed 50 Hz cadence =====
static unsigned long dtSum = 0;      // measured-period accumulator
static unsigned int  dtCount = 0;
static unsigned int  overCount = 0;  // ticks later than period + 2 ms

void tickController() {
  unsigned long now = millis();
  if (prevTickMs != 0) {
    unsigned long dt = now - prevTickMs;
    dtSum += dt;
    dtCount++;
    if (dt > LOOP_PERIOD_MS + 2) overCount++;   // late tick, diagnostic only
    if (dtCount >= 500) {                       // every 500 ticks = 10 s
      if (trace) {
        Serial.print(F("TICK dt_avg=")); Serial.print(dtSum / dtCount);
        Serial.print(F(" over=")); Serial.println(overCount);
      }
      dtSum = 0; dtCount = 0; overCount = 0;
    }
  }
  prevTickMs = now;

  int e = readError();
  if (lineLost(e)) {
    // stop + reset NOW: the integral must not accumulate a value that is not an error;
    // stage 5 owns the search behaviour - stage 4 just halts cleanly
    motorsStop();
    pidReset();
    // sentinel row t=-1 marks the gap in the CSV (not a real timestamp)
    if (record) recordTick(-1, (int16_t)e, 0);
    if (trace) {
      Serial.print(F("LOST side="));
      Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
    }
    return;
  }

  int u = pidStep(e);
  if (record) recordTick((int16_t)(now & 0x7FFF), (int16_t)e, (int16_t)u);

  applyCorrection(u);

  if (trace && driving && !dryRun) {
    Serial.print(F("TRACE t=")); Serial.print(now);
    Serial.print(F(" e=")); Serial.print(e);
    Serial.print(F(" u=")); Serial.print(u);
    Serial.print(F(" L=")); Serial.print(baseSpeed + u);
    Serial.print(F(" R=")); Serial.println(baseSpeed - u);
  }
}

// ===== serial command line parsing =====
static char lineBuf[24];
static uint8_t lineLen = 0;

void handleLine(const char* buf) {
  char cmd = buf[0];
  const char* arg = buf + 1;
  while (*arg == ' ') arg++;

  // 'd' alone toggles dry-run; 'd<digit|->' sets kd
  bool argIsNumber = (arg[0] >= '0' && arg[0] <= '9') || arg[0] == '-';

  switch (cmd) {
    case 'r':
      trs.AnalogRead(rawValues);
      printValues("RAW  : ", rawValues);
      break;
    case 'c':
      trs.readCalibrated(sensorValues);
      printValues("CAL  : ", sensorValues);
      break;
    case 'p': {
      trs.AnalogRead(rawValues);
      int e = readError();
      Serial.print(F("POS raw="));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(rawValues[i]);
        if (i < NUM_SENSORS - 1) Serial.print(',');
      }
      Serial.print(F(" linePos=")); Serial.print(lastLinePos);
      Serial.print(F(" error="));   Serial.print(e);
      Serial.print(F(" lost="));    Serial.print(lineLost(e) ? 1 : 0);
      Serial.print(F(" side="));    Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
      break;
    }
    case 'd':
      if (argIsNumber) { kd = atof(arg); Serial.print(F("kd=")); Serial.println(kd, 3); }
      else { dryRun = !dryRun; Serial.print(F("dryRun=")); Serial.println(dryRun ? 1 : 0); }
      break;
    case 'g':
      // dead-man's switch: 'g' needs 'y' within 2 s or the start is aborted
      Serial.println(F("CONFIRM: send y within 2 s to start driving"));
      {
        unsigned long t0 = millis();
        while (millis() - t0 < 2000) {
          if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'y') {
              driving = true;
              dryRun = false;
              pidReset();   // clean controller state at every launch
              Serial.print(F("DRIVING kp=")); Serial.print(kp, 3);
              Serial.print(F(" ki=")); Serial.print(ki, 4);
              Serial.print(F(" kd=")); Serial.print(kd, 3);
              Serial.print(F(" base=")); Serial.println(baseSpeed);
              return;
            }
            break;
          }
        }
        Serial.println(F("ABORTED"));
      }
      break;
    case 's':
      driving = false;
      dryRun = true;
      motorsStop();
      Serial.println(F("STOPPED"));
      break;
    case 'k':
      kp = atof(arg); Serial.print(F("kp=")); Serial.println(kp, 3);
      break;
    case 'i':
      ki = atof(arg); Serial.print(F("ki=")); Serial.println(ki, 4);
      break;
    case 'b':
      baseSpeed = atoi(arg); Serial.print(F("baseSpeed=")); Serial.println(baseSpeed);
      break;
    case 'l':
      iLimit = atof(arg); Serial.print(F("iLimit=")); Serial.println(iLimit, 1);
      break;
    case 'm':
      smoothError = !smoothError;
      Serial.print(F("smoothError=")); Serial.println(smoothError ? 1 : 0);
      break;
    case 't':
      trace = !trace;
      Serial.print(F("trace=")); Serial.println(trace ? 1 : 0);
      break;
    case 'R':
      record = !record;
      if (record) { bufHead = 0; bufFull = false; }   // fresh recording on ON
      // no buffer clear on OFF - dump it with D after the run
      Serial.print(F("record=")); Serial.println(record ? 1 : 0);
      break;
    case 'D':
      dumpBuffer();
      break;
    case 'z':
      pidReset();
      break;
    case 'h':
      Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go(+y) s=stop"));
      Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
      Serial.println(F("      m=toggle-smoothing t=toggle-trace R=toggle-record D=dump-buffer"));
      Serial.println(F("      z=pidReset h=help"));
      break;
    case '\r':
    case '\n':
    case 0:
      break;
    default:
      Serial.print(F("? ")); Serial.println(cmd);
      break;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ===== stage 1/2 helpers kept verbatim =====
void printValues(const char* label, unsigned int* v) {
  Serial.print(label);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(v[i]);
    if (i < NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  motorsStop();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(SERIAL_BAUD);

  Serial.println(F("LAB2 Stage 4 - full PID"));
  Serial.println(F("CALIBRATING... rotate the robot so the strip crosses the line"));
  digitalWrite(PIN_LED, HIGH);

  // 400 calls x 10 reads each = ~10 s, sweep line + background in that window
  for (int i = 0; i < 400; i++) {
    trs.calibrate();
  }

  digitalWrite(PIN_LED, LOW);
  Serial.println(F("CALIBRATION DONE"));

  printValues("MIN 0..4: ", trs.calibratedMin);
  printValues("MAX 0..4: ", trs.calibratedMax);
  Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go(+y) s=stop"));
  Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
  Serial.println(F("      m=toggle-smoothing t=toggle-trace R=toggle-record D=dump-buffer"));
  Serial.println(F("      z=pidReset h=help"));
  Serial.print(F("dryRun=")); Serial.print(dryRun ? 1 : 0);
  Serial.print(F(" driving=")); Serial.println(driving ? 1 : 0);
  Serial.println(F("all gains 0, base 0 - set k/i/d/b, verify in dry-run, then g+y"));

  motorsStop();   // last line of setup: safe state
}

// fixed-period non-blocking loop, LAB1 pattern: lastTick += period keeps the
// cadence drift-free even if a tick overruns
static unsigned long lastTick = 0;

void loop() {
  readSerialCommands();
  if (millis() - lastTick >= LOOP_PERIOD_MS) {
    lastTick += LOOP_PERIOD_MS;
    tickController();
  }
}
