// iss lab2 - stage 5: two-state machine FOLLOWING<->SEARCHING with line search
// builds on stage 4 unchanged: calibration, readError()/lineLost() (SIGNED int),
// fixed-period 50 Hz millis loop, dead-man's switch g+y, clip-only applyCorrection(),
// PID (P+I+D hand-written), pidReset(), trace buffer (now with a state column)
// adds: explicit state machine, spin-in-place search toward the side the line was
// last seen on, timeout stop, N-consecutive-good-readings return, force-search test cmd
// school results from stage 4 (globals still start at 0):
// // school results: kp=___ ki=___ kd=___ base=___
// motor layer from LAB1 stage7: ENA=5 IN1=A0 IN2=A1, ENB=6 IN3=A2 IN4=A3, SIGN_L=SIGN_R=-1
// PWM_MIN=85 (LAB1 dead band 75 + margin)
// LED: D8 is the searching indicator - NOT D13 (TLC1543 clock, library-owned)

#include <TRSensors.h>

static const unsigned long SERIAL_BAUD = 9600;
static const uint8_t NUM_SENSORS = 5;
static const int SETPOINT = 2000;
static const int LOST_LIMIT = 2000;
static bool white_line = false;   // set from Stage 1 polarity measurement; flip with 'w' if unsure at school

static const int PWM_MAX = 255;
static const int PWM_MIN = 85;
static const unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz; period absorbed into ki/kd

// ===== the two operating states (explicit, assignment requirement) =====
enum State { FOLLOWING, SEARCHING };
static State state = FOLLOWING;

// search parameters (all live-tunable)
static int  searchSpinSpeed   = 120;    // magnitude of the spin, 0..255
static unsigned long searchTimeoutMs = 3000;   // give up after this
static unsigned int  foundReadingsNeeded = 3;  // consecutive good readings to return
static bool searchEnabled = false;      // search only runs if true - safety
static bool forceSearch = false;        // set by 'F', acts for ONE tick, then cleared

// state machine bookkeeping
static int8_t lastSeenSide = 0;         // -1 left, +1 right, 0 unknown yet
static unsigned long searchStartTime = 0;
static unsigned int  searchFoundCount = 0;
static bool searchTimedOut = false;     // timeout message prints ONCE
static bool searchDisabledMsgDone = false;

// tunables (all zero = nothing moves, safety default)
static float kp = 0.0f;
static float ki = 0.0f;
static float kd = 0.0f;
static int   baseSpeed = 0;
static float iLimit = 1000.0f;
static bool  smoothError = false;
static bool  driving = false;
static bool  dryRun   = true;
static bool  trace   = false;
static bool  record  = false;

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
static int lastU = 0;   // last correction, for the trace row

// ===== controller state =====
static float integral = 0.0f;
static int   prevError = 0;
static int   smoothedError = 0;
static unsigned long prevTickMs = 0;

// ===== stage 2 functions, unchanged =====
// SIGNED int — never change this type, the lost-line sign depends on it
int readError() {
  lastLinePos = trs.readLine(sensorValues, white_line);
  return lastLinePos - SETPOINT;
}

bool lineLost(int e) {
  return e > LOST_LIMIT || e < -LOST_LIMIT;
}

// ===== PID (stage 4, unchanged) =====
void pidReset() {
  integral = 0.0f;
  prevError = 0;
  smoothedError = 0;
  Serial.println(F("PIDRESET"));
}

int pidStep(int error) {
  float p = kp * (float)error;

  // I: accumulate, then clamp - a lost-line stall would otherwise wind it up
  integral += ki * (float)error;
  if (integral >  iLimit) integral =  iLimit;
  if (integral < -iLimit) integral = -iLimit;

  // D: raw by default; IIR 3/4 old + 1/4 new when smoothing is on
  int errForD = error;
  if (smoothError) {
    smoothedError = (3 * smoothedError + error) / 4;
    errForD = smoothedError;
  }
  float d = kd * (float)(errForD - prevError);

  prevError = error;

  long u = (long)(p + integral + d);
  if (u >  PWM_MAX) u =  PWM_MAX;
  if (u < -PWM_MAX) u = -PWM_MAX;
  return (int)u;
}

// ===== correction split (stage 3/4, clip-only) =====
// line RIGHT -> error>0 -> u>0 -> L=base+u > R=base-u -> turns right, toward the line
void applyCorrection(int u) {
  int leftSpeed  = baseSpeed + u;
  int rightSpeed = baseSpeed - u;

  if (leftSpeed  >  PWM_MAX) leftSpeed  =  PWM_MAX;
  if (leftSpeed  < -PWM_MAX) leftSpeed  = -PWM_MAX;
  if (rightSpeed >  PWM_MAX) rightSpeed =  PWM_MAX;
  if (rightSpeed < -PWM_MAX) rightSpeed = -PWM_MAX;

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
// stage 4 note repeated: literals live in F(), rows are packed.
// spec's 130x14 B was computed on unpacked rows; packed to 7 B (t,e,u int16 + st int8)
// I keep 160 rows = 1120 B = 3.2 s window at LESS RAM than 130x14 = 1820 B
static const unsigned int BUF_SIZE = 160;
struct TraceRow { int16_t t; int16_t e; int16_t u; int8_t st; };   // st: 0=FOLLOWING 1=SEARCHING
static TraceRow buf[BUF_SIZE];
static unsigned int bufHead = 0;
static bool bufFull = false;

void recordTick(int16_t t, int16_t e, int16_t u, int8_t st) {
  buf[bufHead] = { t, e, u, st };
  bufHead++;
  if (bufHead >= BUF_SIZE) {   // wraparound: oldest data is overwritten first
    bufHead = 0;
    bufFull = true;
  }
}

void dumpBuffer() {
  Serial.println(F("# t,e,u,L,R,state"));
  unsigned int n = bufFull ? BUF_SIZE : bufHead;
  for (unsigned int i = 0; i < n; i++) {
    TraceRow& row = buf[bufFull ? (bufHead + i) % BUF_SIZE : i];
    Serial.print(row.t); Serial.print(',');
    Serial.print(row.e); Serial.print(',');
    Serial.print(row.u); Serial.print(',');
    if (row.t < 0) {
      // sentinel row (lost line): L,R printed as 0
      Serial.print(F("0,0,"));
    } else {
      Serial.print(baseSpeed + row.u); Serial.print(',');
      Serial.print(baseSpeed - row.u); Serial.print(',');
    }
    Serial.println(row.st);
  }
  Serial.print(F("# DUMP_DONE rows=")); Serial.println(n);
}

// ===== search movement =====
// spin in place toward lastSeenSide - safest first choice: does not carry the robot
// away from where the line was lost; chosen over arc (wrong radius can miss) and
// reverse (needs path memory we do not have). KNOWN LIMITATION for the demo: a spin
// cannot find a line lost "ahead" (sharp bend taken on the outside) - the sweep never
// crosses it; the timeout stop is the fallback in that case.
void applySearchMovement(int side) {
  if (!driving || dryRun) {
    // same guards as the controller - bench dry-run shows the spin but must not move
    if (trace) {
      Serial.print(F("DRYSPIN side="));
      Serial.print(side < 0 ? 'L' : 'R');
      Serial.print(F(" L=")); Serial.print(side < 0 ? -searchSpinSpeed : searchSpinSpeed);
      Serial.print(F(" R=")); Serial.println(side < 0 ? searchSpinSpeed : -searchSpinSpeed);
    }
    motorsStop();
    return;
  }
  if (side < 0) driveWheels(-searchSpinSpeed, +searchSpinSpeed);   // spin left
  else          driveWheels(+searchSpinSpeed, -searchSpinSpeed);   // spin right
}

// ===== the state machine - ONE function, called once per tick =====
void stateMachineStep(int e) {
  lastU = 0;

  if (state == FOLLOWING) {
    // 'F' fakes one lost tick so the search can be exercised on the bench
    bool lost = lineLost(e) || forceSearch;
    forceSearch = false;   // acts for exactly one tick

    if (lost) {
      pidReset();   // integral/D must not carry garbage into the search or back
      // SEARCH DIRECTION DERIVATION: the sign of the out-of-range error says which
      // side the line was LAST seen on (-1 -> library returned -1, left; +1 -> 4001)
      lastSeenSide = (e < 0) ? -1 : +1;
      state = SEARCHING;
      searchStartTime = millis();
      searchFoundCount = 0;
      searchTimedOut = false;         // fresh timeout window per search episode
      searchDisabledMsgDone = false;
      digitalWrite(PIN_LED, HIGH);    // visual: search active
      if (trace) {
        Serial.print(F("->SEARCH side="));
        Serial.println(lastSeenSide < 0 ? 'L' : 'R');
      }
    } else {
      lastU = pidStep(e);
      applyCorrection(lastU);
    }
    return;
  }

  // ===== SEARCHING =====
  if (!searchEnabled) {
    // search suppressed: losing the line just stops the robot until 'S' is sent
    motorsStop();
    if (!searchDisabledMsgDone) {
      Serial.println(F("SEARCH DISABLED - stopped"));
      searchDisabledMsgDone = true;
    }
    return;
  }

  if (!lineLost(e)) {
    // one good reading is NOT enough: during a fast spin the strip can sweep across
    // the line accidentally (assignment warning) - require N consecutive readings
    searchFoundCount++;
    if (searchFoundCount >= foundReadingsNeeded) {
      pidReset();   // enter following with a clean controller state
      state = FOLLOWING;
      searchTimedOut = false;
      digitalWrite(PIN_LED, LOW);
      if (trace) Serial.println(F("->FOLLOW"));
    }
    return;
  }

  // still lost
  searchFoundCount = 0;
  if (millis() - searchStartTime > searchTimeoutMs) {
    // TIMEOUT DECISION: stop and wait - do not widen, do not reverse. Safe limiting
    // behaviour for the "end of the line" case; a robot must not spin forever.
    motorsStop();
    if (!searchTimedOut) {
      Serial.println(F("SEARCH TIMEOUT - stopped"));
      searchTimedOut = true;   // message once, stop is idempotent every tick
    }
    return;
  }

  applySearchMovement(lastSeenSide);
}

// ===== one control tick at the fixed 50 Hz cadence =====
static unsigned long dtSum = 0;
static unsigned int  dtCount = 0;
static unsigned int  overCount = 0;

void tickController() {
  unsigned long now = millis();
  if (prevTickMs != 0) {
    unsigned long dt = now - prevTickMs;
    dtSum += dt;
    dtCount++;
    if (dt > LOOP_PERIOD_MS + 2) overCount++;
    if (dtCount >= 500) {
      if (trace) {
        Serial.print(F("TICK dt_avg=")); Serial.print(dtSum / dtCount);
        Serial.print(F(" over=")); Serial.println(overCount);
      }
      dtSum = 0; dtCount = 0; overCount = 0;
    }
  }
  prevTickMs = now;

  int e = readError();
  stateMachineStep(e);   // handles BOTH states: pidStep+applyCorrection OR search
  if (record) {
    // sentinel t=-1 marks lost-line ticks in the CSV (not a real timestamp)
    if (lineLost(e)) recordTick(-1, (int16_t)e, 0, (int8_t)state);
    else             recordTick((int16_t)(now & 0x7FFF), (int16_t)e, (int16_t)lastU, (int8_t)state);
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
              pidReset();
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
      digitalWrite(PIN_LED, LOW);
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
      if (record) { bufHead = 0; bufFull = false; }
      Serial.print(F("record=")); Serial.println(record ? 1 : 0);
      break;
    case 'D':
      dumpBuffer();
      break;
    case 'z':
      pidReset();
      break;
    case 'S':
      searchEnabled = !searchEnabled;
      Serial.print(F("searchEnabled=")); Serial.println(searchEnabled ? 1 : 0);
      if (!searchEnabled && state == SEARCHING) {
        motorsStop();
        Serial.println(F("SEARCH STOPPED"));
      }
      if (searchEnabled) pidReset();   // enter search with a clean controller state
      break;
    case 'Q': {
      Serial.print(F("STATE state="));
      Serial.print(state == FOLLOWING ? F("FOLLOWING") : F("SEARCHING"));
      Serial.print(F(" side="));
      Serial.print(lastSeenSide < 0 ? 'L' : (lastSeenSide > 0 ? 'R' : '?'));
      Serial.print(F(" searchEnabled=")); Serial.print(searchEnabled ? 1 : 0);
      Serial.print(F(" foundCount=")); Serial.print(searchFoundCount);
      Serial.print(F(" elapsedMs="));
      Serial.print(state == SEARCHING ? millis() - searchStartTime : 0);
      Serial.print(F(" timeoutMs=")); Serial.println(searchTimeoutMs);
      break;
    }
    case 'F':
      if (state == FOLLOWING) {
        forceSearch = true;   // next tick pretends lineLost once, then cleared
        Serial.println(F("FORCED SEARCH"));
      } else {
        Serial.println(F("already SEARCHING"));
      }
      break;
    case 'a': {
      int v = atoi(arg);
      if (v < 0) v = 0;
      if (v > PWM_MAX) v = PWM_MAX;
      searchSpinSpeed = v;
      Serial.print(F("searchSpinSpeed=")); Serial.println(searchSpinSpeed);
      break;
    }
    case 'T':
      searchTimeoutMs = strtoul(arg, NULL, 10);
      Serial.print(F("searchTimeoutMs=")); Serial.println(searchTimeoutMs);
      break;
    case 'n':
      foundReadingsNeeded = (unsigned int)atoi(arg);
      Serial.print(F("foundReadingsNeeded=")); Serial.println(foundReadingsNeeded);
      break;
    case 'h':
      Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go(+y) s=stop"));
      Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
      Serial.println(F("      m=toggle-smoothing t=toggle-trace R=toggle-record D=dump-buffer"));
      Serial.println(F("      z=pidReset h=help"));
      Serial.println(F("State cmds: S=toggle-search-enabled Q=state F=force-search"));
      Serial.println(F("  Search params: a<val>=spinSpeed T<val>=timeoutMs n<val>=foundNeeded"));
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

  Serial.println(F("LAB2 Stage 5 - line search"));
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
  Serial.println(F("State cmds: S=toggle-search-enabled Q=state F=force-search"));
  Serial.println(F("  Search params: a<val>=spinSpeed T<val>=timeoutMs n<val>=foundNeeded"));
  Serial.print(F("state=FOLLOWING searchEnabled=")); Serial.println(searchEnabled ? 1 : 0);
  Serial.print(F("dryRun=")); Serial.print(dryRun ? 1 : 0);
  Serial.print(F(" driving=")); Serial.println(driving ? 1 : 0);
  Serial.println(F("all gains 0, base 0 - set k/i/d/b, S enables search, g+y drives"));

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
