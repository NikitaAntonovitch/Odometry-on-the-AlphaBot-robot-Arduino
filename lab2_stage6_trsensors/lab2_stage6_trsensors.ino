// iss lab2 - stage 6: FINAL - test track. Stages 1-5 merged + crossing awareness.
// 2026-11-22 refactor: ALL sensor handling through the TRSensors library.
//   one control-loop sensor call = trs.readLine()  (does the adc read, the
//   0..1000 rescale, the polarity inversion, the weighted average 0..4000 and
//   the lost-line detection in ONE call: -1 / 4001 = lost, sign = last side)
//   raw diagnostic      = trs.AnalogRead()      ('r')
//   calibrated diag     = trs.readCalibrated()  ('c')
//   calibration         = trs.calibrate() x400  (boot + 'W' warmup)
//   min/max arrays      = trs.calibratedMin / trs.calibratedMax (prints + EEPROM)
// the 21.11 hand-written position layer (computePosition + lineDarkFlag
// inversion + sum<250 lost test + polarity auto-measure) is REPLACED by
// readLine() - the pid, motors, state machine, crossing logic and serial
// commands are unchanged.
//
// stage map in this file:
//   Stage 1: calibration (boot + 'W' warmup) via trs.calibrate(), min/max prints
//   Stage 2: readError() = trs.readLine(...) - SETPOINT; lineLost(e) = |e| > 2000
//            (2000 = centre; +-2001 = lost, sign = side the line was last seen
//            on, kept for the search direction). white_line comes from the
//            bench polarity MEASUREMENT (dark line -> 0), bench override 'w'
//   Stage 3: motor layer (LAB1 map + SIGN flip), clip-only applyCorrection, dead-man g
//   Stage 4: hand-written PID + iLimit clamp, trace buffer (packed rows, F() strings)
//   Stage 5: FOLLOWING<->SEARCHING state machine, spin search, timeout stop
//   Stage 6: crossing detect + hold-last-correction override, 'C' config, 'W' warmup
//
// deviations from the kickoff placeholders (floor-verified, all stages):
//   motor ENA=5 IN1=A0 IN2=A1, ENB=6 IN3=A2 IN4=A3 (not 4/7), SIGN_L=SIGN_R=-1
//   PWM_MIN=85 (not 40), LED on D8 (D13 = TLC1543 CLOCK pin, library-owned)
//   trace rows packed 7 B -> 160 rows (1120 B) fits RAM better than 130x14 B (1820 B)

#include <TRSensors.h>
#include <EEPROM.h>

/* ===== EEPROM calibration persistence =====
   Layout v5 (2026-11-22: library refactor - lineDarkFlag byte now stores
   whiteLine, lineLostSumMin dropped: the lost-line threshold lives inside
   the library). Address  Size  Content
   -------  ----  ---------------------------------
   0        1     MAGIC = 0xA8  (v5; any other value = no valid data)
   1        1     whiteLine (0 = dark line: sensor LOW on line, library
                  inverts 1000-v; 1 = bright line)
   2        2     calibratedMin[0]  (unsigned int, little-endian)
   4        2     calibratedMin[1]
   6        2     calibratedMin[2]
   8        2     calibratedMin[3]
   10       2     calibratedMin[4]
   12       2     calibratedMax[0]
   14       2     calibratedMax[1]
   16       2     calibratedMax[2]
   18       2     calibratedMax[3]
   20       2     calibratedMax[4]
   22       1     SIGN_L (int8_t as u8: 0xFF = -1, 0x01 = +1)
   23       1     SIGN_R (same encoding)
   24       4     kp (float)
   28       4     ki (float)
   32       4     kd (float)
   36       2     baseSpeed (int)
   38       1     CRC8 checksum (over bytes 1..37)
   -------------------------------
   Total: 39 bytes used, 985 bytes free on UNO (1024 byte EEPROM).
   v1/v2/v3/v4 saves are rejected by the magic change (0xA5/0xA6/0xA7 never
   match 0xA8) plus per-field range guards - never load garbage.
   Writes happen ONLY on the explicit 'E' command (EEPROM ~100k cycles). */

static const uint8_t  CALIB_EEPROM_MAGIC = 0xA8;   // layout v5 (2026-11-22)
static const int      CALIB_EEPROM_ADDR  = 0;      // start address

// Simple CRC8 (Polynomial 0x07, initial 0x00, no final XOR) - same as Dallas/Maxim 1-Wire CRC.
static uint8_t _crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ===== constants / settings =====
static const unsigned long SERIAL_BAUD = 9600;    // baud MUST match monitor
static const uint8_t NUM_SENSORS = 5;             // library is fixed at 5
static const int SETPOINT = 2000;                 // readLine() centre
static const int LOST_LIMIT = 2000;               // |error| above this = lost
                                                  // (library sentinels -1/4001
                                                  // become +-2001 after -SETPOINT)

// POLARITY, not guessed: bench-measured 09-11 [HW-VERIFIED] - line reads LOW
// (~170-350 raw), background HIGH (~700-900) -> DARK line -> white_line = 0.
// The library then inverts every value (1000 - v) before its weighted
// average: exactly the correction the old lineDarkFlag=1 loop applied.
// 1 would be for a bright line on a dark floor (never seen on this track).
// Bench override: 'w'. Saved to EEPROM with 'E'.
static const unsigned char WHITE_LINE_DEFAULT = 0;
static unsigned char whiteLine = WHITE_LINE_DEFAULT;

static const int PWM_MAX = 255;                   // Stage 3
static const int PWM_MIN = 85;                    // LAB1 dead band 75 + margin
static const unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz; period absorbed into ki/kd

// ===== Stage 6: crossing handling =====
// A crossing stripe puts the line under several widely separated sensors at
// once; the weighted average then points at a place where NO line is, and a
// normal PID step would steer the robot off the track. Detection: count
// sensors "on line" among the corrected values readLine() leaves in
// sensorValues[] (same values the old corrected array held: 1000-v for our
// dark line; ON_LINE_CAL_MIN = 300 matches the library's own on-line test).
static bool crossingHandling = true;               // toggle with 'x'
static const int CROSSING_THRESHOLD = 3;           // sensors "on line" at once to flag a crossing
                                                   // (if the on-site test disagrees: 2 or 4 = recompile)
static unsigned long crossingUntilMs = 0;          // hold window end, 0 = idle
static const unsigned long CROSSING_HOLD_MS = 200; // hold the last good correction this long

// ===== Stage 5: state machine =====
enum State { FOLLOWING, SEARCHING };
static State state = FOLLOWING;

static int  searchSpinSpeed   = 120;
static unsigned long searchTimeoutMs = 3000;
static const unsigned long SEARCH_TIMEOUT_DEFAULT_MS = 3000;   // Bug 1 fix: T0 falls back to this
static unsigned int  foundReadingsNeeded = 3;
static bool searchEnabled = true;       // search only runs if true - safety
static bool forceSearch = false;        // set by 'F', acts for ONE tick

static int8_t lastSeenSide = 0;
static unsigned long searchStartTime = 0;
static unsigned int  searchFoundCount = 0;
static bool searchTimedOut = false;
static bool searchDisabledMsgDone = false;

// ===== Stage 4: tunables + controller state =====
static float kp = 0.25f;
static float ki = 0.0f;
static float kd = 0.05f;   // unit: PWM per (error-count per SECOND) - D is per-second
                           // since 21.11. Old per-tick kd=0.3 -> 15 PWM per count at
                           // dt=20 ms, so 20-50 count residuals gave hundreds of PWM
                           // and saturated the output. PLACEHOLDER - NEEDS TUNING ON
                           // HARDWARE (start 0.02-0.1, raise until zigzag damps).
static int   baseSpeed = 50; //90
static float iLimit = 1000.0f;
static bool  driving = false;
static bool  dryRun   = true;
static bool  trace   = false;
static bool  record  = false;

static float integral = 0.0f;
// D path: first-order low-pass on the MEASUREMENT (error; setpoint 2000 is
// constant, so filtering the error IS filtering the measurement). D compares
// consecutive FILTER outputs - never filtered-vs-raw: that mismatch (old
// smoothedError vs prevError) spiked D by the full tracking lag, saturated u,
// one wheel reversed and the robot spun even at kp=0.
static float filteredError     = 0.0f;
static float prevFilteredError = 0.0f;
static const float PID_D_FILTER_ALPHA = 0.25f;   // tau = dt*(1-a)/a = 60 ms
static const float PID_DT_SCALE = 1000.0f / (float)LOOP_PERIOD_MS;  // 50 = 1/0.02 s
static bool filterSeeded = false; // false after reset: first tick seeds the filter
                                  // with the raw error and outputs NO D (no cold kick)
static unsigned long prevTickMs = 0;

// ===== Stage 6: crossing bookkeeping =====
static int lastU = 0;                   // correction of THIS tick, for the trace row
static int lastGoodU = 0;               // last valid correction, held through a crossing
static bool crossingEvent = false;      // fresh crossing this tick -> t=-2 trace sentinel
static unsigned long lastBlinkMs = 0;   // LED blink clock while crossing/hold
static bool blinkState = false;
static unsigned long lastTick = 0;      // fixed-period loop cadence bookkeeping

// ===== Stage 3: pins (LAB1 frozen map) =====
static const uint8_t PIN_ENA = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // left pwm + dir
static const uint8_t PIN_ENB = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // right pwm + dir
static const uint8_t PIN_LED = 8;                                // search=on, crossing=blink
static int8_t SIGN_L = -1;   // floor-verified 14.10 on the OLD robot - WRONG on the
static int8_t SIGN_R = -1;   // current unit (both-wheels-fwd = rotation). Runtime
                             // flip: 'j' left / 'J' right; save with 'E'; verify with 'M'

// ===== sensor (the ONLY library object + the two arrays) =====
TRSensors trs = TRSensors();               // constructor owns D10-D13 (TLC1543) - never touch those pins
unsigned int sensorValues[NUM_SENSORS];    // readLine() leaves the polarity-corrected
                                           // values here (1000-v for our dark line) -
                                           // reused by 'p' and the crossing test
unsigned int rawValues[NUM_SENSORS];       // raw adc, 'r' command diagnostics only

/* ===== Stage 2: sensor -> error (library path) =====
   ONE trs.readLine() call per tick does everything the old computePosition()
   did: AnalogRead -> per-sensor 0..1000 rescale -> polarity inversion
   (white_line replaces the old lineDarkFlag / 1000-v loop) -> weighted
   average 0..4000 -> lost-line test (old: corrected sum < 250; library: no
   sensor > 300). On a lost line readLine returns -1 or 4001 (which side the
   line was last seen on, remembered INSIDE the library), so:
     error = readLine() - SETPOINT   ->   0 = centred, +-2001 = lost
   The sign of +-2001 carries the search direction - do NOT use unsigned here
   (the library's own example does, and -1 would wrap to 65535). */
int readError() {
  int e = trs.readLine(sensorValues, whiteLine) - SETPOINT;   // signed int - keep it
  return e;
}

bool lineLost(int e) {
  return abs(e) > LOST_LIMIT;   // +-2001 = the library's -1 / 4001 lost sentinels
}

/* ===== Stage 1: calibration core (shared by setup and 'W') ===== */
void runCalibration() {
  // 400 calls x 10 reads each = ~10 s; strip must sweep line + background in that window
  for (int i = 0; i < 400; i++) {
    trs.calibrate();
  }
}

// Stage 6: warmup calibration at the track. Lighting differs across the room and the
// battery sags during a session - recalibrate ON SITE right before the demo runs.
// Allowed to block 10 s: the robot is stationary, wheels off, rotated by hand.
void warmupCalibrate() {
  Serial.println(F("CALIBRATING (warmup)... rotate the robot so the strip crosses the line"));
  digitalWrite(PIN_LED, HIGH);
  runCalibration();
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("CALIBRATION DONE"));
  printValues("MIN 0..4: ", trs.calibratedMin);
  printValues("MAX 0..4: ", trs.calibratedMax);
  Serial.println(F("polarity check: MIN range = dark end (the line), MAX = bright end (background)"));
  lastTick = millis();   // no catch-up tick burst after the 10 s block
  prevTickMs = 0;        // skip one dt sample
}

/* ===== EEPROM calibration save/load/clear =====
   Only touched by explicit commands (E save, L load, X clear) and the one
   auto-load attempt in setup() - never from the control loop. */
bool calibrationLoadFromEEPROM() {
  uint8_t magic;
  EEPROM.get(CALIB_EEPROM_ADDR, magic);
  if (magic != CALIB_EEPROM_MAGIC) return false;   // v1-v4 (0xA5/0xA6/0xA7) never match v5

  // read the full block into temporaries, verify CRC, then commit
  uint8_t wl, sl, sr;
  unsigned int cmin[NUM_SENSORS], cmax[NUM_SENSORS];
  float kpS, kiS, kdS;
  int baseS;
  uint8_t crcStored, crcCalc;

  EEPROM.get(CALIB_EEPROM_ADDR + 1, wl);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.get(CALIB_EEPROM_ADDR + 2 + i * 2, cmin[i]);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.get(CALIB_EEPROM_ADDR + 12 + i * 2, cmax[i]);
  EEPROM.get(CALIB_EEPROM_ADDR + 22, sl);
  EEPROM.get(CALIB_EEPROM_ADDR + 23, sr);
  EEPROM.get(CALIB_EEPROM_ADDR + 24, kpS);
  EEPROM.get(CALIB_EEPROM_ADDR + 28, kiS);
  EEPROM.get(CALIB_EEPROM_ADDR + 32, kdS);
  EEPROM.get(CALIB_EEPROM_ADDR + 36, baseS);
  EEPROM.get(CALIB_EEPROM_ADDR + 38, crcStored);
  // CRC8 over bytes 1..37 (wl + cmin + cmax + signs + gains = 37 bytes)
  crcCalc = 0;
  uint8_t* p;
  p = (uint8_t*)&wl;                  for (int i = 0; i < 1;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)cmin;                 for (int i = 0; i < 10; i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)cmax;                 for (int i = 0; i < 10; i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kpS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kiS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kdS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&baseS;               for (int i = 0; i < 2;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  crcCalc = _crc8_update(crcCalc, sl);
  crcCalc = _crc8_update(crcCalc, sr);

  if (crcCalc != crcStored) return false;   // CRC fail = treat as empty, never load garbage
  // per-field range guards: keep ANY corrupted/garbage block out regardless
  // of what wrote it
  if (wl > 1) return false;                  // polarity flag: only 0/1 are legal
  if (sl != 1 && sl != 0xFF) return false;   // only +/-1 signs are legal
  if (sr != 1 && sr != 0xFF) return false;
  if (kpS < 0.0f || kpS > 20.0f) return false;
  if (kiS < 0.0f || kiS > 5.0f) return false;
  if (kdS < 0.0f || kdS > 50.0f) return false;
  if (baseS < 0 || baseS > 255) return false;

  // commit only after verification
  whiteLine = wl;                     // v5 byte 1 = polarity (0 = dark line)
  SIGN_L = (int8_t)sl;
  SIGN_R = (int8_t)sr;
  kp = kpS;
  ki = kiS;
  kd = kdS;
  baseSpeed = baseS;
  for (int i = 0; i < NUM_SENSORS; i++) {
    trs.calibratedMin[i] = cmin[i];
    trs.calibratedMax[i] = cmax[i];
  }
  return true;
}

// save calibration, polarity, motor signs and PID gains to EEPROM (explicit 'E' only)
void calibrationSaveToEEPROM() {
  EEPROM.put(CALIB_EEPROM_ADDR, CALIB_EEPROM_MAGIC);
  uint8_t wl = whiteLine;
  uint8_t sl = (uint8_t)SIGN_L;
  uint8_t sr = (uint8_t)SIGN_R;
  EEPROM.put(CALIB_EEPROM_ADDR + 1, wl);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.put(CALIB_EEPROM_ADDR + 2 + i * 2,  trs.calibratedMin[i]);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.put(CALIB_EEPROM_ADDR + 12 + i * 2, trs.calibratedMax[i]);
  EEPROM.put(CALIB_EEPROM_ADDR + 22, sl);
  EEPROM.put(CALIB_EEPROM_ADDR + 23, sr);
  EEPROM.put(CALIB_EEPROM_ADDR + 24, kp);
  EEPROM.put(CALIB_EEPROM_ADDR + 28, ki);
  EEPROM.put(CALIB_EEPROM_ADDR + 32, kd);
  EEPROM.put(CALIB_EEPROM_ADDR + 36, baseSpeed);

  // CRC8 over the same 37 bytes
  uint8_t crc = 0;
  uint8_t* p;
  p = (uint8_t*)&wl;                  for (int i = 0; i < 1;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)trs.calibratedMin;    for (int i = 0; i < 10; i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)trs.calibratedMax;    for (int i = 0; i < 10; i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&kp;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&ki;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&kd;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&baseSpeed;           for (int i = 0; i < 2;  i++) crc = _crc8_update(crc, p[i]);
  crc = _crc8_update(crc, sl);
  crc = _crc8_update(crc, sr);
  EEPROM.put(CALIB_EEPROM_ADDR + 38, crc);
}

// mark EEPROM calibration as invalid (explicit 'X' only)
void calibrationClearEEPROM() {
  uint8_t bad = 0xFF;
  EEPROM.put(CALIB_EEPROM_ADDR, bad);   // any value != 0xA8 means "no data"
}

/* ===== Stage 4: PID (D fixed 2026-11-21) =====
   (1) D expressed PER SECOND: d = kd * dfilt * PID_DT_SCALE, kd unit =
       PWM per (count/s); u clamped to +-PWM_MAX here AND re-clamped per
       wheel in applyCorrection() - motors never see more than 255.
   (2) D on the MEASUREMENT through a first-order LPF (alpha 0.25):
       calibration-residual noise (20-50 counts/tick) is attenuated BEFORE
       differentiating instead of being amplified by kd*dt_scale. */
void pidReset() {
  integral = 0.0f;
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  filterSeeded = false;
  Serial.println(F("PIDRESET"));
}

int pidStep(int error) {
  float p = kp * (float)error;

  // first-order LPF on the measurement; seeded on the first tick after a
  // reset (filt = raw, d = 0) so the initial error is never mistaken for a
  // fast change - without the seed, e.g. e=1000 at start gives a 625 PWM
  // D kick (0.05*250*50) and saturates the output for 1-2 ticks
  float d = 0.0f;
  if (!filterSeeded) {
    filteredError = (float)error;
    prevFilteredError = filteredError;
    filterSeeded = true;
  } else {
    filteredError += PID_D_FILTER_ALPHA * ((float)error - filteredError);
    // D per second, between consecutive FILTER outputs only
    d = kd * (filteredError - prevFilteredError) * PID_DT_SCALE;
    prevFilteredError = filteredError;
  }

  /* ANTI-WINDUP (21.11: ki=0.001 + clamp -> robot in DRIVING but frozen).
     The I term kept integrating while the output sat pinned at the clamp
     limit: u stuck at one rail is indistinguishable from "no correction
     left to make" for the wheels, so with |u| pinned for a while the robot
     entered DRIVING and did not move. Conditional integration (clamp check
     FIRST, integrate only when it would not push further into the rail):
       - unclamped, or pushed AWAY from the rail -> integrate normally
       - pushed INTO the same-sign rail -> skip (integrator holds)
     This is the standard integrator-clamping scheme; a back-calculation
     variant would need the real saturated output + ki*Tt, more state, and
     the same effect here - the conditional form is easier to unit-test. */
  float uPre = p + integral + d;
  bool atPosRail = (uPre >= PWM_MAX);
  bool atNegRail = (uPre <= -PWM_MAX);
  bool windPos = atPosRail && (ki * (float)error > 0);
  bool windNeg = atNegRail && (ki * (float)error < 0);
  if (!windPos && !windNeg) {
    integral += ki * (float)error;
    if (integral >  iLimit) integral =  iLimit;   // iLimit stays as the hard cap
    if (integral < -iLimit) integral = -iLimit;
  }

  long u = (long)(p + integral + d);
  if (u >  PWM_MAX) u =  PWM_MAX;    // clamp BEFORE the motors (requirement 1)
  if (u < -PWM_MAX) u = -PWM_MAX;
  return (int)u;
}

/* ===== Stage 3: correction split (clip-only) ===== */
// SIGN FIX (bench 21.11): with the calibrated position path a POSITIVE raw u
// steered AWAY from the line, so the correction is negated ONCE here at the
// single point where it is applied to the motors. SIGN_L/SIGN_R are correct
// (M self-test) and stay untouched.
// After negation: error>0 -> applied u<0 -> L=base+u < R=base-u -> turns
// toward the line. lastU/lastGoodU hold the RAW pidStep output; the trace
// dump applies the same negation (L = base-u, R = base+u).
// SATURATION CLAMP (bench 21.11 PM): |u| above baseSpeed used to request a
// NEGATIVE inner wheel, and deadBandCompensate() turned that small negative
// request into 85+ PWM in REVERSE -> pivot -> spin (observed at kp=0.08
// base=90, and base<85). Clamped to +-baseSpeed: full correction = inner
// wheel at exactly 0 (true stop), never reverse. Search spin intentionally
// bypasses this clamp (its own path, see applySearchMovement).
void applyCorrection(int uIn) {
  int u = -uIn;                       // the ONE sign flip (requirement 3)
  if (u >  baseSpeed) u =  baseSpeed; // inner wheel stops, never reverses
  if (u < -baseSpeed) u = -baseSpeed;
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

/* ===== Stage 3: motor layer (LAB1 pattern + floor-verified sign flip) ===== */
/* ===== Dead-band compensation (requirement 4) =====
   The motors cannot start below ~75 PWM (LAB1 measurement; PWM_MIN=85 with
   margin). Requests 1..255 are remapped LINEARLY onto PWM_MIN..255 so small
   PID corrections still move the wheels; 0 stays a TRUE stop (no creep).
   Applied at the single point where PWM reaches the pins (motorL/motorR), so
   drive, search spin AND the M self-test all get it. Values printed as
   L/R/u elsewhere are REQUESTS - pre-compensation. */
int deadBandCompensate(int pwm) {
  if (pwm == 0) return 0;             // true stop (inner wheel at u=+-baseSpeed lands here)
  int mag = abs(pwm);
  if (mag > PWM_MAX) mag = PWM_MAX;
  int mapped = PWM_MIN + ((PWM_MAX - PWM_MIN) * mag) / PWM_MAX;
  return (pwm > 0) ? mapped : -mapped;
}

void motorL(int pwm) {
  pwm *= SIGN_L;
  pwm = deadBandCompensate(pwm);      // 1..255 -> 85..255, 0 = stop
  digitalWrite(PIN_IN1, pwm >= 0);
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX));
}
void motorR(int pwm) {
  pwm *= SIGN_R;
  pwm = deadBandCompensate(pwm);      // 1..255 -> 85..255, 0 = stop
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

/* ===== Search spin: SEPARATE raw path, bypasses the saturation clamp =====
   A spin NEEDS opposite wheel directions (e.g. -120/+120); the +-baseSpeed
   clamp in applyCorrection() exists to stop the DRIVE path from ever
   reversing a wheel. applySearchMovement() therefore calls driveWheels()
   directly (still through deadBandCompensate, which is fine: the spin speeds
   are >= PWM_MIN-sized on both wheels, so no small-reverse amplification). */

/* ===== 'M' motor self-test (no sensors, no PID, no search) =====
   Wheels lifted. ~1.5 s per step with a stop between.
   NOT gated by dryRun: calls motorL/motorR DIRECTLY, so it exercises the raw
   motor layer exactly as applyCorrection() does. The only gate is on the 'M'
   command itself: refused while the wheels are live on the floor (safety).
   PWM: 100, NOT 60. 60 sat BELOW the motor dead band (~75, PWM_MIN=85), so
   the driver received PWM but the wheels never moved - M looked dead while
   g (base=90) still drove. That mismatch was the bench mystery of 20.11.
   PASS = forward on "LEFT fwd", forward on "RIGHT fwd", body-forward (not
   spin) on "BOTH fwd". Wrong direction on one wheel -> 'j'/'J' -> M again. */
void motorSelfTest() {
  static const int TEST_PWM = 100;   // above dead band + PWM_MIN; still slow and catchable
  static const unsigned long STEP_MS = 1500;
  static const unsigned long GAP_MS = 700;

  motorsStop();
  Serial.print(F("MOTOR TEST 1/4: LEFT fwd   ENA=")); Serial.print(TEST_PWM);
  Serial.print(F(" pwm_out=")); Serial.println(SIGN_L * TEST_PWM);
  motorL(TEST_PWM);
  delay(STEP_MS);
  motorsStop();
  delay(GAP_MS);

  Serial.print(F("MOTOR TEST 2/4: RIGHT fwd  ENB=")); Serial.print(TEST_PWM);
  Serial.print(F(" pwm_out=")); Serial.println(SIGN_R * TEST_PWM);
  motorR(TEST_PWM);
  delay(STEP_MS);
  motorsStop();
  delay(GAP_MS);

  Serial.print(F("MOTOR TEST 3/4: BOTH fwd   pwm_out=")); Serial.println(SIGN_L * TEST_PWM);
  motorL(TEST_PWM);
  motorR(TEST_PWM);
  delay(STEP_MS);
  motorsStop();
  delay(GAP_MS);

  Serial.print(F("MOTOR TEST 4/4: BOTH back  pwm_out=")); Serial.println(-SIGN_L * TEST_PWM);
  motorL(-TEST_PWM);
  motorR(-TEST_PWM);
  delay(STEP_MS);
  motorsStop();
  Serial.println(F("MOTOR TEST DONE - wheels must have rolled fwd,fwd,fwd,back"));
  Serial.println(F("no motion at all? battery/jumpers; one wheel backward? j/J then M again"));
  lastTick = millis();   // no catch-up tick burst after the ~9 s block (same as 'W')
  prevTickMs = 0;        // skip one dt sample
}

/* ===== Stage 6: crossing detection (on the readLine-corrected values) =====
   readLine() leaves the polarity-corrected values in sensorValues[] (for our
   dark line: 1000-v, line HIGH) in the same tick, so no re-read is needed.
   "On line" = corrected value >= ON_LINE_CAL_MIN. Background corrects to ~0,
   the line to hundreds - so a crossing stripe (line under several sensors at
   once) shows as several sensors high simultaneously. 300 = the same number
   the library itself uses for its on-line test. */
static const int ON_LINE_CAL_MIN = 300;   // corrected value above this = sensor on line
bool isCrossing() {
  uint8_t onLine = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if ((int)sensorValues[i] >= ON_LINE_CAL_MIN) onLine++;
  }
  return onLine >= CROSSING_THRESHOLD;
}

// LED blink while a crossing is held: FOLLOWING = off, SEARCHING = on, crossing = blink
void blinkCrossingLed() {
  if (millis() - lastBlinkMs >= 50) {
    blinkState = !blinkState;
    digitalWrite(PIN_LED, blinkState);
    lastBlinkMs = millis();
  }
}

/* ===== Stage 5: search movement =====
   spin in place toward lastSeenSide - safest first choice: does not carry the robot
   away from where the line was lost; chosen over arc (wrong radius can miss) and
   reverse (needs path memory we do not have). KNOWN LIMITATION for the demo: a spin
   cannot find a line lost "ahead" (sharp bend taken on the outside) - the sweep never
   crosses it; the timeout stop is the fallback in that case.
   Calls driveWheels() DIRECTLY - deliberately bypasses the +-baseSpeed clamp of
   applyCorrection(): a spin needs opposite wheel directions, which the clamp
   would forbid. See the note above deadBandCompensate(). */
void applySearchMovement(int side) {
  if (!driving || dryRun) {
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

/* ===== Stage 5 state machine + Stage 6 crossing override ===== */
void stateMachineStep(int e) {
  lastU = 0;

  if (state == FOLLOWING) {
    // 'F' fakes one lost tick so the search can be exercised on the bench
    bool lost = lineLost(e) || forceSearch;
    forceSearch = false;

    if (lost) {
      pidReset();   // integral/D must not carry garbage into the search or back
      // SEARCH DIRECTION DERIVATION: the sign of the lost error says which
      // side the line was LAST seen on (-2001 -> library returned -1, left;
      // +2001 -> 4001, right). readLine remembers the side for us.
      lastSeenSide = (e < 0) ? -1 : +1;
      state = SEARCHING;
      searchStartTime = millis();
      searchFoundCount = 0;
      searchTimedOut = false;
      searchDisabledMsgDone = false;
      blinkState = false;             // search LED = steady on takes over from blink
      digitalWrite(PIN_LED, HIGH);
      if (trace) {
        Serial.print(F("->SEARCH side="));
        Serial.println(lastSeenSide < 0 ? 'L' : 'R');
      }
      return;
    }

    // ===== Stage 6: crossing override, BEFORE pidStep =====
    if (crossingHandling && isCrossing()) {
      bool fresh = (millis() >= crossingUntilMs);   // new event, not already holding
      crossingUntilMs = millis() + CROSSING_HOLD_MS;
      lastU = lastGoodU;
      if (record) crossingEvent = true;             // t=-2 sentinel for the trace
      if (trace && fresh) Serial.println(F("CROSSING detected"));
      // HOLD last correction, NOT "drive straight": the stripe is short and by the
      // time it is detected the robot has mostly crossed it - keeping the last good
      // correction stays on the arc that was working; straight would push it off
      // if the crossing appeared mid-curve. pidReset NOT called: the line was never
      // lost, the integral may keep accumulating.
      blinkCrossingLed();
      applyCorrection(lastGoodU);
      return;
    }
    if (millis() < crossingUntilMs) {
      // inside the hold window: keep the last good correction, no PID this tick
      lastU = lastGoodU;
      blinkCrossingLed();
      applyCorrection(lastGoodU);
      return;
    }
    digitalWrite(PIN_LED, LOW);   // blink is over: FOLLOWING = LED off
    blinkState = false;

    lastU = pidStep(e);
    applyCorrection(lastU);
    lastGoodU = lastU;            // remembered as the fall-back for the next crossing
    return;
  }

  // ===== SEARCHING =====
  // Bug 1 fix: the line-return check and the return-to-FOLLOWING run REGARDLESS
  // of searchEnabled - with the old structure (early return when disabled) the
  // state could never leave SEARCHING once entered with searchEnabled=0
  // (stuck 45 min at school). searchEnabled now gates ONLY the search motion.
  if (!lineLost(e)) {
    // one good reading is NOT enough: during a fast spin the strip can sweep across
    // the line accidentally (assignment warning) - require N consecutive readings
    searchFoundCount++;
    if (searchFoundCount >= foundReadingsNeeded) {
      pidReset();   // enter following with a clean controller state
      state = FOLLOWING;
      searchTimedOut = false;
      crossingUntilMs = 0;          // stale hold window must not survive the transition
      lastGoodU = 0;
      digitalWrite(PIN_LED, LOW);
      if (trace) Serial.println(F("->FOLLOW"));
      return;
    }
    if (!searchEnabled) {
      // line seen but not yet for enough consecutive ticks: hold still, keep counting
      motorsStop();
      if (!searchDisabledMsgDone) {
        Serial.println(F("SEARCH DISABLED - stopped"));
        searchDisabledMsgDone = true;
      }
    }
    return;
  }

  // still lost
  searchFoundCount = 0;
  if (searchEnabled) {
    // Bug 1 fix: a zero timeout must fall back to the default, never "instant stop"
    unsigned long effectiveTimeout = (searchTimeoutMs == 0) ? SEARCH_TIMEOUT_DEFAULT_MS : searchTimeoutMs;
    if (millis() - searchStartTime > effectiveTimeout) {
      // TIMEOUT DECISION: stop and wait - do not widen, do not reverse. Safe limiting
      // behaviour for the "end of the line" case; a robot must not spin forever.
      motorsStop();
      if (!searchTimedOut) {
        Serial.println(F("SEARCH TIMEOUT - stopped"));
        searchTimedOut = true;
      }
      return;
    }

    applySearchMovement(lastSeenSide);
  } else {
    motorsStop();
    if (!searchDisabledMsgDone) {
      Serial.println(F("SEARCH DISABLED - stopped"));
      searchDisabledMsgDone = true;
    }
  }
}

/* ===== Stage 4: trace buffer (RAM) =====
   rows packed to 7 B: t,e,u int16 + st int8. t wraps at 32.7 s (window 3.2 s),
   e fits +-2001. L/R derivable as base+-u, recomputed at dump. 160 rows = 1120 B. */
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
      // sentinels: t=-1 lost line, t=-2 crossing - L,R printed as 0
      Serial.print(F("0,0,"));
    } else {
      Serial.print(baseSpeed + row.u); Serial.print(',');
      Serial.print(baseSpeed - row.u); Serial.print(',');
    }
    Serial.println(row.st);
  }
  Serial.print(F("# DUMP_DONE rows=")); Serial.println(n);
}

/* ===== fixed-period tick (Stages 3-6 merged) ===== */
static unsigned long dtSum = 0;
static unsigned int  dtCount = 0;
static unsigned int  overCount = 0;
static unsigned long lastDtAvg = 0;   // last CLOSED 500-tick window, for the 'C' config
static unsigned int  lastOver = 0;

void tickController() {
  unsigned long now = millis();
  if (prevTickMs != 0) {
    unsigned long dt = now - prevTickMs;
    dtSum += dt;
    dtCount++;
    if (dt > LOOP_PERIOD_MS + 2) overCount++;
    if (dtCount >= 500) {                       // every 500 ticks = 10 s
      lastDtAvg = dtSum / dtCount;
      lastOver = overCount;
      if (trace) {
        Serial.print(F("TICK dt_avg=")); Serial.print(lastDtAvg);
        Serial.print(F(" over=")); Serial.println(lastOver);
      }
      dtSum = 0; dtCount = 0; overCount = 0;
    }
  }
  prevTickMs = now;

  int e = readError();
  stateMachineStep(e);   // handles BOTH states + crossing override

  if (record) {
    // sentinels: t=-1 lost line, t=-2 crossing (distinct in the CSV/plot)
    if (crossingEvent)       recordTick(-2, (int16_t)e, 0, (int8_t)state);
    else if (lineLost(e))    recordTick(-1, (int16_t)e, 0, (int8_t)state);
    else recordTick((int16_t)(now & 0x7FFF), (int16_t)e, (int16_t)lastU, (int8_t)state);
  }
  crossingEvent = false;
}

/* ===== serial command line parsing (all stages) ===== */
static char lineBuf[24];
static uint8_t lineLen = 0;
uint8_t dbgAll6 = 0;             // 'Z' debug: TLC6 dump + D11 idle reads (non-static: TRSensors.cpp weak-defaults it)

void handleLine(const char* buf) {
  char cmd = buf[0];
  const char* arg = buf + 1;
  while (*arg == ' ') arg++;

  // 'd' alone toggles dry-run; 'd<digit|->' sets kd
  bool argIsNumber = (arg[0] >= '0' && arg[0] <= '9') || arg[0] == '-';

  switch (cmd) {
    case 'r':
      if (dbgAll6) { Serial.print(F("D11 idle before=")); Serial.println(digitalRead(11)); }
      trs.AnalogRead(rawValues);
      if (dbgAll6) { Serial.print(F("D11 idle after=")); Serial.println(digitalRead(11)); }
      printValues("RAW  : ", rawValues);
      break;
    case 'Z':
      dbgAll6 = !dbgAll6;
      Serial.print(F("TLC debug all-6 + D11 idle = ")); Serial.println(dbgAll6);
      break;
    case 'c':
      trs.readCalibrated(sensorValues);
      printValues("CAL  : ", sensorValues);
      break;
    case 'p': {
      // one full diagnostic read through the SAME path the control loop uses
      int e = readError();            // one readLine() call, fills sensorValues
      Serial.print(F("POS cal="));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);   // library-corrected 0..1000 (1000-v for dark line)
        if (i < NUM_SENSORS - 1) Serial.print(',');
      }
      long sum = 0;
      for (uint8_t i = 0; i < NUM_SENSORS; i++) sum += sensorValues[i];
      Serial.print(F(" sum="));     Serial.print(sum);
      Serial.print(F(" error="));   Serial.print(e);
      Serial.print(F(" lost="));    Serial.print(lineLost(e) ? 1 : 0);
      Serial.print(F(" side="));    Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
      Serial.print(F("polarity white_line=")); Serial.print(whiteLine ? 1 : 0);
      Serial.println(F(" (0 = dark line: sensor LOW on line, library inverts 1000-v)"));
      Serial.println(F("library lost test: no sensor > 300; values <= 50 excluded from average"));
      break;
    }
    case 'd':
      if (argIsNumber) { kd = atof(arg); Serial.print(F("kd=")); Serial.println(kd, 3); }
      else { dryRun = !dryRun; Serial.print(F("dryRun=")); Serial.println(dryRun ? 1 : 0); }
      break;
    case 'g':
      // countdown start: 3 s to place the robot - 's' cancels
      Serial.println(F("STARTING IN 3 s - put robot on line, send s to cancel"));
      {
        motorsStop();   // hold still while placing the robot
        driving = false;
        dryRun = true;
        unsigned long t0 = millis();
        bool cancelled = false;
        while (millis() - t0 < 3000) {
          if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\r' || c == '\n' || c == ' ') continue;   // monitor line endings - not a command
            if (c == 's') { cancelled = true; break; }
          }
        }
        if (cancelled) {
          motorsStop();
          Serial.println(F("CANCELLED"));
          break;
        }
        // start from a clean state machine: a stale SEARCHING (e.g. after the
        // timeout stop) must not survive a manual 'g' start
        state = FOLLOWING;
        searchStartTime = millis();   // fresh search window if the line is lost right away
        searchFoundCount = 0;         // found-line counter cleared
        searchTimedOut = false;
        crossingUntilMs = 0;          // stale crossing hold must not survive the start
        blinkState = false;
        digitalWrite(PIN_LED, LOW);
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
    case 's':
      driving = false;
      dryRun = true;
      motorsStop();
      digitalWrite(PIN_LED, LOW);
      blinkState = false;
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
      // D smoothing is now ALWAYS on (LPF on the measurement, alpha 0.25);
      // 'm' just reports it. Kept so old command habits don't hit "?".
      Serial.println(F("D smoothing always on: LPF alpha=0.25 on measurement"));
      break;
    case 't':
      // bare t keeps the trace toggle (same digit/- split as d vs d<val>);
      // the old t<val> lost-sum threshold is GONE: the lost-line test lives
      // inside the library now (no sensor > 300) and is not adjustable
      if (argIsNumber) {
        Serial.println(F("lost-line threshold is fixed inside the library now (on-line test: any sensor > 300)"));
        Serial.println(F("bare t = trace toggle"));
      } else {
        trace = !trace;
        Serial.print(F("trace=")); Serial.println(trace ? 1 : 0);
      }
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
    case 'w':
      // manual polarity override for bench experiments ONLY: the compiled-in
      // value comes from the bench measurement (dark line -> 0)
      whiteLine = !whiteLine;
      Serial.print(F("white_line=")); Serial.println(whiteLine ? 1 : 0);
      Serial.println(F("0 = dark line (library inverts 1000-v); re-check with 'p': line right of centre must give side=R"));
      Serial.println(F("keep after reset: send E (saves polarity)"));
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
      Serial.print(F(" timeoutMs=")); Serial.print(searchTimeoutMs);
      // drive-path gate: wheels only move when BOTH are true (verified from
      // the serial link - a frozen robot with driving=1/dryRun=1 is gated,
      // not stalled)
      Serial.print(F(" driving=")); Serial.print(driving ? 1 : 0);
      Serial.print(F(" dryRun=")); Serial.println(dryRun ? 1 : 0);
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
      if (searchTimeoutMs == 0) {   // Bug 1 fix: T0 must not disable the timeout stop
        searchTimeoutMs = SEARCH_TIMEOUT_DEFAULT_MS;
        Serial.print(F("T0 refused - restored default searchTimeoutMs="));
        Serial.println(searchTimeoutMs);
      } else {
        Serial.print(F("searchTimeoutMs=")); Serial.println(searchTimeoutMs);
      }
      break;
    case 'n':
      foundReadingsNeeded = (unsigned int)atoi(arg);
      Serial.print(F("foundReadingsNeeded=")); Serial.println(foundReadingsNeeded);
      break;
    case 'x':
      crossingHandling = !crossingHandling;
      Serial.print(F("crossingHandling=")); Serial.println(crossingHandling ? 1 : 0);
      if (!crossingHandling) crossingUntilMs = 0;   // cancel a pending hold too
      break;
    case 'C':
      Serial.print(F("CONFIG kp=")); Serial.print(kp, 3);
      Serial.print(F(" ki=")); Serial.print(ki, 4);
      Serial.print(F(" kd=")); Serial.print(kd, 3);
      Serial.print(F(" base=")); Serial.print(baseSpeed);
      Serial.print(F(" iLimit=")); Serial.print(iLimit, 1);
      Serial.print(F(" Dfilter alpha=")); Serial.println(PID_D_FILTER_ALPHA);
      Serial.print(F("  search: enabled=")); Serial.print(searchEnabled ? 1 : 0);
      Serial.print(F(" spin=")); Serial.print(searchSpinSpeed);
      Serial.print(F(" timeoutMs=")); Serial.print(searchTimeoutMs);
      Serial.print(F(" foundNeeded=")); Serial.println(foundReadingsNeeded);
      Serial.print(F("  crossing: handling=")); Serial.print(crossingHandling ? 1 : 0);
      Serial.print(F(" threshold=")); Serial.print(CROSSING_THRESHOLD);
      Serial.print(F(" holdMs=")); Serial.println(CROSSING_HOLD_MS);
      Serial.print(F("  loop: periodMs=")); Serial.print(LOOP_PERIOD_MS);
      Serial.print(F(" dt_avg=")); Serial.print(lastDtAvg);
      Serial.print(F(" over=")); Serial.println(lastOver);
      Serial.print(F("  line: white_line=")); Serial.print(whiteLine ? 1 : 0);
      Serial.println(F(" (0 = dark line)"));
      Serial.print(F("  motors: sign_l=")); Serial.print(SIGN_L);
      Serial.print(F(" sign_r=")); Serial.println(SIGN_R);
      Serial.print(F("  state: state="));
      Serial.print(state == FOLLOWING ? F("FOLLOWING") : F("SEARCHING"));
      Serial.print(F(" side="));
      Serial.print(lastSeenSide < 0 ? 'L' : (lastSeenSide > 0 ? 'R' : '?'));
      Serial.print(F(" searchElapsed="));
      Serial.println(state == SEARCHING ? millis() - searchStartTime : 0);
      break;
    case 'M':
      // motor self-test: wheels lifted, no sensors/PID/search involved
      if (driving && !dryRun) { Serial.println(F("REFUSED - send s first")); break; }
      motorSelfTest();
      break;
    case 'j':
      SIGN_L = -SIGN_L;
      Serial.print(F("sign_l=")); Serial.println(SIGN_L);
      Serial.println(F("re-test with M; keep after reset: send E"));
      break;
    case 'J':
      SIGN_R = -SIGN_R;
      Serial.print(F("sign_r=")); Serial.println(SIGN_R);
      Serial.println(F("re-test with M; keep after reset: send E"));
      break;
    case 'W':
      // refuse a warmup while the wheels are live - a 10 s blind drive is not a demo
      if (driving && !dryRun) { Serial.println(F("REFUSED - send s first")); break; }
      warmupCalibrate();
      break;
    case 'E':
      // explicit save only - EEPROM has ~100k write cycles, never auto-save
      calibrationSaveToEEPROM();
      Serial.println(F("CALIB SAVED TO EEPROM"));
      break;
    case 'L':
      if (calibrationLoadFromEEPROM()) {
        Serial.println(F("CALIB LOADED FROM EEPROM"));
        printValues("MIN 0..4: ", trs.calibratedMin);
        printValues("MAX 0..4: ", trs.calibratedMax);
      } else {
        Serial.println(F("EEPROM: no saved calibration"));
      }
      break;
    case 'X':
      calibrationClearEEPROM();
      Serial.println(F("EEPROM CALIB CLEARED"));
      break;
    case 'h':
      Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go s=stop"));
      Serial.println(F("      Z=TLC-debug: dump all 6 internal values + D11 idle state"));
      Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
      Serial.println(F("      m=report-smoothing t=toggle-trace R=toggle-record D=dump-buffer"));
      Serial.println(F("      z=pidReset h=help"));
      Serial.println(F("State cmds: S=toggle-search-enabled Q=state F=force-search"));
      Serial.println(F("  Search params: a<val>=spinSpeed T<val>=timeoutMs n<val>=foundNeeded"));
      Serial.println(F("Stage 6: x=toggle-crossing-handling C=print-config W=warmup-calibrate w=flip-polarity"));
      Serial.println(F("Motors: M=self-test  j=flip-sign-L  J=flip-sign-R  (verify with M, save with E)"));
      Serial.println(F("EEPROM: E=save-cal  L=load-cal  X=clear-cal"));
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

/* ===== Stage 1/2 helper ===== */
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

  Serial.println(F("[BOOT] LAB2 firmware ready"));
  Serial.print(F("D11 idle at boot=")); Serial.println(digitalRead(11));   // fault hunt: TLC1543 DATA line, no clocking yet
  Serial.println(F("baud=9600 - send h for the full command list"));
  Serial.println(F("LAB2 Stage 6 - test track (TRSensors library refactor)"));

  // EEPROM persistence (Option A): try the saved calibration first; only if it is
  // absent or CRC-invalid do we run the ~10 s hand-rotation calibration
  if (calibrationLoadFromEEPROM()) {
    Serial.println(F("EEPROM: calibration loaded (run X to clear)"));
    printValues("MIN 0..4: ", trs.calibratedMin);
    printValues("MAX 0..4: ", trs.calibratedMax);
    Serial.print(F("white_line=")); Serial.print(whiteLine ? 1 : 0);
    Serial.println(F(" (0 = dark line, from the bench measurement; 'w' flips)"));
  } else {
    Serial.println(F("EEPROM: no saved calibration (run W, then E to save)"));
    Serial.println(F("CALIBRATING... rotate the robot so the strip crosses the line"));
    digitalWrite(PIN_LED, HIGH);
    runCalibration();
    digitalWrite(PIN_LED, LOW);
    Serial.println(F("CALIBRATION DONE"));
    printValues("MIN 0..4: ", trs.calibratedMin);
    printValues("MAX 0..4: ", trs.calibratedMax);
    Serial.print(F("white_line=")); Serial.print(whiteLine ? 1 : 0);
    Serial.println(F(" (0 = dark line, measured 09-11: line LOW ~170-350, background HIGH ~700-900)"));
    Serial.println(F("polarity check: MIN range = dark end (the line), MAX = bright end (background); re-check with p"));
  }

  Serial.print(F("state=FOLLOWING searchEnabled=")); Serial.println(searchEnabled ? 1 : 0);
  Serial.print(F("dryRun=")); Serial.print(dryRun ? 1 : 0);
  Serial.print(F(" driving=")); Serial.println(driving ? 1 : 0);
  Serial.print(F("defaults: kp=")); Serial.print(kp, 3);
  Serial.print(F(" kd=")); Serial.print(kd, 3);
  Serial.print(F(" base=")); Serial.print(baseSpeed);
  Serial.print(F(" search=")); Serial.print(searchEnabled ? 1 : 0);
  Serial.print(F(" Dfilter alpha=")); Serial.println(PID_D_FILTER_ALPHA);
  Serial.print(F("line: white_line=")); Serial.print(whiteLine ? 1 : 0);
  Serial.println(F(" (0 = dark line; 'w' flips, E saves)"));
  Serial.print(F("signs: sign_l=")); Serial.print(SIGN_L);
  Serial.print(F(" sign_r=")); Serial.println(SIGN_R);
  Serial.println(F("new robot? wheels up, send M, fix with j/J, then E to save"));
  Serial.println(F("g starts in 3 s (s cancels) - k/i/d/b retune live"));
  Serial.println(F("at the track: recalibrate with W under the target lighting"));

  motorsStop();   // last line of setup: safe state
}

// fixed-period non-blocking loop, LAB1 pattern: lastTick += period keeps the
// cadence drift-free even if a tick overruns
void loop() {
  readSerialCommands();
  if (millis() - lastTick >= LOOP_PERIOD_MS) {
    lastTick += LOOP_PERIOD_MS;
    tickController();
  }
}
