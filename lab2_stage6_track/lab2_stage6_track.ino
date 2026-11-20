// iss lab2 - stage 6: FINAL - test track. Stages 1-5 merged + crossing awareness.
// school results from stage 5 (globals still start at 0 for safety):
// // school results: kp=___ ki=___ kd=___ base=___ spin=___ timeout=___ foundNeeded=___
//
// stage map in this file:
//   Stage 1: sensor + calibration (startup + 'W' warmup at the track)
//   Stage 2: readError()/lineLost(), SIGNED int, white_line flip ('w')
//   Stage 3: motor layer (LAB1 map + SIGN flip), clip-only applyCorrection, dead-man g+y
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

/* ===== EEPROM calibration persistence (added post-stage-6) =====
   Layout v3 (2026-11-20: signs + gains added; v2 2026-11-19: lineThresh).
   Address  Size  Content
   -------  ----  ---------------------------------
   0        1     MAGIC = 0xA6  (v3; any other value = no valid data)
   1        1     white_line (0 or 1, as unsigned char; inert since raw-path change)
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
   22       2     lineThresh (int, little-endian)
   24       1     SIGN_L (int8_t as u8: 0xFF = -1, 0x01 = +1)
   25       1     SIGN_R (same encoding)
   26       4     kp (float)
   30       4     ki (float)
   34       4     kd (float)
   38       2     baseSpeed (int)
   40       1     CRC8 checksum (over bytes 1..39)
   -------------------------------
   Total: 41 bytes used, 983 bytes free on UNO (1024 byte EEPROM).
   v1/v2 saves are rejected by the magic change (0xA5/23-25 B layouts never
   match 0xA6) plus per-field range guards - never load garbage.
   Writes happen ONLY on the explicit 'E' command (EEPROM ~100k cycles). */

static const uint8_t  CALIB_EEPROM_MAGIC = 0xA6;   // layout v3 (2026-11-20)
static const int      CALIB_EEPROM_ADDR  = 0;   // start address

// Simple CRC8 (Polynomial 0x07, initial 0x00, no final XOR) - same as Dallas/Maxim 1-Wire CRC.
static uint8_t _crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

static const unsigned long SERIAL_BAUD = 9600;    // baud MUST match monitor
static const uint8_t NUM_SENSORS = 5;             // Stage 1
static const int SETPOINT = 2000;                 // Stage 2
static const int LOST_LIMIT = 2000;               // Stage 2
// raw-ADC reference levels (measured 2026-11-19): black line ~400, grey floor
// ~800, white stripe ~980. Threshold = midpoint black/grey -> grey and white
// get ZERO weight, only the narrow black line drives the position.
static const int BLACK_RAW = 400;
static const int GREY_RAW  = 800;
static int lineThresh = (BLACK_RAW + GREY_RAW) / 2;   // 600; live: t<val>, saved with E
static const int LINE_LOST_MIN_WEIGHT = 30;   // sum of w below this = no line under strip
static int lastGoodLinePos = 2000;            // direction memory for the lost-line sentinel
// POLARITY (flipped 2026-11-19). Diagnostic that forced the flip: line under
// sensor 3 (RIGHT of centre, raw 721 = darkest reading) was reported as
// linePos=1680 LEFT - the weighted average was chasing the bright background
// pixels, so the controller steered AWAY from the dark line into the white gap.
// TRAPS - read before flipping this back:
//  - TRSensors.cpp inverts every value (1000-value) when white_line is FALSE,
//    even though its own header comment claims the opposite (doc-vs-code bug,
//    good demo answer). On this robot the dark line reads LOW raw, so
//    white_line=0 pointed AT the line (HW-verified 09-11) and white_line=1
//    points at the background. If avoidance behaviour ever returns, flip back
//    at runtime with 'w' and verify with 'p' (side must match the real side) -
//    no recompile needed.
//  - 'E' saves white_line into EEPROM byte 1 and setup() auto-loads it BEFORE
//    the defaults print, so a saved value overrides this initializer on every
//    boot (reflashing does NOT reset it). 'X' clears the block; 'W'+'E'
//    re-saves with the polarity currently active.
// 2026-11-19: readError() switched to the raw-threshold path (lineThresh) and
// no longer calls trs.readLine(), so this flag no longer affects line position
// or crossing detection. Kept (still saved by 'E') only for EEPROM byte-1
// layout compatibility.
static bool white_line = true;   // runtime flip: 'w'; persist across resets: 'E'

static const int PWM_MAX = 255;                   // Stage 3
static const int PWM_MIN = 85;                    // LAB1 dead band 75 + margin
static const unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz; period absorbed into ki/kd

// ===== Stage 6: crossing handling =====
// A crossing stripe puts the line under several widely separated sensors at once;
// the weighted average then points at a place where NO line is, and a normal PID
// step would steer the robot off the track. Detection: count sensors "on line".
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
static float kd = 0.3f;
static int   baseSpeed = 90;
static float iLimit = 1000.0f;
static bool  smoothError = true;
static bool  driving = false;
static bool  dryRun   = true;
static bool  trace   = false;
static bool  record  = false;

static float integral = 0.0f;
static int   prevError = 0;
static int   smoothedError = 0;
static unsigned long prevTickMs = 0;

// ===== Stage 6: crossing bookkeeping =====
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

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int rawValues[NUM_SENSORS];
static int lastLinePos = 0;
static int lastU = 0;   // correction of THIS tick, for the trace row

/* ===== Stage 2: measurement (raw-threshold path, replaced trs.readLine() 2026-11-19) =====
   The track is a NARROW BLACK LINE on a WIDE WHITE STRIPE on GREY FLOOR. The
   library's readLine() weighted average cannot separate "partly dark grey" from
   "black line", so the robot centred on the white stripe. This path:
   - weights ONLY dark readings: w = lineThresh - raw (black ~400 -> w ~200);
     grey ~800 and white ~980 are above the threshold and contribute nothing
   - same 0..4000 position scale, same error = linePos - SETPOINT convention
   - calibration NOT used on this path (W/E/L/X still work, just unused here)
   SIGNED int - never change this type, the lost-line sign depends on it. */
int readError() {
  trs.AnalogRead(rawValues);
  long sum = 0;
  long weighted = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int w = lineThresh - (int)rawValues[i];   // dark sensor = big weight
    if (w < 0) w = 0;                         // grey/white floor: zero weight
    sum += w;
    weighted += (long)w * (i * 1000);
  }
  if (sum < LINE_LOST_MIN_WEIGHT) {
    // library sentinel convention kept: -1 last seen LEFT, 4001 last seen RIGHT,
    // so lineLost() and the search-side derivation work unchanged
    lastLinePos = (lastGoodLinePos < SETPOINT) ? -1 : 4001;
  } else {
    lastLinePos = (int)(weighted / sum);
    lastGoodLinePos = lastLinePos;            // direction memory for the sentinel
  }
  return lastLinePos - SETPOINT;
}

bool lineLost(int e) {
  return e > LOST_LIMIT || e < -LOST_LIMIT;
}

/* ===== Stage 4: PID (unchanged) ===== */
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

/* ===== Stage 3: correction split (clip-only, unchanged) ===== */
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

/* ===== Stage 3: motor layer (LAB1 pattern + floor-verified sign flip) ===== */
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
  lastTick = millis();   // no catch-up tick burst after the 10 s block
  prevTickMs = 0;        // skip one dt sample
}

/* ===== EEPROM calibration save/load/clear (added post-stage-6) =====
   Only touched by explicit commands (E save, L load, X clear) and the one
   auto-load attempt in setup() - never from the control loop. */
bool calibrationLoadFromEEPROM() {
  uint8_t magic;
  EEPROM.get(CALIB_EEPROM_ADDR, magic);
  if (magic != CALIB_EEPROM_MAGIC) return false;   // v1(0xA5)/v2(0xA5) never match v3(0xA6)

  // read the full block into temporaries, verify CRC, then commit
  uint8_t wl, sl, sr;
  unsigned int cmin[NUM_SENSORS], cmax[NUM_SENSORS];
  int lt;
  float kpS, kiS, kdS;
  int baseS;
  uint8_t crcStored, crcCalc;

  EEPROM.get(CALIB_EEPROM_ADDR + 1, wl);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.get(CALIB_EEPROM_ADDR + 2 + i * 2, cmin[i]);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.get(CALIB_EEPROM_ADDR + 12 + i * 2, cmax[i]);
  EEPROM.get(CALIB_EEPROM_ADDR + 22, lt);
  EEPROM.get(CALIB_EEPROM_ADDR + 24, sl);
  EEPROM.get(CALIB_EEPROM_ADDR + 25, sr);
  EEPROM.get(CALIB_EEPROM_ADDR + 26, kpS);
  EEPROM.get(CALIB_EEPROM_ADDR + 30, kiS);
  EEPROM.get(CALIB_EEPROM_ADDR + 34, kdS);
  EEPROM.get(CALIB_EEPROM_ADDR + 38, baseS);
  EEPROM.get(CALIB_EEPROM_ADDR + 40, crcStored);

  // CRC8 over bytes 1..39 (wl + cmin + cmax + lt + signs + gains = 39 bytes)
  crcCalc = 0;
  uint8_t* p;
  p = (uint8_t*)&wl;                  for (int i = 0; i < 1;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)cmin;                 for (int i = 0; i < 10; i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)cmax;                 for (int i = 0; i < 10; i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&lt;                  for (int i = 0; i < 2;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kpS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kiS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&kdS;                 for (int i = 0; i < 4;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  p = (uint8_t*)&baseS;               for (int i = 0; i < 2;  i++) crcCalc = _crc8_update(crcCalc, p[i]);
  crcCalc = _crc8_update(crcCalc, sl);
  crcCalc = _crc8_update(crcCalc, sr);

  if (crcCalc != crcStored) return false;   // CRC fail = treat as empty, never load garbage
  // per-field range guards: a v1/v2 block could only get here by matching the
  // v3 magic (impossible: 0xA5 != 0xA6), but these keep ANY corrupted/garbage
  // block out regardless of what wrote it
  if (lt < 0 || lt > 1023) return false;
  if (sl != 1 && sl != 0xFF) return false;   // only +/-1 signs are legal
  if (sr != 1 && sr != 0xFF) return false;
  if (kpS < 0.0f || kpS > 20.0f) return false;
  if (kiS < 0.0f || kiS > 5.0f) return false;
  if (kdS < 0.0f || kdS > 50.0f) return false;
  if (baseS < 0 || baseS > 255) return false;

  // commit only after verification
  white_line = (wl != 0);
  lineThresh = lt;
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

// save calibration, lineThresh, motor signs and PID gains to EEPROM (explicit 'E' only)
void calibrationSaveToEEPROM() {
  EEPROM.put(CALIB_EEPROM_ADDR, CALIB_EEPROM_MAGIC);
  uint8_t wl = white_line ? 1 : 0;
  uint8_t sl = (uint8_t)SIGN_L;
  uint8_t sr = (uint8_t)SIGN_R;
  EEPROM.put(CALIB_EEPROM_ADDR + 1, wl);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.put(CALIB_EEPROM_ADDR + 2 + i * 2,  trs.calibratedMin[i]);
  for (int i = 0; i < NUM_SENSORS; i++) EEPROM.put(CALIB_EEPROM_ADDR + 12 + i * 2, trs.calibratedMax[i]);
  EEPROM.put(CALIB_EEPROM_ADDR + 22, lineThresh);
  EEPROM.put(CALIB_EEPROM_ADDR + 24, sl);
  EEPROM.put(CALIB_EEPROM_ADDR + 25, sr);
  EEPROM.put(CALIB_EEPROM_ADDR + 26, kp);
  EEPROM.put(CALIB_EEPROM_ADDR + 30, ki);
  EEPROM.put(CALIB_EEPROM_ADDR + 34, kd);
  EEPROM.put(CALIB_EEPROM_ADDR + 38, baseSpeed);

  // CRC8 over the same 39 bytes
  uint8_t crc = 0;
  uint8_t* p;
  p = (uint8_t*)&wl;                  for (int i = 0; i < 1;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)trs.calibratedMin;    for (int i = 0; i < 10; i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)trs.calibratedMax;    for (int i = 0; i < 10; i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&lineThresh;          for (int i = 0; i < 2;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&kp;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&ki;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&kd;                  for (int i = 0; i < 4;  i++) crc = _crc8_update(crc, p[i]);
  p = (uint8_t*)&baseSpeed;           for (int i = 0; i < 2;  i++) crc = _crc8_update(crc, p[i]);
  crc = _crc8_update(crc, sl);
  crc = _crc8_update(crc, sr);
  EEPROM.put(CALIB_EEPROM_ADDR + 40, crc);
}

// mark EEPROM calibration as invalid (explicit 'X' only)
void calibrationClearEEPROM() {
  uint8_t bad = 0xFF;
  EEPROM.put(CALIB_EEPROM_ADDR, bad);   // any value != 0xA5 means "no data"
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

/* ===== Stage 6: crossing detection (raw-based, updated 2026-11-19) =====
   readError() fills rawValues[] right before stateMachineStep() runs in the
   same tick, so no re-read is needed. "On line" = raw BELOW lineThresh:
   grey (~800) and white (~980) never count, the black line (~400) does -
   same semantics as the old calibrated >300 test, same hold logic. */
bool isCrossing() {
  uint8_t onLine = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if ((int)rawValues[i] < lineThresh) onLine++;
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

/* ===== Stage 5: search movement (unchanged) =====
   spin in place toward lastSeenSide - safest first choice: does not carry the robot
   away from where the line was lost; chosen over arc (wrong radius can miss) and
   reverse (needs path memory we do not have). KNOWN LIMITATION for the demo: a spin
   cannot find a line lost "ahead" (sharp bend taken on the outside) - the sweep never
   crosses it; the timeout stop is the fallback in that case. */
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
      // SEARCH DIRECTION DERIVATION: the sign of the out-of-range error says which
      // side the line was LAST seen on (-1 -> library returned -1, left; +1 -> 4001)
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
      int e = readError();   // raw-threshold path fills rawValues itself - print THAT read
      Serial.print(F("POS raw="));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(rawValues[i]);
        if (i < NUM_SENSORS - 1) Serial.print(',');
      }
      Serial.print(F(" linePos=")); Serial.print(lastLinePos);
      Serial.print(F(" error="));   Serial.print(e);
      Serial.print(F(" lost="));    Serial.print(lineLost(e) ? 1 : 0);
      Serial.print(F(" side="));    Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
      Serial.print(F("thr="));      Serial.print(lineThresh);
      Serial.print(F(" sumW>=30 required for linePos (lost sentinels -1/4001)"));
      Serial.println();
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
      smoothError = !smoothError;
      Serial.print(F("smoothError=")); Serial.println(smoothError ? 1 : 0);
      break;
    case 't':
      // t<val> sets the raw threshold; bare t keeps the trace toggle (same
      // digit/- split as d vs d<val>)
      if (argIsNumber) {
        int v = atoi(arg);
        if (v < 0) v = 0;          // 0 = every sensor weighs -> meaningless, refuse
        if (v > 1023) v = 1023;    // above ADC max = nothing is dark -> always lost
        lineThresh = v;
        Serial.print(F("lineThresh=")); Serial.println(lineThresh);
        Serial.println(F("verify with p: black ~400 grey ~800 white ~980"));
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
      white_line = !white_line;
      Serial.print(F("white_line=")); Serial.println(white_line ? 1 : 0);
      Serial.println(F("re-check with 'p': line right of centre must give side=R"));
      Serial.println(F("keep after reset: send E (saves polarity with calibration)"));
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
      Serial.print(F(" smoothError=")); Serial.println(smoothError ? 1 : 0);
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
      Serial.print(F("  line: lineThresh=")); Serial.print(lineThresh);
      Serial.print(F(" white_line=")); Serial.println(white_line ? 1 : 0);
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
      Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go(+y) s=stop"));
      Serial.println(F("      Z=TLC-debug: dump all 6 internal values + D11 idle state"));
      Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
      Serial.println(F("      m=toggle-smoothing t=toggle-trace t<val>=lineThresh R=toggle-record D=dump-buffer"));
      Serial.println(F("      z=pidReset h=help"));
      Serial.println(F("State cmds: S=toggle-search-enabled Q=state F=force-search"));
      Serial.println(F("  Search params: a<val>=spinSpeed T<val>=timeoutMs n<val>=foundNeeded"));
      Serial.println(F("Stage 6: x=toggle-crossing-handling C=print-config W=warmup-calibrate w=flip-white-line"));
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

/* ===== Stage 1/2 helper (kept verbatim) ===== */
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
  Serial.println(F("baud=9600 cmds: c e s x v r k#.# i#.# d#.# b# p#"));
  Serial.println(F("LAB2 Stage 6 - test track"));

  // EEPROM persistence (Option A): try the saved calibration first; only if it is
  // absent or CRC-invalid do we run the ~10 s hand-rotation calibration
  if (calibrationLoadFromEEPROM()) {
    Serial.println(F("EEPROM: calibration loaded (run X to clear)"));
    printValues("MIN 0..4: ", trs.calibratedMin);
    printValues("MAX 0..4: ", trs.calibratedMax);
  } else {
    Serial.println(F("EEPROM: no saved calibration (run W, then E to save)"));
    Serial.println(F("CALIBRATING... rotate the robot so the strip crosses the line"));
    digitalWrite(PIN_LED, HIGH);
    runCalibration();
    digitalWrite(PIN_LED, LOW);
    Serial.println(F("CALIBRATION DONE"));
    printValues("MIN 0..4: ", trs.calibratedMin);
    printValues("MAX 0..4: ", trs.calibratedMax);
  }

  Serial.println(F("Cmds: r c p d=g-toggle-dryrun g=go(+y) s=stop"));
  Serial.println(F("      k<val>=kp i<val>=ki d<val>=kd b<val>=base l<val>=iLimit"));
  Serial.println(F("      m=toggle-smoothing t=toggle-trace t<val>=lineThresh R=toggle-record D=dump-buffer"));
  Serial.println(F("      z=pidReset h=help"));
  Serial.println(F("State cmds: S=toggle-search-enabled Q=state F=force-search"));
  Serial.println(F("  Search params: a<val>=spinSpeed T<val>=timeoutMs n<val>=foundNeeded"));
  Serial.println(F("Stage 6: x=toggle-crossing-handling C=print-config W=warmup-calibrate w=flip-white-line"));
  Serial.println(F("Motors: M=self-test  j=flip-sign-L  J=flip-sign-R  (verify with M, save with E)"));
  Serial.println(F("EEPROM: E=save-cal  L=load-cal  X=clear-cal"));
  Serial.print(F("state=FOLLOWING searchEnabled=")); Serial.println(searchEnabled ? 1 : 0);
  Serial.print(F("dryRun=")); Serial.print(dryRun ? 1 : 0);
  Serial.print(F(" driving=")); Serial.println(driving ? 1 : 0);
  Serial.print(F("defaults: kp=")); Serial.print(kp, 3);
  Serial.print(F(" kd=")); Serial.print(kd, 3);
  Serial.print(F(" base=")); Serial.print(baseSpeed);
  Serial.print(F(" search=")); Serial.print(searchEnabled ? 1 : 0);
  Serial.print(F(" smoothing=")); Serial.println(smoothError ? 1 : 0);
  Serial.print(F(" white_line=")); Serial.print(white_line ? 1 : 0);
  Serial.print(F(" lineThresh=")); Serial.println(lineThresh);
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
