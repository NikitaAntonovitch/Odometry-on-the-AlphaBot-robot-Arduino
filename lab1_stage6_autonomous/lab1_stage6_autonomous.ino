// iss lab1 - stage 6: autonomous A -> B drive
// the stage 5 program IS the stage 6 program (the guide: "nothing new to write
// beyond merging stages 1-5"): this file is stage5 v1 with logic untouched,
// re-identified for the acceptance session (greeting + menu + this header).
// phase 1 route building, phase 2 one-string autonomous run, phase 3 spread -
// procedure lives in MEASURE_STAGE6.md. execution variant B (guide stage-6
// table): the non-blocking parser + 8-command program queue, so a route string
// longer than the 64 B rx buffer still executes whole.
// forbidden: AlphaBot.h and anything from the waveshare examples - all written from scratch

/* ===== interface speeds =====
   kept in one place, a mismatch here gives the symptom "nothing works" */
static const unsigned long SERIAL_BAUD = 9600;   // same link as stages 2/3/4

/* ===== pin map ===== */
static const uint8_t PIN_ENA  = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // channel A pwm + direction
static const uint8_t PIN_ENB  = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // channel B pwm + direction
static const uint8_t PIN_CNTL = 2,  PIN_CNTR = 3;                 // encoders, INT0 and INT1

static const int8_t SIGN_L = -1;   // floor-verified 14.10: bridge-fwd rolls the robot backward,
static const int8_t SIGN_R = -1;   // both signs flipped; dirL/dirR taken BEFORE this correction

static const int PWM_MAX = 255;    // full duty

/* ===== odometry constants (stage 2 calibration) ===== */
static const uint8_t  SLOTS    = 20;       // 20 slots both discs, CHANGE -> 40 counts/rev
static const uint8_t  N_REV    = 2*SLOTS;
static const float    WHEEL_D  = 65.0f;    // mm, measured across the tyre
static const float    K_MM     = 5.1087f;  // mm per count, calibrated (spread 1.54 %)
static const uint16_t COAST_MS = 2000;     // count window after the last drive pulse
static const uint16_t DEBOUNCE_US = 800;   // isr chatter filter

/* ===== shared motion constants ===== */
static const float    PWM_PER_MMS = 130.0f / 302.0f;  // linear pwm<->speed map (stage 2)
static const int      PWM_MIN     = 85;    // floor-measured under-load dead band (75) + margin
static const uint16_t CTRL_MS     = 20;    // control period, 50 Hz
static const uint16_t STALL_MS    = 1500;  // no-counts abort, same as stages 3/4
static const float    K_SYM       = 3.0f;  // symmetry gain, pwm per mm (floor-tuned 14.10)
static const uint16_t SPD_WIN     = 500;   // speed window ms (stage 2 diagnostic)

/* ===== straight-controller constants (stage 3 v4, unchanged values) ===== */
static const int   PWM_CRUISE_DRV = 130;   // straight operating point, mid linear region
static const float V_CRUISE_DRV   = 302.0f;// mm/s wheel average at that pwm
static const float ACC_MM = 60.0f;         // position-driven accel ramp, mm
static const float DEC_MM = 60.0f;         // decel ramp: slowdown to floor kills the run-on
static const float STOP_TOL_MM = 10.0f;    // 2 counts (stage 2 quantisation floor)
static const int   CORR_MAX_DRV = 30;      // straight symmetry clamp (wider: cruise is higher)
static const float TGT_MIN_MM = 5.0f;      // G range in mm: 0.005..5.000 m
static const float TGT_MAX_MM = 5000.0f;

/* ===== turn-controller constants (stage 4 v3, unchanged values) ===== */
/* track stays PROVISIONAL until the 1440deg x3/dir confirmation session (MEASURE_STAGE4 1a).
   do NOT enter 144 - its scale error hides behind run-on at 45-90deg and blows up multi-rev */
static float trackMm      = 151.0f;   // spin: effective track mm (PROVISIONAL)
static float trackMmPivot = 145.0f;   // pivot: caster skid shrinks the effective track (PROVISIONAL)
static const int   PWM_CRUISE_TRN = 110;  // turns run slower than straights: less run-on
static const float V_CRUISE_TRN   = 110.0f / PWM_PER_MMS;  // wheel mm/s at turn cruise
static const float ACC_DEG = 30.0f;       // angular accel ramp, body degrees
static const float DEC_DEG = 30.0f;       // angular decel ramp
static const float STOP_TOL_DEG = 0.7f;   // ~1.5 counts at the calibrated track
static const int   CORR_MAX_TRN = 20;     // turn symmetry clamp (stage 4 value)
static const int   PULSE_PWM    = 120;    // breakaway pulse: > profile max, breaks stiction
static const uint16_t PULSE_MS  = 150;
static const int   KICK_PWM     = 95;     // active-brake reverse pulse: kills momentum
static const uint16_t KICK_MS   = 60;
static const uint16_t QUIET_MS  = 100;    // coast settle: no new counts = stopped
static const uint16_t COAST_TMO = 1000;   // coast poll timeout (hardware fault guard)
static const int   RETRY_MAX    = 1;      // stall auto-retries before the watchdog aborts

/* ===== protocol constants (stage 5, the new layer) =====
   command string: G<metres> L<deg> R<deg>, separated by anything non-alphanumeric.
   rules decided and recorded in REPORT_STAGE5 sec.2:
   - G: SIGNED metres, negative = backwards (the guide's table). converted to mm here.
   - L/R: direction lives in the LETTER; a negative number is taken as its absolute
     value (same rule as stage 3 F/B and stage 4 L/R - one convention everywhere).
   - ranges: G 0.005..5.000 m (stage 3 envelope), L/R 5..1440 deg (stage 4 envelope).
     out of range = one err line, NO motion, the rest of the string still executes.
   - letters are case-insensitive; any other character is skipped with no side effect.
     diag keys (c b e m p s t x z h K J) are reserved single characters - a junk string
     that happens to contain one gets that diagnostic action (documented trade-off).
   - one RESULT line per command (done=... / tturn=...) + one go acknowledgement at
     motion start - the go/done pair is the stages 3/4 format, kept for continuity. */
static const float ANG_MIN_DEG = 5.0f;
static const float ANG_MAX_DEG = 1440.0f;  // 4 revolutions: multi-rev calibration needs this

/* ===== command queue (execution variant B, guide stage-6 table) =====
   a complete parsed command waits here while a motion runs, so a sequence sent as
   one string executes back to back WITHOUT the new-command-aborts-motion rule of
   stages 3/4 (that rule would eat every command after the first). chosen over
   variant A because the parser is non blocking: chars are consumed as they arrive,
   so the rx buffer alone cannot hold the execution order. capacity: a stage-6 route
   is ~4-8 commands; 8 x 8 B = 64 B RAM on a 2 kB part. overflow = err + drop. */
static const uint8_t QUEUE_CAP = 8;
struct CmdItem {                      // NOTE: not used in any function signature - the
  char  letter;                       // .ino prototype generator would otherwise need it
  float value;                        // declared above every function that touches it
};
static CmdItem cmdQ[QUEUE_CAP];
static uint8_t qHead = 0, qLen = 0;   // ring buffer

bool queuePush(char letter, float value) {
  if (qLen >= QUEUE_CAP) return false;
  cmdQ[(qHead + qLen) % QUEUE_CAP] = { letter, value };
  qLen++;
  return true;
}
bool queuePop(char &letter, float &value) {
  if (qLen == 0) return false;
  letter = cmdQ[qHead].letter;
  value  = cmdQ[qHead].value;
  qHead = (qHead + 1) % QUEUE_CAP;
  qLen--;
  return true;
}
void queueClear() { qHead = 0; qLen = 0; }   // operator abort: stop doing things, all of them
uint8_t queueDepth() { return qLen; }

void execPending(char letter, float v);      // forward: flushPendingCmd dispatches through it

/* ===== encoder state (stage 2/3/4, unchanged) ===== */
static volatile int32_t cntL = 0, cntR = 0;   // signed counts, sign = commanded direction
static volatile int8_t  dirL  = +1, dirR  = +1;
static volatile uint32_t lastDriveMs = 0;
static volatile int32_t snapL = 0, snapR = 0;
static bool benchMode = false;

static volatile uint32_t lastEdgeUsL = 0, lastEdgeUsR = 0;

void isrL() {
  uint32_t t = micros();
  if (t - lastEdgeUsL < DEBOUNCE_US) return;          // chatter, not a slot edge
  lastEdgeUsL = t;
  if (!benchMode && millis() - lastDriveMs >= COAST_MS) return;   // window closed -> freeze
  if (dirL >= 0) cntL++; else cntL--;
}
void isrR() {
  uint32_t t = micros();
  if (t - lastEdgeUsR < DEBOUNCE_US) return;
  lastEdgeUsR = t;
  if (!benchMode && millis() - lastDriveMs >= COAST_MS) return;
  if (dirR >= 0) cntR++; else cntR--;
}

bool encCounting() { return benchMode || (millis() - lastDriveMs) < COAST_MS; }

int32_t atomicRead(volatile int32_t &v) {            // no tearing on a 32 bit value, 8 bit core
  uint8_t s = SREG;
  noInterrupts();
  int32_t t = v;
  SREG = s;                                          // already-off interrupts stay off
  return t;
}
int32_t encCountL() { return encCounting() ? atomicRead(cntL) : atomicRead(snapL); }
int32_t encCountR() { return encCounting() ? atomicRead(cntR) : atomicRead(snapR); }
void snapshotEncoders() { noInterrupts(); snapL = cntL; snapR = cntR; interrupts(); }
void zeroEncoders() { noInterrupts(); cntL = 0; cntR = 0; snapL = 0; snapR = 0; interrupts(); }

/* ===== motor layer (stage 3/4, SIGN handled as verified on the floor) ===== */
void motorL(int pwm) {
  dirL = (pwm >= 0) ? +1 : -1;      // user-intent sign BEFORE the wiring flip
  pwm *= SIGN_L;
  digitalWrite(PIN_IN1, pwm >= 0);
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX));
  if (pwm) lastDriveMs = millis();
}
void motorR(int pwm) {
  dirR = (pwm >= 0) ? +1 : -1;
  pwm *= SIGN_R;
  digitalWrite(PIN_IN3, pwm <  0);
  digitalWrite(PIN_IN4, pwm >= 0);
  analogWrite(PIN_ENB, min(abs(pwm), PWM_MAX));
  if (pwm) lastDriveMs = millis();
}
void coast() {
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
}
void brake() {                                      // dynamic braking: en high + both inputs low
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENB, HIGH);
}

float encMM(int32_t c) { return (float)c * K_MM; }

/* ===== shared motion state =====
   the two controllers are mutually exclusive (a motion starts only when idle), so the
   start counters, control-period and watchdog stamps are ONE pair shared by both */
static int32_t startL = 0, startR = 0;   // counters captured when the motion starts
static uint32_t lastCtrlMs = 0;          // fixed-period bookkeeping
static uint32_t lastProgressMs = 0;      // stall watchdog input
static float   symGain = K_SYM;          // runtime-tunable via K (both controllers)
static float   spdLatestL = 0.0f, spdLatestR = 0.0f;   // for the s key
static bool    telOn = false;            // telemetry toggle, diagnostic only, off for real runs
static uint32_t lastTelMs = 0;

/* ===== straight-line controller (stage 3 v4) =====
   position-driven speed profile + stopping condition + proportional symmetry
   controller on the wheel odometry difference (the assignment-required feedback) */
enum MotState : uint8_t { M_IDLE, M_RUN };
static MotState mot = M_IDLE;
static int8_t  motDir = +1;          // +1 forward, -1 backward
static float   tgtMM = 0.0f;         // commanded distance magnitude, mm
static float   cmdM = 0.0f;          // commanded value in PROTOCOL units (signed metres), for the done line

void stopMotion() {                  // cut the drive, back to idle (no result line on abort)
  brake();
  mot = M_IDLE;
}

void beginDrive(int8_t dir, float mm) {
  startL = encCountL();              // deltas measured from here, no z needed between motions
  startR = encCountR();
  motDir = dir;
  tgtMM  = mm;
  lastCtrlMs = millis();             // first control step a full period after the start
  lastProgressMs = millis();         // stall watchdog armed from motion start
  mot = M_RUN;
}

void controlStep() {                 // every CTRL_MS while M_RUN, never blocks
  int32_t cL = encCountL() - startL;
  int32_t cR = encCountR() - startR;
  if (cL != 0 || cR != 0) lastProgressMs = millis();   // wheels turned -> drive is alive
  else if (millis() - lastProgressMs > STALL_MS) {     // jam / dead band / no power
    stopMotion();
    Serial.print(F("err: stalled - no counts in "));
    Serial.print(STALL_MS);
    Serial.println(F("ms (check battery/dead band), aborting"));
    return;
  }
  float dL = cL * K_MM;              // signed wheel distances, mm
  float dR = cR * K_MM;
  float prog = 0.5f * (dL + dR) * motDir;   // mean progress magnitude toward the target
  float rem  = tgtMM - prog;
  if (rem <= STOP_TOL_MM) {          // stopping condition, tolerance = 2 counts
    stopMotion();
    float got = prog * motDir;       // achieved signed distance, odometry truth
    Serial.print(F("done cmd=G"));   // protocol echo: what was commanded (metres)
    Serial.print(cmdM, 3);
    Serial.print(F("m got="));       // and what odometry achieved (mm, +fwd/-back)
    Serial.print(got, 1);
    Serial.print(F("mm err="));
    Serial.print(got - cmdM * 1000.0f, 1);
    Serial.print(F("mm L="));
    Serial.print((long)cL);
    Serial.print(F("cnt R="));
    Serial.print((long)cR);
    Serial.println(F("cnt"));
    return;
  }
  // speed profile: position driven on both ends, floored at the dead band
  float v = V_CRUISE_DRV;
  float uAcc = prog / ACC_MM;        // 0 at the start -> 1 after ACC_MM of travel
  float uDec = rem  / DEC_MM;        // 1 far from the target -> 0 at it
  float u = min(uAcc, uDec);         // whichever ramp is active wins
  if (u < 1.0f) v = PWM_MIN / PWM_PER_MMS + (V_CRUISE_DRV - PWM_MIN / PWM_PER_MMS) * u;
  int pwm = (int)(v * PWM_PER_MMS + 0.5f);
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  if (pwm > PWM_CRUISE_DRV) pwm = PWM_CRUISE_DRV;
  // symmetry controller: err > 0 when the left wheel travelled more (in commanded sense)
  float err = (dL - dR) * motDir;
  int corr = (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f));   // pwm units, rounded
  if (corr >  CORR_MAX_DRV) corr =  CORR_MAX_DRV;
  if (corr < -CORR_MAX_DRV) corr = -CORR_MAX_DRV;
  int pwmL = pwm - corr;             // left ahead -> left slows
  int pwmR = pwm + corr;             //   and right catches up
  if (pwmL < PWM_MIN) pwmL = PWM_MIN;   // correction must not stall a wheel below the dead band
  if (pwmR < PWM_MIN) pwmR = PWM_MIN;
  motorL(motDir * pwmL);
  motorR(motDir * pwmR);
  if (telOn && millis() - lastTelMs >= 100) {   // opt-in diagnostic, default off
    lastTelMs = millis();
    Serial.print(F("tel prog="));
    Serial.print(prog, 1);
    Serial.print(F(" rem="));
    Serial.print(rem, 1);
    Serial.print(F(" err="));
    Serial.print(err, 1);
    Serial.print(F(" pwmL="));
    Serial.print(pwmL);
    Serial.print(F(" pwmR="));
    Serial.println(pwmR);
  }
}

/* ===== turn controller (stage 4 v3, full stop-quality machine) =====
   kinematics: spin s_w = b*theta/2 per wheel, pivot s_m = b*theta on the moving wheel.
   sign convention (recorded, reused by the protocol): POSITIVE = ccw from above = LEFT */
enum TurnState : uint8_t { T_IDLE, T_RUN, T_PULSE, T_COAST, T_KICK };
enum TurnMode  : uint8_t { M_SPIN, M_PIVOT_L, M_PIVOT_R };
static TurnState trn = T_IDLE;
static uint32_t  phaseStartMs = 0;    // PULSE/KICK/COAST phase timer
static uint32_t  coastLastMs = 0;     // last time coast counts changed
static int32_t   coastSnapL = 0, coastSnapR = 0;
static int32_t   coastL = 0, coastR = 0;           // coast counts folded into odometry
static int8_t    retriesLeft = RETRY_MAX;
static bool      corrected = false;   // one corrective reverse move per turn (no hunting)
static TurnMode  tmode = M_SPIN;
static int8_t    tsign = +1;          // +1 = left (ccw), -1 = right (cw)
static float     tgtDeg = 0.0f;
static float     lastAchievedDeg = 0.0f;           // kept after the turn for the result line

static float getTrack(bool pivot) {    // SINGLE source: targets AND odometry route through here
  return pivot ? trackMmPivot : trackMm;
}

static float wheelTargetMM() {                       // commanded per-wheel distance magnitude
  float rad = tgtDeg * 3.14159265f / 180.0f;
  if (tmode == M_SPIN)  return getTrack(false) * rad / 2.0f;
  return getTrack(true) * rad;                       // pivot: the moving wheel does b*theta
}

static float progressDeg() {                         // achieved angle per odometry, signed ccw+
  int32_t cL = encCountL() - startL + coastL;        // coast counts are real wheel travel
  int32_t cR = encCountR() - startR + coastR;
  float sL = cL * K_MM;
  float sR = cR * K_MM;
  if (tmode == M_SPIN)  return (sR - sL) / getTrack(false) * 180.0f / 3.14159265f;
  if (tmode == M_PIVOT_L) return ( sR / getTrack(true)) * 180.0f / 3.14159265f;
  return                        (-sL / getTrack(true)) * 180.0f / 3.14159265f;
}

float turnAchievedDeg() { return trn == T_RUN ? progressDeg() : lastAchievedDeg; }

void stallAbort() {                                  // watchdog net after the retry budget
  lastAchievedDeg = progressDeg();
  brake();
  trn = T_IDLE;
  Serial.println(F("err: stalled - no counts in 1500ms, aborting turn"));
}

void driveTurn(int pwm) {              // the ONLY place that drives wheels during a turn
  if (pwm < 0) pwm = -pwm;             // pwm is a magnitude: sign comes from mode+tsign
  if (tmode == M_SPIN) {
    int mag = pwm;
    if (mag > PWM_CRUISE_TRN) mag = PWM_CRUISE_TRN;
    if (tsign > 0) { motorL(-mag); motorR(+mag); }   // LEFT: left backward, right forward
    else           { motorL(+mag); motorR(-mag); }
  } else if (tmode == M_PIVOT_L) {
    motorL(0);
    motorR(tsign > 0 ? +pwm : -pwm);
  } else {
    motorR(0);
    motorL(tsign > 0 ? -pwm : +pwm);
  }
}

void driveReverse(int pwm) {           // active brake / corrective move: mirror of driveTurn
  if (pwm < 0) pwm = -pwm;
  if (tmode == M_SPIN) {
    if (tsign > 0) { motorL(+pwm); motorR(-pwm); }
    else           { motorL(-pwm); motorR(+pwm); }
  } else if (tmode == M_PIVOT_L) {
    motorL(0);
    motorR(tsign > 0 ? -pwm : +pwm);
  } else {
    motorR(0);
    motorL(tsign > 0 ? +pwm : -pwm);
  }
}

void startCoastPoll() {                // after the motors stop, keep counting until quiet
  brake();
  coastSnapL = encCountL();
  coastSnapR = encCountR();
  coastLastMs = millis();
  phaseStartMs = millis();
  trn = T_COAST;
}

void turnStart(float deg, uint8_t mode, int8_t sign) {
  startL = encCountL();
  startR = encCountR();
  tmode = (TurnMode)mode;
  tsign = sign;
  tgtDeg = deg;
  coastL = 0; coastR = 0;                            // per-turn coast/retry state resets
  retriesLeft = RETRY_MAX;
  corrected = false;
  lastCtrlMs = millis();                             // shared stamps armed from motion start
  lastProgressMs = millis();
  phaseStartMs = millis();
  trn = T_PULSE;                                     // breakaway pulse first (stiction ~1/10)
  driveTurn(PULSE_PWM);
}

void reportTurn(int32_t cL, int32_t cR) {            // got= is ONLY counts x deg-per-count
  Serial.print(F("tturn cmd="));
  Serial.print(tsign > 0 ? F("L") : F("R"));
  Serial.print(tgtDeg, 1);
  Serial.print(F(" got="));
  Serial.print(lastAchievedDeg, 1);                  // the achieved angle the guide requires
  Serial.print(F("deg L="));
  Serial.print((long)cL);
  Serial.print(F("cnt R="));
  Serial.print((long)cR);
  Serial.print(F("cnt"));
  if (coastL != 0 || coastR != 0) {
    Serial.print(F(" coast="));
    Serial.print((long)(labs(coastL) > labs(coastR) ? coastL : coastR));
    Serial.println(F("cnt"));
  } else {
    Serial.println();
  }
}

void stopTurn() {                                    // stop condition met: kill momentum,
  brake();                                           // coast-count until quiet, then report
  coastSnapL = encCountL();
  coastSnapR = encCountR();
  coastLastMs = millis();
  phaseStartMs = millis();
  trn = T_KICK;
  driveReverse(KICK_PWM);
}

void settlePhase() {                                 // coast-settle poll, folds coast counts,
  int32_t nowL = encCountL(), nowR = encCountR();    // then correction decision or report
  if (nowL != coastSnapL || nowR != coastSnapR) {
    coastL += nowL - coastSnapL;
    coastR += nowR - coastSnapR;
    coastSnapL = nowL; coastSnapR = nowR;
    coastLastMs = millis();
    return;
  }
  if (millis() - coastLastMs < QUIET_MS && millis() - phaseStartMs < COAST_TMO) return;
  // wheels quiet (or timeout - counted as-is): every count since start belongs to got=
  float prog = progressDeg() * tsign;
  float over = prog - tgtDeg;                        // >0 = coast pushed past the target
  float oneCntDeg = (tmode == M_SPIN ? 2.0f : 1.0f) * K_MM / getTrack(tmode != M_SPIN) * 180.0f / 3.14159265f;
  if (!corrected && over >= oneCntDeg * 0.5f) {      // overshot >= half a count: ONE reverse
    corrected = true;
    coastSnapL = encCountL();
    coastSnapR = encCountR();
    coastLastMs = millis();
    phaseStartMs = millis();
    trn = T_KICK;
    driveReverse(KICK_PWM);
    return;
  }
  trn = T_IDLE;
  lastAchievedDeg = progressDeg();                   // got= is a pure function of counts
  reportTurn(encCountL() - startL + coastL, encCountR() - startR + coastR);
}

void turnStep() {                                    // 50 Hz phase machine, never blocks
  if (trn == T_PULSE) {
    if (encCountL() != startL || encCountR() != startR) {   // wheels broke free
      lastCtrlMs = millis();
      lastProgressMs = millis();
      trn = T_RUN;
    } else if (millis() - phaseStartMs >= PULSE_MS) {
      lastCtrlMs = millis();
      lastProgressMs = millis();
      trn = T_RUN;
    } else if (millis() - phaseStartMs > STALL_MS) {
      if (retriesLeft > 0) {
        retriesLeft--;
        phaseStartMs = millis();
        driveTurn(PULSE_PWM);
      } else {
        stallAbort();
      }
    }
    return;
  }
  if (trn == T_COAST) { settlePhase(); return; }
  if (trn == T_KICK) {
    if (millis() - phaseStartMs >= KICK_MS) {
      brake();                                       // stop the reverse drive before polling
      coastL += encCountL() - coastSnapL;            // kick counts are real motion: fold them
      coastR += encCountR() - coastSnapR;
      coastSnapL = encCountL();
      coastSnapR = encCountR();
      coastLastMs = millis();
      phaseStartMs = millis();
      trn = T_COAST;
    }
    return;
  }
  if (trn != T_RUN) return;

  int32_t cL = encCountL() - startL + coastL;
  int32_t cR = encCountR() - startR + coastR;
  if (cL != 0 || cR != 0) lastProgressMs = millis();
  else if (millis() - lastProgressMs > STALL_MS) {
    if (retriesLeft > 0) {
      retriesLeft--;
      phaseStartMs = millis();
      trn = T_PULSE;
      driveTurn(PULSE_PWM);
      return;
    }
    stallAbort();
    return;
  }
  float prog = progressDeg() * tsign;
  float rem  = tgtDeg - prog;
  float vDeg = V_CRUISE_TRN / getTrack(tmode != M_SPIN) * 180.0f / 3.14159265f;
  float uAcc = prog / ACC_DEG;
  float uDec = rem  / DEC_DEG;
  float u = min(uAcc, uDec);
  if (u < 1.0f) {
    float vFloor = PWM_MIN / PWM_PER_MMS / getTrack(tmode != M_SPIN) * 180.0f / 3.14159265f;
    vDeg = vFloor + (vDeg - vFloor) * u;
  }
  if (rem <= STOP_TOL_DEG) {                         // kick + coast poll absorb AND count
    stopTurn();                                      // the run-on, so the allowance is just
    return;                                          // the tolerance (stage 4 design note)
  }
  float vMMs = vDeg / 180.0f * 3.14159265f * getTrack(tmode != M_SPIN);
  int pwm = (int)(vMMs * PWM_PER_MMS + 0.5f);
  if (pwm < PWM_MIN)  pwm = PWM_MIN;
  if (pwm > PWM_CRUISE_TRN) pwm = PWM_CRUISE_TRN;

  if (tmode == M_SPIN) {
    // symmetry: equalise the wheel path MAGNITUDES (guide p.24), not signed difference
    float err = (fabsf((float)cL) - fabsf((float)cR)) * K_MM;   // fabsf: AVR abs is a macro
    int corr = (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f));
    if (corr >  CORR_MAX_TRN) corr =  CORR_MAX_TRN;
    if (corr < -CORR_MAX_TRN) corr = -CORR_MAX_TRN;
    int magL = pwm - corr;
    int magR = pwm + corr;
    if (magL < PWM_MIN) magL = PWM_MIN;              // clamp magnitudes, never the signed pwm
    if (magR < PWM_MIN) magR = PWM_MIN;
    if (tsign > 0) { motorL(-magL); motorR(+magR); }
    else           { motorL(+magL); motorR(-magR); }
  } else if (tmode == M_PIVOT_L) {
    motorL(0);                                       // no symmetry controller in pivot:
    motorR(tsign > 0 ? +pwm : -pwm);                 // one wheel, nothing to equalise
  } else {
    motorR(0);
    motorL(tsign > 0 ? -pwm : +pwm);
  }

  if (telOn && millis() - lastTelMs >= 100) {
    lastTelMs = millis();
    Serial.print(F("tel prog="));
    Serial.print(prog, 1);
    Serial.print(F(" rem="));
    Serial.print(rem, 1);
    Serial.print(F("deg pwm="));
    Serial.println(pwm);
  }
}

/* ===== command execution (protocol layer) ===== */
void execDriveG(float m) {             // protocol G: SIGNED metres, negative = backwards
  float mag = m < 0 ? -m : m;          // range on the MAGNITUDE - the sign is the direction
  float mm = mag * 1000.0f;            // unit decision: protocol metres -> odometry mm
  if (mm < TGT_MIN_MM || mm > TGT_MAX_MM) {
    Serial.println(F("err: G out of range (0.005..5.000 m)"));
    return;
  }
  cmdM = m;                            // remembered for the done line echo (signed)
  beginDrive(m >= 0 ? +1 : -1, mm);    // the SIGN carries the direction
  Serial.print(F("go cmd=G"));
  Serial.print(m, 3);
  Serial.print(F("m dir="));
  Serial.println(m >= 0 ? F("fwd") : F("back"));
}

void execTurn(char letter, float arg) {
  float deg = arg < 0 ? -arg : arg;    // direction lives in the letter (documented rule)
  if (deg < ANG_MIN_DEG || deg > ANG_MAX_DEG) {
    Serial.println(F("err: angle out of range (5..1440 deg)"));
    return;
  }
  turnStart(deg, tmode, letter == 'L' ? +1 : -1);    // tmode: current spin/pivot selection
  Serial.print(F("go cmd="));
  Serial.print(letter);
  Serial.print(deg, 1);
  Serial.print(F("deg mode="));
  Serial.println(tmode == M_SPIN ? F("spin") : (tmode == M_PIVOT_L ? F("pivotL") : F("pivotR")));
}

/* ===== protocol parser =====
   same char state machine as stages 3/4: non blocking, no parseFloat timeout, one
   decimal point, sign anywhere before the digits, flush on terminator or 50 ms quiet */
static char   cmdLetter = 0;         // pending G/L/R/K/J awaiting its number
static bool   numNeg = false;
static long   numAcc = 0;
static long   numFrac = 0;
static int    numFracN = -1;          // -1 = no point seen yet
static bool   numSeen = false;
static uint32_t lastCharMs = 0;

void flushPendingCmd() {
  if (!cmdLetter) return;
  char letter = cmdLetter;
  float v = (float)numAcc;
  if (numFracN > 0) {
    float div = 1.0f;
    for (int i = 0; i < numFracN; i++) div *= 10.0f;
    v += (float)numFrac / div;
  }
  if (numNeg) v = -v;
  cmdLetter = 0; numAcc = 0; numFrac = 0; numFracN = -1; numSeen = false; numNeg = false;
  if (letter == 'K') {                               // diag: symmetry gain setter
    if (v < 0) v = 0;
    symGain = v;
    Serial.print(F("gain="));
    Serial.println(symGain, 2);
    return;                                          // setters act NOW, never queue
  }
  if (letter == 'J') {                               // diag: track setter, CURRENT mode only
    if (v < 50) v = 50;
    if (tmode == M_SPIN) {
      trackMm = v;
      Serial.print(F("track spin="));
    } else {
      trackMmPivot = v;
      Serial.print(F("track pivot="));
    }
    Serial.println(v, 1);
    return;
  }
  // motion commands G/L/R: if a motion is running, the command WAITS in the queue
  // (guide variant B) instead of aborting the running one - a sequence sent as one
  // string must execute whole. setters never queue (they configure the NEXT motion)
  bool busy = (mot == M_RUN) || (trn != T_IDLE);
  if (busy) {
    if (queuePush(letter, v)) return;                // deferred, executed when idle
    Serial.println(F("err: queue full, command dropped"));   // overflow: honest fail
    return;
  }
  execPending(letter, v);                            // idle: start immediately
}

void execPending(char letter, float v) {             // actually starts a motion command
  if (letter == 'G') { execDriveG(v); return; }      // v stays SIGNED: negative = backwards
  execTurn(letter, v);                               // L/R: abs rule inside
}

void dispatchStep() {                                // idle + queued command -> start it
  if (mot == M_RUN || trn != T_IDLE) return;         // still busy
  char letter; float v;
  if (queuePop(letter, v)) execPending(letter, v);   // one per pass; the new motion
}                                                    // gates the next pop until it ends

void handleChar(char ch) {
  if (cmdLetter) {                                   // collecting the argument
    if (ch >= '0' && ch <= '9') {
      if (numFracN >= 0) { numFrac = numFrac * 10 + (ch - '0'); numFracN++; }
      else numAcc = numAcc * 10 + (ch - '0');
      numSeen = true; lastCharMs = millis(); return;
    }
    if (ch == '.') { if (numFracN < 0) numFracN = 0; return; }
    if (ch == '-') { numNeg = !numNeg; return; }
    if (ch == '+' || ch == ' ') return;              // tolerated but meaningless
    flushPendingCmd();                               // any other char ends the number
    if (ch == '\r' || ch == '\n') return;
  }
  lastCharMs = millis();
  switch (ch) {                                      // switch: all letters visible at once,
    case 'G': case 'g':                              // default IS the protocol's skip rule
    case 'L': case 'l':
    case 'R': case 'r':                              // protocol letters, case-insensitive
    case 'K':                                        // diag setters carry numbers too
    case 'J':
      cmdLetter = (ch >= 'a' && ch <= 'z') ? (char)(ch - 'a' + 'A') : ch;
      numAcc = 0; numFrac = 0; numFracN = -1; numSeen = false; numNeg = false;
      break;
    case 'm':                                        // diag: cycle spin -> pivotL -> pivotR
      tmode = (tmode == M_SPIN) ? M_PIVOT_L : (tmode == M_PIVOT_L ? M_PIVOT_R : M_SPIN);
      Serial.print(F("turn mode="));
      Serial.println(tmode == M_SPIN ? F("spin") : (tmode == M_PIVOT_L ? F("pivotL (about left wheel)") : F("pivotR (about right wheel)")));
      break;
    case 'c':                                        // abort + coast: works MID-MOTION and
      if (mot == M_RUN) stopMotion();                // flushes the pending queue - the
      if (trn == T_RUN) stopTurn();                  // operator said stop, not pause; a
      queueClear();                                  // turn finishes its stop sequence and
      coast();                                       // reports the partial angle, a drive
      Serial.println(F("mode=stop pwm=n/a (coast)")); // just stops silently (documented)
      break;
    case 'b':
      if (mot == M_RUN) stopMotion();
      if (trn == T_RUN) stopTurn();
      queueClear();
      brake();
      Serial.println(F("mode=stop pwm=n/a (brake)"));
      break;
    case 't':
      telOn = !telOn;
      Serial.print(F("tel="));
      Serial.println(telOn ? F("on") : F("off"));
      break;
    case 'e': {
      int32_t cl = encCountL(), cr = encCountR();
      Serial.print(F("enc L="));
      Serial.print((long)cl);
      Serial.print(F("cnt "));
      Serial.print(encMM(cl), 1);
      Serial.print(F("mm | R="));
      Serial.print((long)cr);
      Serial.print(F("cnt "));
      Serial.print(encMM(cr), 1);
      Serial.print(F("mm "));
      Serial.println((mot == M_RUN || trn == T_RUN) ? F("running") : (encCounting() ? F("counting") : F("frozen")));
      break;
    }
    case 'z':
      if (mot == M_RUN) stopMotion();                // zeroing under a running controller
      if (trn == T_RUN) stopTurn();                  // breaks it - stop first
      zeroEncoders();
      Serial.println(F("enc zeroed"));
      break;
    case 's':
      Serial.print(F("spd L="));
      Serial.print(spdLatestL, 1);
      Serial.print(F("mm/s R="));
      Serial.print(spdLatestR, 1);
      Serial.println(F("mm/s"));
      break;
    case 'p':
      Serial.print(F("mode="));
      if (mot == M_RUN) {
        Serial.print(F("drive "));
        Serial.print(motDir > 0 ? F("fwd ") : F("back "));
        Serial.println(tgtMM, 1);
      } else if (trn == T_RUN) {
        Serial.print(F("turn "));
        Serial.print(tsign > 0 ? F("L") : F("R"));
        Serial.print(tmode == M_SPIN ? F(" spin ") : F(" pivot "));
        Serial.println(tgtDeg, 1);
      } else {
        Serial.println(F("idle"));
      }
      break;
    case 'x':
      benchMode = !benchMode;
      Serial.print(F("bench="));
      Serial.println(benchMode ? F("on (raw edges, no freeze)") : F("off (coast window gating)"));
      break;
    case 'h': case '?':
      Serial.println(F("== stage6 A-B drive: G<metres> L<deg> R<deg>, e.g. G0.8 L90 G1.2 =="));
      Serial.println(F("G signed metres (negative=back), L/R direction from letter (negative=abs)"));
      Serial.println(F("m turn mode  K<gain> symmetry  J<track mm>  e enc  z zero  s speed"));
      Serial.println(F("p state  t telemetry  x bench  c coast-stop  b brake  h menu"));
      break;
    default:
      break;                                         // unrecognised: skipped, NO side effect
  }
}

void pollPendingCmd() {              // number sent without a terminator still executes
  if (cmdLetter && numSeen && millis() - lastCharMs > 50) flushPendingCmd();
  if (cmdLetter && !numSeen && millis() - lastCharMs > 300) {  // bare letter: reject
    cmdLetter = 0;
    Serial.println(F("err: command without a number"));
  }
}

void setup() {
  // order matters: pin directions first, then the safe drive state, only then the link
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_CNTL, INPUT_PULLUP);   // bias for the open-collector LM393 outputs
  pinMode(PIN_CNTR, INPUT_PULLUP);
  coast();
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("stage6 v1: A-B autonomous | G/L/R protocol | k=5.109 spin=151 pivot=145 [prov]"));
  attachInterrupt(digitalPinToInterrupt(PIN_CNTL), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CNTR), isrR, CHANGE);
  Serial.println(F("h = menu"));
}

void loop() {
  // variant B execution: chars parse AS THEY ARRIVE (non blocking); a motion command
  // that lands while a motion runs waits in the program queue; dispatchStep starts
  // the next one when idle. abort keys c/b are served by the same parser mid-motion
  while (Serial.available() > 0) handleChar((char)Serial.read());
  pollPendingCmd();
  dispatchStep();                    // idle + queued -> next command starts here
  if ((mot == M_RUN || trn != T_IDLE) && millis() - lastCtrlMs >= CTRL_MS) {
    lastCtrlMs = millis();           // fixed control period, 50 Hz, exactly one owner
    if (mot == M_RUN) controlStep();
    else turnStep();
  }
  // speed window bookkeeping (stage 2 diagnostic, kept alive)
  static uint32_t winStart = 0;
  uint32_t t = millis();
  if (t - winStart >= SPD_WIN) {
    static int32_t prevL = 0, prevR = 0;
    int32_t cl = encCountL(), cr = encCountR();
    spdLatestL = (cl - prevL) * K_MM * 1000.0f / (t - winStart);
    spdLatestR = (cr - prevR) * K_MM * 1000.0f / (t - winStart);
    prevL = cl; prevR = cr;
    winStart = t;
  }
  // snapshot when the counting window closes - SILENT in stage 5: the protocol prints
  // result lines only, so the stage-3 settled-odometry line is dropped (documented)
  static bool wasCounting = true;
  bool nowCounting = encCounting();
  if (wasCounting && !nowCounting) snapshotEncoders();
  wasCounting = nowCounting;
}
