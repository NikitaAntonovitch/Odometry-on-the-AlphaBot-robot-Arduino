// iss lab1 - stage 4: turn controller for a given angle
// stage 3 straight-line controller extended with turns: spin (both wheels counter-rotate)
// and pivot (one wheel planted). profile generator + stopping condition + symmetry
// controller equalising wheel-distance MAGNITUDES (spin only - pivot drives one wheel).
// forbidden: AlphaBot.h and anything from the waveshare examples - all written from scratch

/* ===== interface speeds ===== */
static const unsigned long SERIAL_BAUD = 9600;   // same link as stage 2/3

/* ===== pin map ===== */
static const uint8_t PIN_ENA  = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // channel A pwm + direction
static const uint8_t PIN_ENB  = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // channel B pwm + direction
static const uint8_t PIN_CNTL = 2,  PIN_CNTR = 3;                 // encoders, INT0 and INT1

static const int8_t SIGN_L = -1;   // floor-verified 14.10: bridge-fwd rolls the robot backward,
static const int8_t SIGN_R = -1;   // both signs flipped; dirL/dirR taken BEFORE this correction

static const int PWM_MAX = 255;

/* ===== odometry constants (stage 2 calibration) ===== */
static const uint8_t  SLOTS   = 20;
static const uint8_t  N_REV   = 2*SLOTS;
static const float    WHEEL_D = 65.0f;
static const float    K_MM    = 5.1087f;   // mm per count, calibrated (stage 2, spread 1.54 %)

/* ===== turn constants =====
   track b: the value in dtheta = (sR - sL)/b is the EFFECTIVE track - contact-point shift,
   lateral slip in spin turns and caster friction make it differ from a tape measurement,
   so it is a CALIBRATION parameter (multi-rev procedure, MEASURE_STAGE4 sec.1), not a tape one.
   start value = tape estimate of the AlphaBot (~160 mm); recalibrated on the robot. */
static const float TRACK_MM  = 160.0f;    // effective track, replaced by the calibrated value
static const int   PWM_CRUISE = 110;      // turns run slower than straights: less run-on,
                                          // smaller angular overshoot, still well above the 85 floor
static const float V_CRUISE   = 110.0f / (130.0f / 302.0f);  // mm/s at that pwm via the stage-2 map
static const float PWM_PER_MMS = 130.0f / 302.0f;            // linear pwm<->speed map (stage 2)
static const int   PWM_MIN    = 85;       // floor-measured under-load dead band (75) + margin
static const float ACC_DEG = 30.0f;       // angular accel ramp: degrees of turn used to spin up
static const float DEC_DEG = 30.0f;       // angular decel ramp: last degrees slow to the floor
static const float STOP_TOL_DEG = 0.7f;   // stopping condition, set from the resolution work-out
                                          // (report sec.2: 1 count ~ 0.46 deg spin) - just above
static const float BRAKE_ALLOW_DEG = 2.5f;// brake run-on in body degrees, empirically measured
                                          // in sim (2.5-3.2 deg at the stop speed); the
                                          // profile enters the stop from the DEC ramp floor
static const uint16_t CTRL_MS  = 20;      // control period, 50 Hz
static const uint16_t STALL_MS = 1500;    // no-counts abort, same as stage 3

/* ===== stop-quality constants (session 15.10: coast was uncounted ~3-6 cnt/stop, first turn
   after idle stalled at pwm 85 - stiction; evidence in REPORT_STAGE4 sec.4) ===== */
static const int   PULSE_PWM    = 120;    // breakaway pulse duty: above profile max (110) so
                                          // static friction breaks on the first turn after idle
static const uint16_t PULSE_MS  = 150;    // breakaway pulse length ms: enough to unstick, short
                                          // enough to add at most ~1 count of travel
static const int   KICK_PWM     = 95;     // active-brake reverse pulse duty: just above the
                                          // motion threshold - kills momentum, minimal overshoot
static const uint16_t KICK_MS   = 60;     // active-brake reverse pulse length, ms
static const uint16_t QUIET_MS  = 100;    // coast settle: no new counts this long = stopped
static const uint16_t COAST_TMO = 1000;   // coast polling timeout ms (hardware fault guard)
static const int   RETRY_MAX    = 1;      // stall auto-retries: one breakaway + retry before the
                                          // watchdog aborts (stiction ~1/10 starts, retry fixes it)
static const float K_SYM  = 3.0f;         // magnitude-equalising gain (spin only), pwm per mm
static const int   CORR_MAX = 20;         // clamp; turn wheels both carry drive, headroom is wider
static const float ANG_MIN_DEG = 5.0f;    // argument range rules (report sec.6)
static const float ANG_MAX_DEG = 1440.0f; // 4 revolutions - multi-rev calibration needs this

/* ===== encoder state (stage 2/3, unchanged) ===== */
static volatile int32_t cntL = 0, cntR = 0;
static volatile int8_t  dirL  = +1, dirR  = +1;
static volatile uint32_t lastDriveMs = 0;
static volatile int32_t snapL = 0, snapR = 0;
static bool benchMode = false;

static volatile uint32_t lastEdgeUsL = 0, lastEdgeUsR = 0;

void isrL() {
  uint32_t t = micros();
  if (t - lastEdgeUsL < 800) return;
  lastEdgeUsL = t;
  if (!benchMode && millis() - lastDriveMs >= 2000) return;
  if (dirL >= 0) cntL++; else cntL--;
}
void isrR() {
  uint32_t t = micros();
  if (t - lastEdgeUsR < 800) return;
  lastEdgeUsR = t;
  if (!benchMode && millis() - lastDriveMs >= 2000) return;
  if (dirR >= 0) cntR++; else cntR--;
}

bool encCounting() { return benchMode || (millis() - lastDriveMs) < 2000; }

int32_t atomicRead(volatile int32_t &v) {
  uint8_t s = SREG;
  noInterrupts();
  int32_t t = v;
  SREG = s;
  return t;
}
int32_t encCountL() { return encCounting() ? atomicRead(cntL) : atomicRead(snapL); }
int32_t encCountR() { return encCounting() ? atomicRead(cntR) : atomicRead(snapR); }
void snapshotEncoders() { noInterrupts(); snapL = cntL; snapR = cntR; interrupts(); }
void zeroEncoders() { noInterrupts(); cntL = 0; cntR = 0; snapL = 0; snapR = 0; interrupts(); }

/* ===== motor layer (stage 3, SIGN handled as verified on the floor) ===== */
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
void brake() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENB, HIGH);
}

float encMM(int32_t c) { return (float)c * K_MM; }

/* ===== turn controller =====
   kinematics (derivation in the report, sec.1):
     spin turn:   dtheta = (sR - sL)/b, sR = -sL = b*theta/2  -> each wheel travels b*|theta|/2
     pivot turn:  one wheel planted, s_planted = 0, the other = b*theta -> HALF the spin path,
                  but the sensor counts the FULL arc on one wheel -> pivot resolves angles
                  TWICE as finely per count (one wheel does the whole arc)
   SIGN CONVENTION (recorded, stage 5 will reuse it):
     POSITIVE angle = counter-clockwise seen from above = robot turns LEFT.
     Command letter carries the direction (L/R); a negative number after L or R
     is taken as its absolute value - same rule as F/B in stage 3. */
enum TurnState : uint8_t { T_IDLE, T_RUN, T_PULSE, T_COAST, T_KICK };
enum TurnMode  : uint8_t { M_SPIN, M_PIVOT_L, M_PIVOT_R };
static TurnState trn = T_IDLE;
static uint32_t  phaseStartMs = 0;    // PULSE/KICK/COAST phase timer
static uint32_t  coastLastMs = 0;     // last time coast counts changed
static int32_t   coastSnapL = 0, coastSnapR = 0;   // counts at coast-poll start
static int32_t   coastL = 0, coastR = 0;           // coast counts folded into odometry
static int8_t    retriesLeft = RETRY_MAX;          // stall auto-retry budget
static bool      corrected = false;   // one corrective reverse move per turn (no hunting)
static TurnMode  tmode = M_SPIN;
static int8_t    tsign = +1;          // +1 = left (ccw), -1 = right (cw)
static float     tgtDeg = 0.0f;
static int32_t   startL = 0, startR = 0;
static uint32_t  lastCtrlMs = 0;
static uint32_t  lastProgressMs = 0;
static float     lastAchievedDeg = 0.0f;             // kept after the turn for the result line

static float     trackMm = TRACK_MM;  // runtime-tunable for the calibration procedure
static float     symGain = K_SYM;
static bool      telOn = false;
static uint32_t  lastTelMs = 0;
static float     spdLatestL = 0.0f, spdLatestR = 0.0f;

/* per-mode helpers: progress in DEGREES and per-wheel targets in mm
   spin:       progress = b/(2k) * (|sL| + |sR|)/2 /b*2 ... implemented directly:
               theta = (sR - sL)/b with signs; progress magnitude = |theta|
               per-wheel distance target = b*|theta_tgt|/2
   pivot L:    left planted, right does the arc: theta = sR/b (sign by direction)
   pivot R:    right planted, left does the arc: theta = -sL/b */
static float wheelTargetMM() {                       // commanded per-wheel distance magnitude
  float rad = tgtDeg * 3.14159265f / 180.0f;
  if (tmode == M_SPIN)  return trackMm * rad / 2.0f;
  return trackMm * rad;                              // pivot: the moving wheel does b*theta
}

static float progressDeg() {                         // achieved angle per odometry, signed ccw+
  int32_t cL = encCountL() - startL + coastL;        // coast counts are real wheel travel: they
  int32_t cR = encCountR() - startR + coastR;        // belong to odometry AND to got= (F2/F5)
  float sL = cL * K_MM;                              // signed wheel paths, mm (+ = user fwd)
  float sR = cR * K_MM;
  if (tmode == M_SPIN)  return (sR - sL) / trackMm * 180.0f / 3.14159265f;
  if (tmode == M_PIVOT_L) return ( sR / trackMm) * 180.0f / 3.14159265f;   // left planted
  return                        (-sL / trackMm) * 180.0f / 3.14159265f;   // right planted
}

float turnAchievedDeg() { return trn == T_RUN ? progressDeg() : lastAchievedDeg; }

void stallAbort() {                                  // watchdog final net: reached only after
  lastAchievedDeg = progressDeg();                   // the retry budget is spent (F6)
  brake();
  trn = T_IDLE;
  Serial.println(F("err: stalled - no counts in 1500ms, aborting turn"));
}

void driveTurn(int pwm) {              // the ONLY place that drives wheels during a turn;
  if (pwm < 0) pwm = -pwm;             // pwm is a magnitude: motion sign comes from mode+tsign
  if (tmode == M_SPIN) {
    int mag = pwm;
    if (mag > PWM_CRUISE) mag = PWM_CRUISE;
    if (tsign > 0) { motorL(-mag); motorR(+mag); }   // LEFT turn: left backward, right forward
    else           { motorL(+mag); motorR(-mag); }   // right turn: mirrored
  } else if (tmode == M_PIVOT_L) {
    motorL(0);
    motorR(tsign > 0 ? +pwm : -pwm);                 // ccw = right wheel forward
  } else {                                           // M_PIVOT_R: right planted, left arcs
    motorR(0);
    motorL(tsign > 0 ? -pwm : +pwm);                 // ccw = left wheel backward
  }
}

void driveReverse(int pwm) {           // active brake + corrective move: OPPOSITE of the turn
  if (pwm < 0) pwm = -pwm;             // direction, same per-mode drive pattern
  if (tmode == M_SPIN) {
    if (tsign > 0) { motorL(+pwm); motorR(-pwm); }
    else           { motorL(-pwm); motorR(+pwm); }
  } else if (tmode == M_PIVOT_L) {
    motorL(0);
    motorR(tsign > 0 ? -pwm : +pwm);                 // mirror of the forward arc
  } else {
    motorR(0);
    motorL(tsign > 0 ? +pwm : -pwm);
  }
}

void startCoastPoll() {                // F2: after the motors stop, keep counting until quiet;
  brake();                             // coast counts fold into odometry (progressDeg)
  coastSnapL = encCountL();
  coastSnapR = encCountR();
  coastLastMs = millis();
  phaseStartMs = millis();
  trn = T_COAST;
}

void turnStart(float deg, uint8_t mode, int8_t sign) {
  startL = encCountL();                              // deltas from here, no z between turns
  startR = encCountR();
  tmode = (TurnMode)mode;
  tsign = sign;
  tgtDeg = deg;
  coastL = 0; coastR = 0;                            // per-turn coast/retry state resets
  retriesLeft = RETRY_MAX;
  corrected = false;

  // F1 breakaway pulse: the first turn after idle must break static friction - the profile
  // floor alone stalls ~1 in 10 starts (measured). T_PULSE runs the stronger duty for
  // PULSE_MS, then the trapezoidal profile takes over with the wheels already moving.
  lastCtrlMs = millis();
  lastProgressMs = millis();
  phaseStartMs = millis();
  trn = T_PULSE;
  driveTurn(PULSE_PWM);
}

void reportTurn(int32_t cL, int32_t cR) {            // F5: got= is ONLY counts x deg-per-count
  Serial.print(F("tturn cmd="));                     // (coast included) - deterministic in the
  Serial.print(tsign > 0 ? F("L") : F("R"));         // counts printed on the same line
  Serial.print(tgtDeg, 1);
  Serial.print(F(" got="));
  Serial.print(lastAchievedDeg, 1);
  Serial.print(F("deg L="));
  Serial.print((long)cL);
  Serial.print(F("cnt R="));
  Serial.print((long)cR);
  Serial.print(F("cnt"));
  if (coastL != 0 || coastR != 0) {                  // optional field at the END: parsers that
    Serial.print(F(" coast="));                      // match the prefix keep working
    Serial.print((long)(labs(coastL) > labs(coastR) ? coastL : coastR));
    Serial.println(F("cnt"));
  } else {
    Serial.println();
  }
}

void stopTurn() {                                    // stopping condition met: kill momentum
  brake();                                           // with the active brake pulse (F3), then
  coastSnapL = encCountL();                          // coast-count until quiet (F2) and report
  coastSnapR = encCountR();
  coastLastMs = millis();
  phaseStartMs = millis();
  trn = T_KICK;
  driveReverse(KICK_PWM);                            // short reverse pulse against the motion
}

void settlePhase() {                                 // shared coast-settle step: polls the
  int32_t nowL = encCountL(), nowR = encCountR();    // counters until QUIET_MS silent or the
  if (nowL != coastSnapL || nowR != coastSnapR) {    // coast timeout, folds coast into odometry,
    coastL += nowL - coastSnapL;                     // then runs the (single) corrective move
    coastR += nowR - coastSnapR;                     // decision (F2) or reports (F5)
    coastSnapL = nowL; coastSnapR = nowR;
    coastLastMs = millis();
    return;
  }
  if (millis() - coastLastMs < QUIET_MS && millis() - phaseStartMs < COAST_TMO) return;
  // wheels quiet (or timeout - counted as-is): decide correction vs report
  // F5 bookkeeping rule: EVERY count since startL/R belongs to the turn - kick counts and
  // coast counts alike. Nothing is zeroed or discarded, so got= stays a deterministic
  // function of the total counts.
  float prog = progressDeg() * tsign;
  float over = prog - tgtDeg;                        // >0 = coast pushed past the target
  float oneCntDeg = (tmode == M_SPIN ? 2.0f : 1.0f) * K_MM / trackMm * 180.0f / 3.14159265f;
  if (!corrected && over >= oneCntDeg * 0.5f) {      // overshot by >= half a count: ONE
    corrected = true;                                // corrective reverse pulse (capped - no
    coastSnapL = encCountL();                        // hunting); its counts fold via T_KICK
    coastSnapR = encCountR();
    coastLastMs = millis();
    phaseStartMs = millis();
    trn = T_KICK;
    driveReverse(KICK_PWM);
    return;
  }
  trn = T_IDLE;                                      // settled - report with the FULL count set:
  lastAchievedDeg = progressDeg();                   // got= is a pure function of counts (F5)
  reportTurn(encCountL() - startL + coastL, encCountR() - startR + coastR);
}

void turnStep() {                                    // 50 Hz phase machine, never blocks
  // ---- T_PULSE: F1 breakaway, full pulse duty, watchdog still guarded ----
  if (trn == T_PULSE) {
    if (encCountL() != startL || encCountR() != startR) {   // wheels broke free
      lastCtrlMs = millis();
      lastProgressMs = millis();
      trn = T_RUN;
    } else if (millis() - phaseStartMs >= PULSE_MS) {       // pulse spent: try the profile,
      lastCtrlMs = millis();                                // wheels usually unstuck by now
      lastProgressMs = millis();
      trn = T_RUN;
    } else if (millis() - phaseStartMs > STALL_MS) {        // pulse must respect the watchdog
      if (retriesLeft > 0) {                                // F6: one retry, then abort
        retriesLeft--;
        phaseStartMs = millis();
        driveTurn(PULSE_PWM);
      } else {
        stallAbort();
      }
    }
    return;
  }
  // ---- T_COAST: settle poll (F2) ----
  if (trn == T_COAST) { settlePhase(); return; }
  // ---- T_KICK: reverse pulse running, hand over to coast poll when done ----
  if (trn == T_KICK) {
    if (millis() - phaseStartMs >= KICK_MS) {
      brake();                                       // STOP the reverse drive before polling:
      coastL += encCountL() - coastSnapL;            // kick counts are real motion: fold them
      coastR += encCountR() - coastSnapR;            // (an un-stopped kick kept driving while
      coastSnapL = encCountL();                      // settlePhase polled - fixed in sim)
      coastSnapR = encCountR();
      coastLastMs = millis();
      phaseStartMs = millis();
      trn = T_COAST;                                 // settle again; quiet -> report (or, if
    }                                                // the kick overshot, ONE more correction
                                                     // is allowed by the corrected flag)
    return;
  }
  if (trn != T_RUN) return;                          // T_IDLE / stray: nothing to do

  // ---- T_RUN: trapezoidal profile ----
  int32_t cL = encCountL() - startL + coastL;
  int32_t cR = encCountR() - startR + coastR;
  if (cL != 0 || cR != 0) lastProgressMs = millis();
  else if (millis() - lastProgressMs > STALL_MS) {   // jam / dead band / no power
    if (retriesLeft > 0) {                           // F6: one breakaway + retry before abort
      retriesLeft--;
      phaseStartMs = millis();
      trn = T_PULSE;
      driveTurn(PULSE_PWM);
      return;
    }
    stallAbort();                                    // watchdog kept as the final net
    return;
  }
  float prog = progressDeg() * tsign;                // magnitude toward the target
  float rem  = tgtDeg - prog;
  // angular speed profile (unchanged shape); computed before the stop check
  float vDeg = V_CRUISE / trackMm * 180.0f / 3.14159265f;   // deg/s at cruise pwm
  float uAcc = prog / ACC_DEG;
  float uDec = rem  / DEC_DEG;
  float u = min(uAcc, uDec);
  if (u < 1.0f) {
    float vFloor = PWM_MIN / PWM_PER_MMS / trackMm * 180.0f / 3.14159265f;
    vDeg = vFloor + (vDeg - vFloor) * u;
  }
  // stopping condition: the kick (F3) plus the coast poll (F2) absorb the run-on and COUNT
  // it, so the allowance shrinks to the tolerance - the old fixed run-on estimate is gone.
  if (rem <= STOP_TOL_DEG) {
    stopTurn();
    return;
  }
  float vMMs = vDeg / 180.0f * 3.14159265f * trackMm;       // wheel speed for the current rate
  int pwm = (int)(vMMs * PWM_PER_MMS + 0.5f);
  if (pwm < PWM_MIN)  pwm = PWM_MIN;
  if (pwm > PWM_CRUISE) pwm = PWM_CRUISE;

  if (tmode == M_SPIN) {
    // symmetry controller: equalise the MAGNITUDES of the two wheel paths (guide p.24).
    // err > 0 when the left wheel path magnitude runs ahead of the right -> slow left.
    float err = (fabsf((float)cL) - fabsf((float)cR)) * K_MM;   // fabsf: the AVR core defines
                                                                // abs as a macro, std::abs breaks
    int corr = (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f));
    if (corr >  CORR_MAX) corr =  CORR_MAX;
    if (corr < -CORR_MAX) corr = -CORR_MAX;
    int magL = pwm - corr;                           // left ahead -> left slows
    int magR = pwm + corr;
    if (magL < PWM_MIN) magL = PWM_MIN;              // clamp MAGNITUDES - the direction is
    if (magR < PWM_MIN) magR = PWM_MIN;              // decided by the turn, not by the clamp
    if (tsign > 0) { motorL(-magL); motorR(+magR); } // LEFT turn: left wheel backward,
    else           { motorL(+magL); motorR(-magR); } // right forward; mirrored for right
  } else if (tmode == M_PIVOT_L) {
    // pivot about the left wheel: left stays planted (no drive), right arcs.
    // NO symmetry controller here - one wheel only, nothing to equalise (guide p.24)
    motorL(0);
    motorR(tsign > 0 ? +pwm : -pwm);                 // ccw = right wheel forward
  } else {                                           // M_PIVOT_R: right planted, left arcs
    motorR(0);
    motorL(tsign > 0 ? -pwm : +pwm);                 // ccw = left wheel backward
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

/* ===== command execution ===== */
void execTurn(char letter, float arg) {
  float deg = arg < 0 ? -arg : arg;                  // direction lives in the letter (documented)
  if (deg < ANG_MIN_DEG || deg > ANG_MAX_DEG) {
    Serial.println(F("err: angle out of range (5..1440 deg)"));
    return;
  }
  uint8_t mode = tmode;                              // 'm' toggles spin/pivot before the turn
  turnStart(deg, mode, letter == 'L' ? +1 : -1);
  Serial.print(F("go turn="));
  Serial.print(letter == 'L' ? F("L") : F("R"));
  Serial.print(mode == M_SPIN ? F(" spin ") : F(" pivot "));
  Serial.print(deg, 1);
  Serial.println(F("deg"));
}

/* ===== serial ui (stage 3 commands kept; T/B carry the angle, m toggles the mode) ===== */
static char   cmdLetter = 0;
static bool   numNeg = false;
static long   numAcc = 0;
static long   numFrac = 0;
static int    numFracN = -1;
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
  if (letter == 'K') {                               // gain setter (spin symmetry)
    if (v < 0) v = 0;
    symGain = v;
    Serial.print(F("gain="));
    Serial.println(symGain, 2);
    return;
  }
  if (letter == 'J') {                               // track setter, for the calibration runs
    if (v < 50) v = 50;
    trackMm = v;
    Serial.print(F("track="));
    Serial.println(trackMm, 1);
    return;
  }
  execTurn(letter, v);
}

void handleChar(char ch) {
  if (cmdLetter) {
    if (ch >= '0' && ch <= '9') {
      if (numFracN >= 0) { numFrac = numFrac * 10 + (ch - '0'); numFracN++; }
      else numAcc = numAcc * 10 + (ch - '0');
      numSeen = true; lastCharMs = millis(); return;
    }
    if (ch == '.') { if (numFracN < 0) numFracN = 0; return; }
    if (ch == '-') { numNeg = !numNeg; return; }
    if (ch == '+' || ch == ' ') return;
    flushPendingCmd();
    if (ch == '\r' || ch == '\n') return;
  }
  lastCharMs = millis();
  switch (ch) {
    case 'T': case 'B':                              // T<deg> = turn left, B<deg> = turn right
    case 'L': case 'R':                              // also accepted: L/R, matches the stage-5 protocol
    case 'K':                                        // K<gain> symmetry gain
    case 'J':                                        // J<track> effective track mm
      cmdLetter = ch; numAcc = 0; numFrac = 0; numFracN = -1; numSeen = false; numNeg = false;
      break;
    case 'm':                                        // cycle spin -> pivotL -> pivotR -> spin
      tmode = (tmode == M_SPIN) ? M_PIVOT_L : (tmode == M_PIVOT_L ? M_PIVOT_R : M_SPIN);
      Serial.print(F("turn mode="));
      Serial.println(tmode == M_SPIN ? F("spin") : (tmode == M_PIVOT_L ? F("pivotL (about left wheel)") : F("pivotR (about right wheel)")));
      break;
    case 'c':
      if (trn == T_RUN) stopTurn();
      coast();
      Serial.println(F("mode=stop pwm=n/a (coast)"));
      break;
    case 'b':
      if (trn == T_RUN) stopTurn();
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
      Serial.println(trn == T_RUN ? F("turning") : (encCounting() ? F("counting") : F("frozen")));
      break;
    }
    case 'z':
      if (trn == T_RUN) stopTurn();
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
      if (trn == T_RUN) {
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
      Serial.println(F("== stage4 turn controller =="));
      Serial.println(F("L<deg> turn left  R<deg> turn right (T<deg>/B<deg> also work)"));
      Serial.println(F("m toggles spin/pivot   J<mm> set track   K<gain> symmetry gain"));
      Serial.println(F("e enc  z zero  s speed  p state  t telemetry  x bench"));
      Serial.println(F("c coast  b brake  h menu"));
      break;
    default:
      break;
  }
}

void pollPendingCmd() {
  if (cmdLetter && numSeen && millis() - lastCharMs > 50) flushPendingCmd();
  if (cmdLetter && !numSeen && millis() - lastCharMs > 300) {
    cmdLetter = 0;
    Serial.println(F("err: command without a number"));
  }
}

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_CNTL, INPUT_PULLUP);
  pinMode(PIN_CNTR, INPUT_PULLUP);
  coast();
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("== lab1 stage4: turn controller =="));
  attachInterrupt(digitalPinToInterrupt(PIN_CNTL), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CNTR), isrR, CHANGE);
  Serial.println(F("h = menu"));
}

void loop() {
  while (Serial.available() > 0) handleChar((char)Serial.read());
  pollPendingCmd();
  if (trn != T_IDLE && millis() - lastCtrlMs >= CTRL_MS) {   // all phases tick at 50 Hz
    lastCtrlMs = millis();
    turnStep();
  }
  // speed window (stage 2 diagnostic, kept alive)
  static uint32_t winStart = 0;
  uint32_t t = millis();
  if (t - winStart >= 500) {
    static int32_t prevL = 0, prevR = 0;
    int32_t cl = encCountL(), cr = encCountR();
    spdLatestL = (cl - prevL) * K_MM * 1000.0f / (t - winStart);
    spdLatestR = (cr - prevR) * K_MM * 1000.0f / (t - winStart);
    prevL = cl; prevR = cr;
    winStart = t;
  }
  static bool wasCounting = true;
  bool nowCounting = encCounting();
  if (wasCounting && !nowCounting) snapshotEncoders();
  wasCounting = nowCounting;
}
