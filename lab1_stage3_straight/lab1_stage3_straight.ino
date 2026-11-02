// iss lab1 - stage 3: straight-line controller for a given distance
// stage 2 encoder/motor layers unchanged, control added: position-driven speed profile
// with a stopping condition (the "distance controller") + proportional symmetry
// controller on the wheel odometry difference (the part the assignment requires).
// forbidden: AlphaBot.h and anything from the waveshare examples - all written from scratch

/* ===== interface speeds =====
   kept in one place, a mismatch here gives the symptom "nothing works" */
static const unsigned long SERIAL_BAUD = 9600;   // same link speed as stage 2, IDE monitor default

/* ===== pin map ===== */
static const uint8_t PIN_ENA  = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // channel A pwm + direction
static const uint8_t PIN_ENB  = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // channel B pwm + direction
static const uint8_t PIN_CNTL = 2,  PIN_CNTR = 3;                 // encoders, INT0 and INT1

static const int8_t SIGN_L = -1;   // floor-verified: on nominal wiring the robot rolled BACKWARD
static const int8_t SIGN_R = -1;   // on a "forward" command (both wheels, 14.10 session). flipping
                                   // both keeps the straight line, F now = the robot's front.
                                   // dirL/dirR below are taken from the USER sign, so positive
                                   // counts still mean physical forward

static const int PWM_MAX = 255;    // full duty

/* ===== odometry constants (stage 2 calibration, entered from the worksheet) ===== */
static const uint8_t  SLOTS   = 20;       // measured: 20 slots on both discs, 1 rev = 40 counts
static const uint8_t  N_REV   = 2*SLOTS;  // counts per wheel rev: CHANGE fires on both edges
static const float    WHEEL_D = 65.0f;    // mm, measured across the tyre
static const float    K_MM    = 5.1087f;  // mm per count, calibrated (spread 1.54 %, k_geom agrees +0.07 %)
static const uint16_t COAST_MS = 2000;    // coast window from stage 2, unchanged
static const uint16_t SPD_WIN  = 500;     // speed window ms, unchanged
static const uint16_t DEBOUNCE_US = 800;  // isr chatter filter, unchanged

/* ===== control parameters (from the calibration sheet, tuned on the robot) =====
   cruise point: pwm 130 sits mid-table on the measured characteristic,
   (292+312)/2 = 302 mm/s average of the two wheels */
static const int   PWM_CRUISE  = 130;     // operating point inside the linear region
static const float V_CRUISE    = 302.0f;  // mm/s the two wheels average at that pwm
static const float PWM_PER_MMS = PWM_CRUISE / V_CRUISE;  // linear pwm <-> speed map
static const int   PWM_MIN     = 85;      // profile floor: floor-measured dead band ~75 (wheels first
                                          // rolled at pwm<=75 under load, 14.10) + 10 margin.
                                          // a floor below the real dead band deadlocks the position
                                          // driven profile (no progress -> pwm stays at floor)
static const float V_FLOOR     = PWM_MIN / PWM_PER_MMS;  // profile never commands below this
static const float ACC_MM = 60.0f;        // acceleration ramp length (position driven)
static const float DEC_MM = 60.0f;        // deceleration ramp: >= stage 1 coast run-on at cruise,
                                          // slowdown to V_FLOOR removes almost all of it
static const float STOP_TOL_MM = 10.0f;   // 2 counts - quantisation floor from stage 2 report sec.8
static const uint16_t CTRL_MS  = 20;      // control period, 50 Hz, far above the ~11 ms/edge pulse rate
static const uint16_t STALL_MS = 1500;    // abort when BOTH wheels produce no counts this long: jam,
                                          // dead band below the floor, or the drive is simply unpowered
static const uint16_t WHEEL_STOP_MS = 300;  // wheel guard, detector 1 (silence): a wheel producing
                                            // no edge this long while the partner still edges is
                                            // stalled. sized vs the edge gap at the profile floor
                                            // (pwm 85 ~ 197 mm/s ~ one edge / 26 ms): 300 ms =
                                            // ~11 missed edges - a real stop, not a slow wheel
static const uint16_t GUARD_WIN_MS   = 200; // wheel guard, detector 2 (ratio): window over which the
                                            // two wheel displacements are compared
static const uint8_t  GUARD_RATIO_PCT = 20; // ...one wheel moved < 20 % of the partner's path ->
                                            // it is HELD (a hand on the wheel). catches the jerky
                                            // hold that defeats detector 1: the held wheel keeps
                                            // squeezing out single jerk edges which reset the
                                            // silence timer, but the jerks cannot fake the
                                            // displacement. margins both ways: healthy asymmetry
                                            // is <= 7 % (stage 2), a firm hand-hold slips <= 1
                                            // count / 200 ms -> <= 13 % even at the slow floor
                                            // speed (pwm 85 ~ 38 cnt/s real). NOTE for a stage-4/6
                                            // port: pivot turns have one wheel at zero -> the
                                            // ratio detector MUST be suspended in pivot mode
static const int32_t  GUARD_MIN_MOVE = 5;   // ...and the partner must have moved >= 5 counts
                                            // (~25 mm, ~127 mm/s) in the window: below that the
                                            // wheels are too slow for the ratio to mean anything
                                            // and detector 1 covers the case (edge gap << 300 ms)
static const float K_SYM  = 3.0f;         // symmetry gain, pwm per mm of wheel difference.
                                          // floor-tuned 14.10: L-R count spread 10/18 cnt at K0
                                          // -> 4 at K1 -> 2 at K3 (B-drives); no start wobble seen
static const int   CORR_MAX = 30;         // symmetry correction clamp, keeps a wheel above the dead band
static const float TGT_MIN_MM = 5.0f;     // argument range rules (documented in the report)
static const float TGT_MAX_MM = 5000.0f;
static const int   PWM_STEP   = 5;        // step for the manual dead-band search keys + / -

/* ===== encoder state ===== */
static volatile int32_t cntL = 0, cntR = 0;   // signed counts, sign = commanded direction
static volatile int8_t  dirL  = +1, dirR  = +1;
static volatile uint32_t lastDriveMs = 0;
static volatile int32_t snapL = 0, snapR = 0;
static bool benchMode = false;

/* ===== isr layer (stage 2, unchanged) ===== */
static volatile uint32_t lastEdgeUsL = 0, lastEdgeUsR = 0;

void isrL() {
  uint32_t t = micros();
  if (t - lastEdgeUsL < DEBOUNCE_US) return;        // chatter, not a slot edge
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

bool encCounting() {
  return benchMode || (millis() - lastDriveMs) < COAST_MS;
}

int32_t atomicRead(volatile int32_t &v) {           // no tearing on a 32 bit value on an 8 bit core
  uint8_t s = SREG;
  noInterrupts();
  int32_t t = v;
  SREG = s;                                         // if interrupts were already off they stay off
  return t;
}

int32_t encCountL() {
  return encCounting() ? atomicRead(cntL) : atomicRead(snapL);
}
int32_t encCountR() {
  return encCounting() ? atomicRead(cntR) : atomicRead(snapR);
}

void snapshotEncoders() {                           // freeze bookkeeping shared with the window logic
  noInterrupts(); snapL = cntL; snapR = cntR; interrupts();
}
void zeroEncoders() {
  noInterrupts(); cntL = 0; cntR = 0; snapL = 0; snapR = 0; interrupts();
}

/* ===== motor layer (stage 1/2, unchanged) ===== */
void motorL(int pwm) {
  dirL = (pwm >= 0) ? +1 : -1;      // encoder sign from USER intent, BEFORE the wiring correction:
  pwm *= SIGN_L;                    // positive counts must mean physical forward even when
  digitalWrite(PIN_IN1, pwm >= 0);  // SIGN flips the bridge command
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX));
  if (pwm) lastDriveMs = millis();
}
void motorR(int pwm) {
  dirR = (pwm >= 0) ? +1 : -1;      // same as motorL
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

/* ===== straight-line controller =====
   two independent parts acting on the same pair of motors (guide p.19):
   1. distance controller - a speed profile generator with a stopping condition,
      NOT a proportional controller. progress and remaining distance come from the
      encoders, so the ramp is position driven: slow near the start (no slip from a
      pwm step), slow near the target (run-on shrinks to a brake-sized stop).
   2. symmetry controller - proportional on the odometry difference of the wheels,
      the controller the assignment explicitly requires. it equalises wheel distances
      so the robot holds a straight line despite the measured ~7 % wheel asymmetry. */
enum MotState : uint8_t { M_IDLE, M_RUN };
static MotState mot = M_IDLE;
static int8_t  motDir = +1;          // +1 forward, -1 backward
static float   tgtMM = 0.0f;         // commanded distance magnitude
static int32_t startL = 0, startR = 0;   // counters captured when the motion starts
static uint32_t lastCtrlMs = 0;      // control period bookkeeping
static uint32_t lastProgressMs = 0;  // last time the counters moved, stall watchdog input
static float   symGain = K_SYM;      // runtime-tunable so gains can be tried without reflash
static float spdLatestL = 0.0f, spdLatestR = 0.0f;   // latest window speed, for the s command
static bool telOn = false;           // telemetry toggle, diagnostic only, off for real runs
static uint32_t lastTelMs = 0;

/* ===== manual open-loop mode (bench tool) =====
   keys 1/2 drive at curPwm without the controller, + / - step the duty.
   exists to measure the UNDER-LOAD dead band on the floor: the profile floor
   PWM_MIN must sit above it, otherwise the position driven ramp deadlocks
   (no progress -> pwm never rises). '0' stops the manual mode */
static bool  manualOn = false;
static int8_t manualDir = +1;
static int   curPwm = 130;           // run-on test default: cruise duty, settle, then c/b and watch

void stopMotion() {                  // cut the drive, freeze the state, one line stays printable
  brake();                           // dynamic brake, not coast: run-on after V_FLOOR is tiny
  mot = M_IDLE;
}

/* ===== wheel guard =====
   one shared detector used by BOTH the commanded drives and the manual mode:
   if ONE wheel stops rotating while the other still turns, brake both at once -
   the robot must never pivot around a held wheel, and distance counting must
   freeze with the real travel. two detectors, either one fires:
   1. silence - no edge from one wheel for WHEEL_STOP_MS while the partner edges
      (catches a clean stall: motor off on one side, rope on the wheel)
   2. ratio - over a GUARD_WIN_MS window one wheel moved < GUARD_RATIO_PCT of the
      partner's path while the partner covered >= GUARD_MIN_MOVE (catches a HAND
      HOLD: the held wheel keeps squeezing out jerk edges that reset the silence
      timer, but the jerks are single counts - the displacement ratio exposes them)
   the guard brakes immediately and latches one-shot; the caller (loop) ends the
   active mode - commanded drive or manual - so nothing restarts the wheels */
static bool guardTrippedL = false, guardTrippedR = false;   // which wheel fired, for the line
static int32_t  gWinL = 0, gWinR = 0;                       // counters at the last ratio window
static uint32_t gWinMs = 0;                                 // ratio window bookkeeping
static int32_t  gLastL = 0, gLastR = 0;                     // detector 1: counters at the last pass
static uint32_t gMoveLms = 0, gMoveRms = 0;                 // detector 1: last edge time per wheel
static bool guardLatched = false;                           // one-shot: fired stays fired until re-arm

bool wheelGuardCheck() {             // call every loop pass; true exactly once per fault
  uint32_t nowMs = millis();
  int32_t cL = encCountL();
  int32_t cR = encCountR();
  if (benchMode || millis() - lastDriveMs >= 100) {
    // guard watches only while the drive is COMMANDED now (a drive command refreshes
    // lastDriveMs every control step, manual mode via loop below): a hand push on a
    // PARKED robot must not trip it - there is no torque to fight, the coast window
    // freeze takes care of counting. bench mode is quiet for the same reason.
    gWinL = cL; gWinR = cR; gWinMs = nowMs;                // windows stay fresh, no stale jump
    gLastL = cL; gLastR = cR; gMoveLms = nowMs; gMoveRms = nowMs;
    guardLatched = false;                                  // arm state follows the drive state
    return false;
  }
  if (guardLatched) return false;    // already fired, wait for wheelGuardArm() on the next motion
  // --- detector 1: per-wheel silence ---
  if (cL != gLastL) { gLastL = cL; gMoveLms = nowMs; }   // left wheel produced edges
  if (cR != gLastR) { gLastR = cR; gMoveRms = nowMs; }   // right wheel produced edges
  bool lSilent = nowMs - gMoveLms > WHEEL_STOP_MS;
  bool rSilent = nowMs - gMoveRms > WHEEL_STOP_MS;
  bool oneSilent = lSilent != rSilent;                   // exactly one wheel silent
  // --- detector 2: displacement ratio over the window ---
  if (nowMs - gWinMs >= GUARD_WIN_MS) {
    int32_t dL = cL - gWinL;                           // signed per-wheel displacement in the window
    int32_t dR = cR - gWinR;
    int32_t aL = dL < 0 ? -dL : dL;                    // magnitudes: a held wheel jerking back
    int32_t aR = dR < 0 ? -dR : dR;                    // and forth nets near zero either way
    int32_t big = aL > aR ? aL : aR;                   // the rolling wheel, whatever the direction
    int32_t small = aL > aR ? aR : aL;                 // the possibly-held wheel
    if (big >= GUARD_MIN_MOVE && (int32_t)small * 100 < (int32_t)big * GUARD_RATIO_PCT) {
      oneSilent = true;                                // treat as one-wheel-stopped
      guardTrippedL = aL < aR;                         // name the held wheel
      guardTrippedR = !guardTrippedL;
    }
    gWinL = cL; gWinR = cR; gWinMs = nowMs;
  }
  if (oneSilent) {
    guardLatched = true;                               // one shot: no duplicate lines from run-on
    if (oneSilent && !guardTrippedL && !guardTrippedR) {   // detector 1 verdict: name the silent one
      guardTrippedL = lSilent;
      guardTrippedR = rSilent;
    }
    brake();                                           // both wheels stop NOW, ~5 mm run-on
    Serial.print(F("err: wheel stopped ("));
    Serial.print(guardTrippedL ? F("L") : F("R"));
    Serial.println(F(") - both stopped, distance frozen"));
    return true;
  }
  return false;
}

void wheelGuardArm() {               // (re)arm from the current counters - at every motion start
  gWinL = encCountL();
  gWinR = encCountR();
  gWinMs = millis();                 // ratio window restarts with the motion
  gLastL = gWinL; gLastR = gWinR;
  gMoveLms = millis(); gMoveRms = millis();   // silence timers restart too
  guardTrippedL = guardTrippedR = false;
  guardLatched = false;
}

void beginDrive(int8_t dir, float mm) {
  startL = encCountL();              // deltas measured from here, no z needed between drives
  startR = encCountR();
  motDir = dir;
  tgtMM  = mm;
  lastCtrlMs = millis();             // first control step a full period after the start
  lastProgressMs = millis();         // stall watchdog armed from motion start
  wheelGuardArm();                   // wheel guard re-armed on this motion's counters
  mot = M_RUN;
}

void controlStep() {                 // runs every CTRL_MS while M_RUN, never blocks
  int32_t cL = encCountL() - startL; // atomic reads inside, deltas since motion start
  int32_t cR = encCountR() - startR;
  if (cL != 0 || cR != 0) lastProgressMs = millis();   // wheels turned -> drive is alive
  else if (millis() - lastProgressMs > STALL_MS) {     // no counts for 1.5 s = stuck:
    stopMotion();                                      // jam, floor below dead band, or no power
    Serial.print(F("err: stalled - no counts in "));
    Serial.print(STALL_MS);
    Serial.println(F("ms (check battery/dead band), aborting"));
    return;
  }
  // per-wheel watchdog handled by wheelGuardCheck() in loop() - it must also see
  // manual-mode stalls, so it lives outside the controller step
  float dL = cL * K_MM;              // signed wheel distances, mm
  float dR = cR * K_MM;
  float prog = 0.5f * (dL + dR) * motDir;   // mean progress magnitude toward the target
  float rem  = tgtMM - prog;
  if (rem <= STOP_TOL_MM) {          // stopping condition, tolerance = 2 counts (stage 2 report)
    stopMotion();
    float got = prog * motDir;       // achieved signed distance, odometry truth (+fwd, -back)
    Serial.print(F("done cmd="));
    Serial.print(motDir > 0 ? F("+") : F("-"));
    Serial.print(tgtMM, 1);
    Serial.print(F("mm got="));
    Serial.print(got, 1);
    Serial.print(F("mm err="));
    Serial.print((got - (motDir > 0 ? tgtMM : -tgtMM)), 1);
    Serial.print(F("mm L="));
    Serial.print((long)cL);
    Serial.print(F("cnt R="));
    Serial.print((long)cR);
    Serial.println(F("cnt"));
    return;
  }
  // speed profile: position driven on both ends, floored at the dead band
  float v = V_CRUISE;
  float uAcc = prog / ACC_MM;        // 0 at the start -> 1 after ACC_MM of travel
  float uDec = rem  / DEC_MM;        // 1 far from the target -> 0 at it
  float u = min(uAcc, uDec);         // whichever ramp is active wins
  if (u < 1.0f) v = V_FLOOR + (V_CRUISE - V_FLOOR) * u;   // linear ramp between floor and cruise
  int pwm = (int)(v * PWM_PER_MMS + 0.5f);
  if (pwm < PWM_MIN) pwm = PWM_MIN;
  if (pwm > PWM_CRUISE) pwm = PWM_CRUISE;
  // symmetry controller: err > 0 when the left wheel travelled more (in commanded sense)
  float err = (dL - dR) * motDir;
  int corr = (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f));   // pwm units, rounded
  if (corr >  CORR_MAX) corr =  CORR_MAX;
  if (corr < -CORR_MAX) corr = -CORR_MAX;
  int pwmL = pwm - corr;             // left ahead -> left slows
  int pwmR = pwm + corr;             //   and right catches up
  if (pwmL < PWM_MIN) pwmL = PWM_MIN;   // correction must not stall a wheel below the dead band
  if (pwmR < PWM_MIN) pwmR = PWM_MIN;
  motorL(motDir * pwmL);
  motorR(motDir * pwmR);
  if (telOn && millis() - lastTelMs >= 100) {   // telemetry, ~45 chars per 100 ms = 450 B/s,
    lastTelMs = millis();                        // under the 9600 link budget, tx is buffered anyway
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

/* ===== command execution ===== */
void execDrive(char letter, float arg) {         // F/B with the number assembled char by char
  float mm = arg < 0 ? -arg : arg;               // rule: direction lives in the letter, a negative
                                                 // number is taken as its absolute value (documented)
  if (mm < TGT_MIN_MM || mm > TGT_MAX_MM) {      // out of range: rejected with one line, no motion
    Serial.println(F("err: distance out of range (5..5000 mm)"));
    return;
  }
  if (mot == M_RUN) stopMotion();                // a new command aborts a running motion
  beginDrive(letter == 'F' ? +1 : -1, mm);
  Serial.print(F("go dir="));
  Serial.print(letter == 'F' ? F("fwd") : F("back"));
  Serial.print(F(" dist="));
  Serial.print(mm, 1);
  Serial.println(F("mm"));
}

/* ===== serial ui =====
   one-char stage 2 commands kept for manual checks; F/B carry a number.
   parsing is a small char state machine: non blocking, no parseFloat timeout.
   the number supports one decimal point (needed for K gains like K0.6) */
static char   cmdLetter = 0;         // pending F/B/K awaiting its number
static bool   numNeg = false;
static long   numAcc = 0;            // integer part
static long   numFrac = 0;           // fractional part, digits accumulated
static int    numFracN = 0;          // how many fractional digits seen (-1 = no point yet)
static bool   numSeen = false;
static uint32_t lastCharMs = 0;      // flushes a number sent without a terminator

void flushPendingCmd() {             // called when the number stream goes quiet
  if (!cmdLetter) return;
  char letter = cmdLetter;
  float v = (float)numAcc;
  if (numFracN > 0) {                // paste the fractional digits back on
    float div = 1.0f;
    for (int i = 0; i < numFracN; i++) div *= 10.0f;
    v += (float)numFrac / div;
  }
  if (numNeg) v = -v;
  cmdLetter = 0; numAcc = 0; numFrac = 0; numFracN = -1; numSeen = false; numNeg = false;
  if (letter == 'K') {               // gain setter, not a motion
    if (v < 0) v = 0;
    symGain = v;
    Serial.print(F("gain="));
    Serial.println(symGain, 2);
    return;
  }
  execDrive(letter, v);
}

void handleChar(char ch) {
  if (cmdLetter) {                              // collecting the argument of a pending F/B/K
    if (ch >= '0' && ch <= '9') {
      if (numFracN >= 0) { numFrac = numFrac * 10 + (ch - '0'); numFracN++; }
      else numAcc = numAcc * 10 + (ch - '0');
      numSeen = true; lastCharMs = millis(); return;
    }
    if (ch == '.') { if (numFracN < 0) numFracN = 0; return; }   // start the fraction (one point)
    if (ch == '-') { numNeg = !numNeg; return; }  // sign anywhere before the digits
    if (ch == '+' || ch == ' ') return;         // tolerated but meaningless
    flushPendingCmd();                          // any other char ends the number
    if (ch == '\r' || ch == '\n') return;       // terminator itself is not a new command
  }
  lastCharMs = millis();
  switch (ch) {
    case 'F': case 'B':                         // start of a drive command with argument
    case 'K':                                   // symmetry gain setter with argument
      cmdLetter = ch; numAcc = 0; numFrac = 0; numFracN = -1; numSeen = false; numNeg = false;
      break;
    case 'c':
      if (mot == M_RUN) stopMotion();           // manual stop aborts the motion, no result line
      manualOn = false;
      coast();
      Serial.println(F("mode=stop pwm=n/a (coast)"));
      break;
    case 'b':
      if (mot == M_RUN) stopMotion();
      manualOn = false;
      brake();
      Serial.println(F("mode=stop pwm=n/a (brake)"));
      break;
    case '1':                                   // manual fwd: open loop at curPwm, no controller
      mot = M_IDLE;                             // a running drive aborts silently, no done line
      manualOn = true; manualDir = +1;
      wheelGuardArm();                          // fresh guard window for this manual run
      motorL(+curPwm); motorR(+curPwm);
      Serial.print(F("manual fwd pwm="));
      Serial.println(curPwm);
      break;
    case '2':                                   // manual backward
      mot = M_IDLE;
      manualOn = true; manualDir = -1;
      wheelGuardArm();                          // fresh guard window for this manual run
      motorL(-curPwm); motorR(-curPwm);
      Serial.print(F("manual back pwm="));
      Serial.println(curPwm);
      break;
    case '0':                                   // stop manual mode
      manualOn = false;
      coast();
      Serial.println(F("manual off"));
      break;
    case '+':                                   // pwm up in manual mode (dead-band search)
    case '-': {
      bool up = ch == '+';
      curPwm = up ? min(curPwm + PWM_STEP, PWM_MAX) : max(curPwm - PWM_STEP, 0);
      if (manualOn) {                           // live change while the manual drive runs
        if (manualDir > 0) { motorL(+curPwm); motorR(+curPwm); }
        else               { motorL(-curPwm); motorR(-curPwm); }
      }
      Serial.print(F("pwm="));
      Serial.println(curPwm);
      break;
    }
    case 't':                                   // telemetry toggle, diagnostics only
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
      Serial.println(mot == M_RUN ? F("running") : (encCounting() ? F("counting") : F("frozen")));
      break;
    }
    case 'z':
      if (mot == M_RUN) stopMotion();           // zeroing under a running controller breaks it
      zeroEncoders();
      wheelGuardArm();                          // counters jumped -> guard windows must restart
      Serial.println(F("enc zeroed"));
      break;
    case 's':                                   // s - window speed, diagnostic
      Serial.print(F("spd L="));
      Serial.print(spdLatestL, 1);
      Serial.print(F("mm/s R="));
      Serial.print(spdLatestR, 1);
      Serial.println(F("mm/s"));
      break;
    case 'p':
      Serial.print(F("mode="));
      if (mot == M_RUN) {
        Serial.print(motDir > 0 ? F("run fwd ") : F("run back "));
        Serial.println(tgtMM, 1);
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
      Serial.println(F("== stage3 straight-line controller =="));
      Serial.println(F("F<mm> drive forward   B<mm> drive backward"));
      Serial.println(F("K<gain> symmetry gain (0=off, default 0.6)"));
      Serial.println(F("1/2 manual fwd/back at pwm, +/- pwm up/down, 0 stop (dead-band search)"));
      Serial.println(F("e enc  z zero  s speed  p state  t telemetry  x bench"));
      Serial.println(F("c coast  b brake  h menu"));
      break;
    default:
      break;                                    // unknown characters are skipped silently
  }
}

void pollPendingCmd() {              // number typed without terminator still executes
  if (cmdLetter && numSeen && millis() - lastCharMs > 50) flushPendingCmd();
  if (cmdLetter && !numSeen && millis() - lastCharMs > 300) {  // bare letter: reject, no default drive
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
  Serial.println(F("== lab1 stage3 v4: K_SYM=3 PWM_MIN=85 settled-odom patch =="));
  attachInterrupt(digitalPinToInterrupt(PIN_CNTL), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CNTR), isrR, CHANGE);
  Serial.println(F("h = menu"));
}

void loop() {
  while (Serial.available() > 0) {   // non blocking, never waits
    handleChar((char)Serial.read());
  }
  pollPendingCmd();                  // flush a number that arrived without a terminator
  if (manualOn) {
    lastDriveMs = millis();          // manual drive is COMMANDED driving: keep the encoder
  }                                 // coast window open (stage-2 pattern, curMode != 0 there)
  // wheel guard: watches BOTH wheels on every pass, in commanded drives AND manual
  // mode. verdict true = one wheel stopped while the other turned -> the guard has
  // already braked; here we only end the mode so no controller restarts the wheels
  if (wheelGuardCheck()) {
    if (mot == M_RUN) stopMotion();          // abort the commanded drive (no done line - it never reached the target)
    manualOn = false;                        // manual mode ends too
    mot = M_IDLE;
  }
  if (mot == M_RUN && millis() - lastCtrlMs >= CTRL_MS) {
    lastCtrlMs = millis();           // fixed control period, 50 Hz
    controlStep();                   // the only place that drives the motors during a motion
  }
  // speed window bookkeeping (stage 2 diagnostic, kept alive)
  static uint32_t winStart = 0;
  uint32_t t = millis();
  if (t - winStart >= SPD_WIN) {
    static int32_t prevL = 0, prevR = 0;
    int32_t cl = encCountL(), cr = encCountR();
    float sL = (cl - prevL) * K_MM * 1000.0f / (t - winStart);
    float sR = (cr - prevR) * K_MM * 1000.0f / (t - winStart);
    prevL = cl; prevR = cr;
    winStart = t;
    spdLatestL = sL; spdLatestR = sR;   // stored for the s command, not printed here
  }
  // snapshot when the counting window closes (stage 2 logic, unchanged)
  static bool wasCounting = true;
  bool nowCounting = encCounting();
  if (wasCounting && !nowCounting) {
    snapshotEncoders();
    // window just closed: one settled-odometry line from the frozen snapshot,
    // mean of the two wheels in mm - the final travel the coast window counted
    float settled = 0.5f * ((float)snapL + (float)snapR) * K_MM;
    Serial.print(F("settled odom="));
    Serial.print(settled, 1);
    Serial.print(F("mm L="));
    Serial.print((long)snapL);
    Serial.print(F("cnt R="));
    Serial.print((long)snapR);
    Serial.println(F("cnt"));
  }
  wasCounting = nowCounting;
}
