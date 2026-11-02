// iss lab1 - stage 7: wheel-sync / fault-recovery layer on top of the frozen stage 6
// stage 6 files stay BYTE-IDENTICAL (acceptance test cannot break): this sketch is the
// stage 6 source + a purely ADDITIVE fault layer inside controlStep + an S status key.
// G/L/R/K/J/m/c/b/t/e/z/s/p/x/h behaviour is unchanged; the stage-5 protocol decisions
// in REPORT_STAGE5 sec.2 still rule here.
// the stage 5 program IS the stage 6 program (the guide: "nothing new to write
// beyond merging stages 1-5"): this file is stage5 v1 with logic untouched,
// re-identified for the acceptance session (greeting + menu + this header).
// phase 1 route building, phase 2 one-string autonomous run, phase 3 spread -
// procedure lives in MEASURE_STAGE6.md. execution variant B (guide stage-6
// table): the non-blocking parser + 8-command program queue, so a route string
// longer than the 64 B rx buffer still executes whole.
// forbidden: AlphaBot.h and anything from the waveshare examples - all written from scratch

/* ===== FREEZE STAMP =====
   date: 21.10.2026 | firmware tag: stage6-v1-FROZEN
   k = 5.108 mm/count (turn grid 1.94 deg/count)
   b_spin = 151 mm (1440-gate confirmed) | TRACK_PIVOT = 145 mm (provisional)
   K_SYM = 3.0 | PWM_MIN = 85 | PWM_CRUISE 130 (drive) / 110 (turn)
   decision gate 21.10: A->B 3 runs + turn-band check -> FREEZE / breakaway patch
   FROZEN - no further calibration. known limitations: see REPORT_STAGE6.md */

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
static const float TURN_STOP_LEAD_DEG = 7.8f; // stop this much EARLY: brake residual + stop-step
                                              // overshoot at the PWM_MIN floor (measured on the
                                              // robot as got= minus target with lead=0; sim brake
                                              // plant: 2 count-pairs = 7.75 deg). recipe: flash
                                              // with 0.0, run L90, set lead = (got - target),
                                              // re-flash. quantisation floor: 1 pair = 3.9 deg
static const int   CORR_MAX_TRN = 20;     // turn symmetry clamp (stage 4 value)
static const int   PULSE_PWM    = 120;    // breakaway pulse: > profile max, breaks stiction
static const uint16_t PULSE_MS  = 150;
static const int   KICK_PWM     = 95;     // active-brake reverse pulse: kills momentum
static const uint16_t KICK_MS   = 60;
static const uint16_t QUIET_MS  = 100;    // coast settle: no new counts = stopped
static const uint16_t COAST_TMO = 1000;   // coast poll timeout (hardware fault guard)
static const int   RETRY_MAX    = 1;      // stall auto-retries before the watchdog aborts

/* ===== wheel-sync / fault-recovery layer (stage 7, additive) =====
   sits INSIDE controlStep (no second timer, nothing blocking): windowed per-wheel
   counters, no floats in the ISR path (the layer never touches the ISR).
   1. STALL one wheel: relative RATIO detector over each STALL_WIN_MS window (the
      stage-3 wheel-guard v2 pattern, proven on the floor): one wheel moving < 20 %
      of the partner's path while the partner covers >= STALL_MIN_MOVE counts is
      "quiet"; N_STALL_STEPS consecutive quiet windows -> stalled -> BOTH wheels
      brake within one control step (dynamic brake, no coast) and M_HOLD latches.
      margins: healthy asymmetry <= 7 % (stage 2), a firm hold slips <= 13 % even
      at the profile floor; 20 % sits between. windowed DISPLACEMENT, so jerk
      edges from a hand-hold cannot reset the evidence (guard v1's failure mode).
   1b. STALL both wheels: both quiet for the same budget -> mid-drive freeze
      (grab / front jam) -> abort with an err line. (the stage-6 STALL_MS watchdog
      only covers the never-started case: it refreshes on ANY count since start.)
   2. SLIP rate: |d(cL-cR)| per SLIP_WIN_MS above SLIP_DIFF_MAX_CNT = the faster
      wheel is running away from the ground. physical cap: both wheels clamped to
      pwm 85..160 differ by <= ~1.4 cnt/window (CORR_MAX_DRV 30 + 6 % motor asym);
      a real free-spinning wheel does ~23 cnt/window (stage-1 bench: ~300 cnt/s
      free vs ~59 cnt/s loaded at pwm 130).
   3. SLIP sustained: |(cL-cR)*K_MM| >= SLIP_ERR_MM for SLIP_ERR_WINDOWS windows
      catches partial slip too slow for the rate detector (the clamped P-term lets
      the difference grow linearly); floor data caps healthy err at ~2 cnt = 10 mm.
      corr resumes only after SLIP_VERIFY_WINDOWS clean rate windows.
   M_HOLD = periodic TEST PULSES in alternating directions (the yaw of the pair
   cancels; a spinning free wheel would otherwise pivot the chassis around the
   held wheel and accumulate heading error). HOLD_TEST_PWM spins a free wheel
   (~3-4 cnt per 250 ms pulse) but stays under the ~120 pwm stiction breakaway
   (stage-4 PULSE_PWM measurement) so a truly held wheel stays held. The stuck
   wheel producing >= HOLD_RESUME_CNT counts in a cycle = cause gone ->
   RESYNC: progress := MIN(|cL|,|cR|) (a slipping wheel over-counts, a held
   wheel under-counts, the good wheel reads the ground), symmetry reference :=
   the post-event counter difference (continue PARALLEL, no steer-back), profile
   resumes toward the ORIGINAL target. Both wheels dead in hold for
   HOLD_BLOCKED_CYCLES cycles -> abort. Operator: c/b abort, any G/L/R
   re-commands (direct exec during hold IS the re-command path), S = last fault. */
static const uint16_t STALL_WIN_MS      = 80;    // per-wheel displacement window (4 control steps)
static const int32_t  STALL_MIN_MOVE    = 3;     // rolling wheel >= counts/window for the ratio to be
                                                 // judgeable (below: too slow to judge - the ramp tail
                                                 // and the never-started watchdog own that region)
static const int32_t  STALL_QUIET_CNT   = 2;     // <= counts/window = the wheel produced no output
static const uint8_t  STALL_RATIO_PCT   = 20;    // quiet wheel moves < 20 % of the partner's path
                                                 // (guard-v2 margins: healthy <= 7 %, hold <= 13 %)
static const uint8_t  N_STALL_STEPS     = 3;     // consecutive judged windows -> verdict (~240 ms: a
                                                 // displacement verdict cannot be honest in fewer -
                                                 // 2-4 single control steps would false-positive on
                                                 // jerk edges and spool-up; braked the moment it fires)
static const uint16_t SLIP_WIN_MS       = 100;   // wheel-difference rate window (5 control steps)
static const int32_t  SLIP_DIFF_MAX_CNT = 8;     // max legit |d(cL-cR)|/window (cap ~1.4, free spin ~23)
static const float    SLIP_ERR_MM       = 20.0f; // sustained wheel-path difference = slip (healthy <= 10 mm)
static const uint8_t  SLIP_ERR_WINDOWS  = 2;     // ...consecutive windows (200 ms)
static const float    SLIP_ERR_CLEAR_MM = 10.0f; // diff detector re-arms below this err (hysteresis)
static const uint8_t  SLIP_VERIFY_WINDOWS = 5;   // clean rate windows after a fault before corr resumes
static const int      HOLD_TEST_PWM     = 110;   // test pulse: spins a FREE wheel, stays under the ~120 pwm
                                                  // breakaway (stage-4 PULSE_PWM) so a HELD wheel stays held.
                                                  // tune on the robot: raise until the free wheel reliably
                                                  // counts per pulse, check the held one stays at 0
static const uint16_t HOLD_TEST_MS      = 250;   // pulse length (spool-up eats ~120 ms -> ~3-4 cnt per pulse)
static const uint16_t HOLD_GAP_MS       = 150;   // braked gap between the fwd and rev test pulses
static const int32_t  HOLD_RESUME_CNT   = 3;     // held-side |counts| per 2-pulse cycle >= this = freed
static const uint8_t  HOLD_CLR_WINDOWS  = 2;     // consecutive freed cycles before the resync (~1.6 s)
static const uint8_t  HOLD_BLOCKED_CYCLES = 3;   // countless cycles in hold -> abort (both wheels jammed)
static const uint16_t SPOOL_GRACE_MS    = 400;   // stall/blocked detectors wait this long (or until
                                                 // real motion) after start/resync: the ~120 ms
                                                 // motor spool-up yields <= ~2.5 cnt per 80 ms window,
                                                 // BELOW the quiet threshold - without the grace the
                                                 // detector would abort every drive at t=240 ms
static const int32_t  STALL_MOVED_CNT   = 10;    // ~51 mm total travel: once covered, the wheels are
                                                 // provably past spool-up and the grace lifts early

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
enum MotState : uint8_t { M_IDLE, M_RUN, M_HOLD };   // M_HOLD: stage-7 fault latch (brake +
                                                     // test pulses + resync, see faultLayer)
static MotState mot = M_IDLE;
static int8_t  motDir = +1;          // +1 forward, -1 backward
static float   tgtMM = 0.0f;         // commanded distance magnitude, mm
static float   cmdM = 0.0f;          // commanded value in PROTOCOL units (signed metres), for the done line

/* ===== stage-7 fault state (straight controller owns it; guarded by mot == M_HOLD) ===== */
static int8_t  faultSide = 0;        // stalled wheel: -1 left, +1 right, 0 none (M_HOLD selector)
static uint8_t faultQuiet = 0;       // consecutive quiet windows on the suspect side
static uint8_t blockedQuiet = 0;     // consecutive BOTH-quiet windows (mid-drive freeze)
static int32_t fPrevL = 0, fPrevR = 0;   // per-wheel counters at the last stall-window edge
static uint32_t fWinMs = 0;          // stall-window bookkeeping
static int32_t slipPrevDiff = 0;     // (cL-cR) at the last slip-rate window edge
static uint32_t slipWinMs = 0;       // slip-rate window bookkeeping
static uint8_t slipRateClean = 0;    // consecutive clean rate windows (corr release timer)
static uint8_t slipDiffWins = 0;     // consecutive windows with |diff| >= SLIP_ERR_MM
static bool    slipLatch = false;    // true: progress = MIN(|cL|,|cR|), symmetry corr = 0
static float   symRefRaw = 0.0f;     // symmetry reference in raw counter-mm: 0 at motion start,
                                     // re-captured at every fault event (continue parallel)
static uint8_t holdClear = 0;        // consecutive "freed" test cycles
static uint8_t blockedCycles = 0;    // countless test cycles (both wheels jammed in hold)
static bool    faultArmed = false;   // detectors run only while armed (beginDrive..stopMotion)
static uint32_t spoolUntilMs = 0;    // detectors quiet until this stamp (spool-up blind spot) or
                                     // until the wheels produce STALL_MOVED_CNT counts, whichever
                                     // comes first
// last-fault record for the S key (one slot, overwritten by the next event)
static int8_t   lfSide = 0;          // -1 L, +1 R, 0 both/none
static uint8_t  lfState = 0;         // 1 hold, 2 cleared/resumed, 3 aborted
static uint32_t lfEnterMs = 0;       // millis() at hold entry
static uint32_t lfDurationMs = 0;    // hold length, filled on clear or abort
static float    lfSlipMM = 0.0f;     // wheel-path difference at entry, mm
static float    lfRecoveredMM = 0.0f;// MIN-count progress preserved through the event
static uint8_t  lfCount = 0;         // events since power-up
// lfRateCnt: slip magnitude in counts/window from detector 2 (0 for the sustained type)
static int32_t lfRateCnt = 0;
// hold test cycle phases: fwd pulse -> braked gap -> reverse pulse -> judge -> gap ...
enum : uint8_t { HP_GAP, HP_FWD, HP_REV };
static uint8_t  fPhase = HP_GAP;
static bool     fNextRev = false;    // false: next pulse forward, true: reverse
static int32_t  fPhaseL = 0, fPhaseR = 0;  // counters at the current pulse start
static int32_t  fAccL = 0, fAccR = 0;      // |counts| folded over the current 2-pulse cycle
static uint32_t fPhaseMs = 0;              // current hold phase timer (gap or pulse)

float faultProgress(int32_t cL, int32_t cR) {   // slip-safe progress: MIN of the |counts|
  int32_t aL = labs(cL), aR = labs(cR);         // a slipping/free wheel OVER-counts, a held
  int32_t m = (aL < aR) ? aL : aR;              // wheel UNDER-counts, the good wheel reads the
  return (float)m * K_MM;                       // ground. biased LOW on purpose: the robot may
}                                               // stop short of the target, never overshoot it

void faultDisarm() {                 // stop watching (motion over or aborted)
  faultArmed = false;
}

void faultArm() {                    // (re)arm every window from the CURRENT counters
  fPrevL = encCountL() - startL;     // relative counts, same frame the detectors use
  fPrevR = encCountR() - startR;
  fWinMs = millis();
  slipPrevDiff = fPrevL - fPrevR;
  slipWinMs = millis();
  slipRateClean = 0;
  slipDiffWins = 0;
  slipLatch = false;
  symRefRaw = 0.0f;                  // fresh motion: symmetry error measured from zero
  faultQuiet = 0;
  faultSide = 0;
  blockedQuiet = 0;
  blockedCycles = 0;
  holdClear = 0;
  spoolUntilMs = millis() + SPOOL_GRACE_MS;   // detectors blind through the spool-up
  faultArmed = true;
}

void stopMotion() {                  // cut the drive, back to idle (no result line on abort)
  brake();
  faultDisarm();                     // stage 7: detectors live only inside a commanded drive
  mot = M_IDLE;
}

void beginDrive(int8_t dir, float mm) {
  startL = encCountL();              // deltas measured from here, no z needed between motions
  startR = encCountR();
  motDir = dir;
  tgtMM  = mm;
  lastCtrlMs = millis();             // first control step a full period after the start
  lastProgressMs = millis();         // stall watchdog armed from motion start
  faultArm();                        // stage 7: fault windows start from THIS motion's counters
  mot = M_RUN;
}

/* ===== stage-7 fault layer: record, hold, hold-step (all non-blocking) ===== */
void faultRecord(int8_t fastSide, int32_t rateCnt, float diffMM) {   // S-key record
  if (mot == M_HOLD) return;         // an open hold event IS the current fault record
  lfSide = fastSide;
  lfState = 4;                       // 4 = slip detected, progress suppressed (no hold needed)
  lfEnterMs = millis();
  lfDurationMs = 0;
  lfRateCnt = rateCnt;
  lfSlipMM = diffMM;
  lfCount++;
}

void faultEnterHold(int8_t side, int32_t cL, int32_t cR) {   // one-wheel stall verdict
  brake();                           // BOTH wheels stop THIS control step - no coast, no run-on
  faultSide = side;
  lfSide = side;
  lfState = 1;                       // 1 = hold active
  lfEnterMs = millis();
  lfRateCnt = 0;
  lfSlipMM = fabsf((float)(cL - cR)) * K_MM;
  lfRecoveredMM = faultProgress(cL, cR);
  lfCount++;
  holdClear = 0;
  blockedCycles = 0;
  blockedQuiet = 0;
  fAccL = 0; fAccR = 0;
  fPhase = HP_GAP;                   // first test pulse after the gap
  fNextRev = false;
  fPhaseMs = millis();
  mot = M_HOLD;
  Serial.print(F("warn: "));
  Serial.print(side < 0 ? F("L") : F("R"));
  Serial.println(F(" wheel stopped - both braked, holding (c=abort, S=status)"));
}

void faultHoldStep() {               // M_HOLD: alternating test pulses, judge, resync or abort
  int32_t cL = encCountL() - startL;
  int32_t cR = encCountR() - startR;
  if (fPhase == HP_GAP) {
    brake();
    if (millis() - fPhaseMs < HOLD_GAP_MS) return;
    fPhase = fNextRev ? HP_REV : HP_FWD;       // pulse directions alternate: the yaw of the
    fPhaseL = cL; fPhaseR = cR;                // pair cancels over a cycle, no heading drift
    fPhaseMs = millis();
    int sgn = (fPhase == HP_FWD) ? +1 : -1;
    motorL(motDir * sgn * HOLD_TEST_PWM);
    motorR(motDir * sgn * HOLD_TEST_PWM);
    return;
  }
  if (millis() - fPhaseMs < HOLD_TEST_MS) return;   // pulse still running
  brake();                           // pulse over: back to dynamic brake
  fAccL += labs(cL - fPhaseL);       // |counts| of BOTH wheels over the pulse
  fAccR += labs(cR - fPhaseR);
  fNextRev = !fNextRev;
  fPhase = HP_GAP;
  fPhaseMs = millis();
  if (fNextRev) return;              // one pulse done, judge after the second
  // full 2-pulse cycle: judge
  int32_t stuck = (faultSide < 0) ? fAccL : fAccR;   // the stalled wheel
  int32_t mate  = (faultSide < 0) ? fAccR : fAccL;   // the free wheel
  fAccL = 0; fAccR = 0;
  if (stuck >= HOLD_RESUME_CNT) {    // held wheel turns under the test pulse = cause gone
    holdClear++;
    blockedCycles = 0;
    if (holdClear < HOLD_CLR_WINDOWS) return;   // require 2 clean cycles: no jitter resync
    // --- RESYNC (rationale in the layer header): the robot continues PARALLEL to its
    // post-fault heading toward the ORIGINAL target - no steer-back, no restart from 0.
    symRefRaw = (float)(cL - cR) * K_MM * motDir;   // symmetry reference := measured diff
    slipRateClean = SLIP_VERIFY_WINDOWS;            // both wheels just proved alive: trust counts
    slipLatch = false;
    lfState = 2;                       // 2 = cleared and resumed
    lfDurationMs = millis() - lfEnterMs;
    lfRecoveredMM = faultProgress(cL, cR);
    holdClear = 0;
    faultSide = 0;
    fPrevL = cL; fPrevR = cR;          // stall windows restart from the current counters
    fWinMs = millis();
    slipPrevDiff = cL - cR;
    slipWinMs = millis();
    blockedQuiet = 0;
    mot = M_RUN;
    Serial.print(F("sync: fault cleared after "));
    Serial.print(lfDurationMs);
    Serial.print(F("ms, resuming from "));
    Serial.print(lfRecoveredMM, 1);
    Serial.print(F("mm of "));
    Serial.print(tgtMM, 1);
    Serial.println(F("mm (target unchanged)"));
    return;
  }
  holdClear = 0;
  if (mate >= HOLD_RESUME_CNT) {     // free wheel spins: hold is the correct state, keep testing
    blockedCycles = 0;
    return;
  }
  if (++blockedCycles >= HOLD_BLOCKED_CYCLES) {  // BOTH wheels dead through 3 cycles: nothing
    lfState = 3;                   // will free itself - hand the robot back to the operator
    lfDurationMs = millis() - lfEnterMs;
    stopMotion();
    Serial.print(F("err: still blocked after "));
    Serial.print(lfDurationMs);
    Serial.println(F("ms in hold - aborting drive"));
  }
}

void controlStep() {                 // every CTRL_MS while M_RUN, never blocks
  int32_t cL = encCountL() - startL;
  int32_t cR = encCountR() - startR;
  bool spooled = (int32_t)millis() >= (int32_t)spoolUntilMs;   // stage 7: windowed detectors
  if (faultArmed && spooled && labs(cL) + labs(cR) >= STALL_MOVED_CNT)
    spoolUntilMs = 0;                // real motion ends the grace early
  if (faultArmed && !spooled) {      // during grace: keep the windows FRESH so the first
    fPrevL = cL; fPrevR = cR;        // post-grace window is a normal 80 ms one, not a
    fWinMs = slipWinMs = millis();   // 400 ms smear of the whole spool-up
    slipPrevDiff = cL - cR;
  } else if (faultArmed) {
    if (millis() - fWinMs >= STALL_WIN_MS) {   // detector 1: per-wheel stall, RELATIVE ratio
      int32_t dL = labs(cL - fPrevL), dR = labs(cR - fPrevR);
      int32_t big = dL > dR ? dL : dR;         // the rolling wheel, whatever the side
      if (big == 0) {                          // BOTH dead in a judged window: mid-drive freeze
        if (++blockedQuiet >= N_STALL_STEPS) { // (grab / front jam) - the stage-6 watchdog only
          stopMotion();                        // catches the never-started case
          Serial.print(F("err: drive blocked (both wheels) - aborting after "));
          Serial.print((unsigned long)N_STALL_STEPS * STALL_WIN_MS);
          Serial.println(F("ms of zero output"));
          return;
        }
      } else if (big >= STALL_MIN_MOVE) {      // judgeable window (slow ramps EXTEND the window
        int32_t small = dL > dR ? dR : dL;     // instead of producing a false quiet)
        // one wheel produced < RATIO_PCT of the partner's path: held/stalled. the quiet
        // side must also be <= STALL_QUIET_CNT so two nearly-equal slow wheels never
        // read as a pair of stalls
        bool lQuiet = dL <= STALL_QUIET_CNT, rQuiet = dR <= STALL_QUIET_CNT;
        bool oneQuiet = (lQuiet != rQuiet) &&
                        (int32_t)small * 100 < big * STALL_RATIO_PCT;
        blockedQuiet = 0;
        faultQuiet = oneQuiet ? (uint8_t)(faultQuiet + 1) : 0;
        if (faultQuiet >= N_STALL_STEPS) {     // 3 consecutive windows = stall verdict
          faultEnterHold(dL < dR ? -1 : +1, cL, cR);
          return;
        }
        fPrevL = cL; fPrevR = cR;
      }                                        // else: window stays open (extends) until judgeable
      fWinMs = millis();
    }
    if (millis() - slipWinMs >= SLIP_WIN_MS) { // detectors 2+3: slip (100 ms windows)
      int32_t diff = cL - cR;
      int32_t rate = diff - slipPrevDiff;      // change of the wheel difference per window
      if (rate > SLIP_DIFF_MAX_CNT || rate < -SLIP_DIFF_MAX_CNT) {
        slipRateClean = 0;
        if (!slipLatch) {                      // record ONCE per latch, not every window
          faultRecord(rate > 0 ? -1 : +1, rate, fabsf((float)diff) * K_MM);
          slipLatch = true;                    // faster wheel is slipping: distrust its counts
          Serial.print(F("warn: slip "));
          Serial.print(rate > 0 ? F("L") : F("R"));
          Serial.println(F(" wheel - progress = MIN(count), corr off"));
        }
      } else if (slipRateClean < SLIP_VERIFY_WINDOWS) slipRateClean++;
      slipPrevDiff = diff;
      slipWinMs = millis();
      float diffMM = (float)diff * K_MM * motDir;   // detector 3: sustained difference
      if (fabsf(diffMM) >= SLIP_ERR_MM) {           // (the rate detector misses slow leaks)
        if (!slipLatch && ++slipDiffWins >= SLIP_ERR_WINDOWS) {
          slipLatch = true;
          slipRateClean = 0;
          faultRecord(diff > 0 ? +1 : -1, 0, fabsf(diffMM));   // record once, at the latch
          Serial.print(F("warn: sustained "));
          Serial.print(fabsf(diffMM), 1);
          Serial.println(F("mm wheel difference - progress = MIN(count), corr off"));
        }
      } else if (fabsf(diffMM) <= SLIP_ERR_CLEAR_MM) slipDiffWins = 0;   // hysteresis re-arm
    }
  }
  if (cL != 0 || cR != 0) lastProgressMs = millis();   // wheels turned -> drive is alive
  else if (millis() - lastProgressMs > STALL_MS) {     // never-started watchdog (stage 6, kept)
    stopMotion();
    Serial.print(F("err: stalled - no counts in "));
    Serial.print(STALL_MS);
    Serial.println(F("ms (check battery/dead band), aborting"));
    return;
  }
  float dL = cL * K_MM;              // signed wheel distances, mm
  float dR = cR * K_MM;
  // stage 7: slip-safe progress + symmetry reference. slipLatch = MIN of |counts|; the
  // reference carries the pre-fault asymmetry so the error is measured against it
  float prog = (slipLatch ? faultProgress(cL, cR) : 0.5f * (dL + dR) * motDir) - symRefRaw;
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
  // symmetry controller, stage 7: the error term is (symmetry signal) - (reference in
  // the same units). reference = 0 on a healthy motion (identical to stage 6); after a
  // fault it is re-captured from the counters so the robot continues PARALLEL (heading
  // term unchanged, no steer-back into the pre-fault counter pair). during a slip latch
  // corr = 0: the P-term must not chase counts that are not ground truth.
  float err = ((slipLatch ? (float)(labs(cL) - labs(cR)) * K_MM * motDir
                          : (dL - dR) * motDir) - symRefRaw);
  int corr = slipLatch ? 0
           : (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f));   // pwm units, rounded
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
static int32_t   coastSnapL = 0, coastSnapR = 0;   // quiet-detector tracker (refreshed, never folded)
static int32_t   stopSnapL = 0, stopSnapR = 0;     // counters AT the stop instant: coast= residual
static int32_t   coastL = 0, coastR = 0;           // is measured against THIS (information only)
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
  int32_t cL = encCountL() - startL;                 // coastL/R retired: with sign-corrected ISR
  int32_t cR = encCountR() - startR;                 // counting the live counters already include
  float sL = cL * K_MM;                              // every post-stop residual; folds double-counted
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

void stopTurn() {                                    // stop condition met: DYNAMIC BRAKE ONLY.
  brake();                                           // no reverse kick: driveReverse re-stamps
  stopSnapL = encCountL();                           // dirL/dirR mid-rotation, so every residual
  stopSnapR = encCountR();                           // edge still turning forward counts BACKWARD
  coastSnapL = stopSnapL;                            // (isrL/isrR sign by commanded direction) - and
  coastSnapR = stopSnapR;                            // the old fold added those counts AGAIN. the
  coastLastMs = millis();                            // settle poll tracks quiet on the live counters,
  phaseStartMs = millis();                           // residual is signed honestly, nothing folds
  trn = T_COAST;
}

void turnStep() {                                    // 50 Hz phase machine, never blocks
  lastCtrlMs = millis();                             // EVERY phase owns the 50 Hz slot: the loop
  if (trn == T_PULSE) {                              // scheduler shares lastCtrlMs with controlStep,
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
  if (trn == T_COAST) {                              // settle poll: quiet tracking on the LIVE
    int32_t nowL = encCountL(), nowR = encCountR();  // counters; the tracker follows them so the
    if (nowL != coastSnapL || nowR != coastSnapR) {  // quiet timer measures stillness, and the
      coastSnapL = nowL; coastSnapR = nowR;          // residual itself is NEVER folded (it is
      coastLastMs = millis();                        // already inside the live counters)
      return;
    }
    if (millis() - coastLastMs < QUIET_MS && millis() - phaseStartMs < COAST_TMO) return;
    // wheels quiet (or timeout - counted as-is): every count since start belongs to got=
    float prog = progressDeg() * tsign;
    float over = prog - tgtDeg;                      // >0 = residual pushed past the target
    float oneCntDeg = (tmode == M_SPIN ? 2.0f : 1.0f) * K_MM / getTrack(tmode != M_SPIN) * 180.0f / 3.14159265f;
    if (!corrected && over >= oneCntDeg * 0.5f) {    // overshot >= half a count: ONE short
      corrected = true;                              // reverse pulse, then settle again
      coastSnapL = encCountL();                      // next residual measured from the kick end
      coastSnapR = encCountR();
      coastLastMs = millis();
      phaseStartMs = millis();
      trn = T_KICK;
      driveReverse(KICK_PWM);
      return;
    }
    trn = T_IDLE;
    coastL = encCountL() - stopSnapL;                // post-stop residual, information only:
    coastR = encCountR() - stopSnapR;                // ALREADY included in the live counters
    lastAchievedDeg = progressDeg();                 // got= is a pure function of counts
    reportTurn(encCountL() - startL, encCountR() - startR);
    return;
  }
  if (trn == T_KICK) {                               // overshoot correction ONLY (rare):
    if (millis() - phaseStartMs >= KICK_MS) {
      brake();                                       // stop the reverse drive before polling
      coastSnapL = encCountL();                      // post-kick residual measured from here
      coastSnapR = encCountR();
      coastLastMs = millis();
      phaseStartMs = millis();
      trn = T_COAST;
    }
    return;
  }
  if (trn != T_RUN) return;

  int32_t cL = encCountL() - startL;                 // live counters are the whole truth: every
  int32_t cR = encCountR() - startR;                 // edge is already signed and counted
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
  if (rem <= STOP_TOL_DEG + TURN_STOP_LEAD_DEG) {    // stop BEFORE the target: the brake
    stopTurn();                                      // residual (counted honestly now) and the
    return;                                          // stop-step overshoot land ON the target
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
#if defined(TELEMETRY_TURN) && TELEMETRY_TURN
    // debug-only hook (compile out for acceptance: leave TELEMETRY_TURN undefined)
    // target_ticks = theta(rad) * b_spin / 2 / K_MM  (spin: each wheel travels b*theta/2)
    {
      float tgtTicks = tgtDeg * 3.14159265f / 180.0f * getTrack(false) / 2.0f / K_MM;
      Serial.print(F("tturn_tel tgt_ticks=")); Serial.print(tgtTicks, 1);
      Serial.print(F(" NL="));  Serial.print((long)cL);
      Serial.print(F(" NR="));  Serial.print((long)cR);
      Serial.print(F(" pwmL=")); Serial.print(tsign > 0 ? -magL : magL);
      Serial.print(F(" pwmR=")); Serial.print(tsign > 0 ? magR : -magR);
      Serial.print(F(" e="));   Serial.print(err, 1);
      Serial.print(F(" corr=")); Serial.println(corr);
    }
#endif
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
  if (mot != M_IDLE || trn != T_IDLE) return;         // still busy - INCLUDING M_HOLD (stage 7):
  char letter; float v;                               // a queued command must not fight the fault
  if (queuePop(letter, v)) execPending(letter, v);   // hold; a FRESH typed G/L/R bypasses the
}                                                    // queue (busy=false in hold) = re-command

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
      if (mot == M_RUN || mot == M_HOLD) stopMotion();  // MID-HOLD (stage 7: abort out of
      if (trn == T_RUN) stopTurn();                  // the fault latch too) and flushes the
      queueClear();                                  // turn finishes its stop sequence and
      coast();                                       // reports the partial angle, a drive
      Serial.println(F("mode=stop pwm=n/a (coast)")); // just stops silently (documented)
      break;
    case 'b':
      if (mot == M_RUN || mot == M_HOLD) stopMotion();   // brake works out of hold too (stage 7)
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
      } else if (mot == M_HOLD) {                    // stage 7: hold state visible to the operator
        Serial.print(F("hold "));
        Serial.print(faultSide < 0 ? F("L") : F("R"));
        Serial.print(F(" stalled, target "));
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
    case 'S': {                                       // stage 7: last fault record (one slot)
      Serial.print(F("flt #"));
      Serial.print(lfCount);
      Serial.print(F(" state="));
      if (lfState == 0)      Serial.println(F("none"));
      else if (lfState == 1) Serial.println(F("HOLD active"));
      else if (lfState == 2) Serial.print(F("cleared+resumed after "));
      else if (lfState == 3) Serial.print(F("aborted after "));
      else                   Serial.print(F("slip-suppressed, diff="));
      if (lfState == 2 || lfState == 3) {
        Serial.print(lfDurationMs);
        Serial.print(F("ms, recovered="));
        Serial.print(lfRecoveredMM, 1);
        Serial.print(F("mm, diff="));
        Serial.print(lfSlipMM, 1);
        Serial.println(F("mm"));
      } else if (lfState == 4) {
        Serial.print(lfSlipMM, 1);
        Serial.print(F("mm, rate="));
        Serial.print((long)lfRateCnt);
        Serial.println(F("cnt/win"));
      } else if (lfState == 1) {
        Serial.print(F("side="));
        Serial.print(lfSide < 0 ? F("L") : F("R"));
        Serial.print(F(", held "));
        Serial.print(millis() - lfEnterMs);
        Serial.print(F("ms, entry diff="));
        Serial.print(lfSlipMM, 1);
        Serial.println(F("mm"));
      }
      break;
    }
    case 'h': case '?':
      Serial.println(F("== stage7 A-B drive + fault recovery: G<metres> L<deg> R<deg> =="));
      Serial.println(F("G signed metres (negative=back), L/R direction from letter (negative=abs)"));
      Serial.println(F("m turn mode  K<gain> symmetry  J<track mm>  e enc  z zero  s speed"));
      Serial.println(F("p state  t telemetry  x bench  c coast-stop  b brake  h menu  S last fault"));
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
  Serial.println(F("stage7 v1: A-B drive + fault recovery | G/L/R protocol | k=5.109 spin=151 pivot=145 [prov]"));
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
  if (mot == M_HOLD) {               // stage 7: hold runs on the same 50 Hz slot (no 2nd timer);
    if (millis() - lastCtrlMs >= CTRL_MS) {
      lastCtrlMs = millis();
      faultHoldStep();               // pulses are phase-timed inside, always non-blocking
    }
  } else if ((mot == M_RUN || trn != T_IDLE) && millis() - lastCtrlMs >= CTRL_MS) {
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
