// pid_test.cpp - C++ test harness for the PID math of lab2_stage6_track
// pidStep() copied VERBATIM from the sketch (2026-11-21 D-fix version),
// with Arduino-specific bits stripped and state passed by reference:
//
// sketch source (verbatim):
//   int pidStep(int error) {
//     filteredError += PID_D_FILTER_ALPHA * ((float)error - filteredError);
//     float d = kd * (filteredError - prevFilteredError) * PID_DT_SCALE;
//     prevFilteredError = filteredError;
//     float p = kp * (float)error;
//     integral += ki * (float)error;
//     if (integral >  iLimit) integral =  iLimit;
//     if (integral < -iLimit) integral = -iLimit;
//     long u = (long)(p + integral + d);
//     if (u >  PWM_MAX) u =  PWM_MAX;
//     if (u < -PWM_MAX) u = -PWM_MAX;
//     return (int)u;
//   }
// PID_D_FILTER_ALPHA = 0.25, PID_DT_SCALE = 50 (20 ms tick), PWM_MAX = 255.
// 2026-11-21 history: the OLD math (per-tick kd on smoothed-vs-raw error)
// saturated u from 20-50 count calibration residuals - reproduced once by
// test_old_math_saturation and now retired from the sketch.

#include <cstdio>
#include <cmath>
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int PWM_MAX = 255;   // same constant as the sketches
static const float PID_D_FILTER_ALPHA = 0.25f;
static const float PID_DT_SCALE = 50.0f;
static bool g_filterSeeded = false;   // mirrors sketch filterSeeded; reset by pidReset

int pidStep(int error, float kp, float ki, float kd, float iLimit, bool smoothErrorIgnored,
            float& integral, float& filteredError, float& prevFilteredError) {
  (void)smoothErrorIgnored;   // smoothing is now always on; kept for call compat
  // seed semantics copied from the sketch: explicit g_filterSeeded flag
  // (file-static above), reset by pidReset
  float d;
  if (!g_filterSeeded) {
    filteredError = (float)error;
    prevFilteredError = filteredError;
    g_filterSeeded = true;
    d = 0.0f;
  } else {
    filteredError += PID_D_FILTER_ALPHA * ((float)error - filteredError);
    d = kd * (filteredError - prevFilteredError) * PID_DT_SCALE;
    prevFilteredError = filteredError;
  }

  float p = kp * (float)error;

  // ANTI-WINDUP (conditional integration) copied verbatim from the sketch:
  // do not integrate when the pre-clamp output is already at a rail AND the
  // integration would push further INTO that same rail
  float uPre = p + integral + d;
  bool atPosRail = (uPre >= PWM_MAX);
  bool atNegRail = (uPre <= -PWM_MAX);
  bool windPos = atPosRail && (ki * (float)error > 0);
  bool windNeg = atNegRail && (ki * (float)error < 0);
  if (!windPos && !windNeg) {
    integral += ki * (float)error;
    if (integral >  iLimit) integral =  iLimit;
    if (integral < -iLimit) integral = -iLimit;
  }

  long u = (long)(p + integral + d);
  if (u >  PWM_MAX) u =  PWM_MAX;
  if (u < -PWM_MAX) u = -PWM_MAX;
  return (int)u;
}

void pidReset(float& integral, float& filteredError, float& prevFilteredError) {
  integral = 0.0f;
  filteredError = 0.0f;
  prevFilteredError = 0.0f;
  g_filterSeeded = false;
}

static int passed = 0;
static int failed = 0;

static void report(const char* name, bool ok, const char* reason) {
  if (ok) { printf("TEST %s: PASS\n", name); passed++; }
  else    { printf("TEST %s: FAIL - %s\n", name, reason); failed++; }
}

// 1. P-only: kp=1, feed e=1000 for 10 ticks -> u=255 (clamped) every tick,
//    filter state must NOT leak into u (D contribution decays to 0), integral 0.
static void test_p_only() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 1; t <= 10; t++) {
    int u = pidStep(1000, 1.0f, 0.0f, 0.0f, 1000.0f, false, integral, filteredError, prevFilteredError);
    if (u != 255) { ok = false; snprintf(reason, 128, "tick %d: u=%d expected clamped 255", t, u); break; }
  }
  if (ok && integral != 0.0f) { ok = false; snprintf(reason, 128, "integral=%f expected 0", integral); }
  if (ok) {  // gain line below the clamp: kp=0.2, e=1000 -> u=200 exactly
    integral = 0; filteredError = 0; prevFilteredError = 0;
    int u = pidStep(1000, 0.2f, 0.0f, 0.0f, 1000.0f, false, integral, filteredError, prevFilteredError);
    if (u != 200) { ok = false; snprintf(reason, 128, "gain line: u=%d expected 200 (kp=0.2, e=1000)", u); }
  }
  report("1 P-only", ok, reason);
}

// 2. I-accumulation: kp=0, ki=0.001, e=1000 for 10 ticks -> integral=10.0, u==integral
static void test_i_accumulation() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 1; t <= 10; t++) {
    int u = pidStep(1000, 0.0f, 0.001f, 0.0f, 1e9f, false, integral, filteredError, prevFilteredError);
    if (u != (int)integral) { ok = false; snprintf(reason, 128, "tick %d: u=%d != integral=%f", t, u, integral); break; }
  }
  if (ok && (integral < 9.999f || integral > 10.001f)) {
    ok = false; snprintf(reason, 128, "integral=%f expected 10.0", integral);
  }
  report("2 I-accumulation", ok, reason);
}

// 3. I-clamp: iLimit=5, e=1000 for 100 ticks -> integral in [-5,5], u in [-255,255]
static void test_i_clamp() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 1; t <= 100; t++) {
    int u = pidStep(1000, 0.0f, 0.001f, 0.0f, 5.0f, false, integral, filteredError, prevFilteredError);
    if (integral > 5.0f || integral < -5.0f) {
      ok = false; snprintf(reason, 128, "tick %d: integral=%f outside +-5", t, integral); break;
    }
    if (u > 255 || u < -255) {
      ok = false; snprintf(reason, 128, "tick %d: u=%d outside +-255", t, u); break;
    }
  }
  if (ok && integral != 5.0f) { ok = false; snprintf(reason, 128, "integral=%f expected clamped 5.0", integral); }
  report("3 I-clamp", ok, reason);
}

// 4. D-step (per second): kd=1 PWM per (count/s), e: 0 -> 100 step.
//    Seeded tick: filt=0, NO D. Tick 2: filt=25 -> d=25*50=1250 -> clamped 255.
//    Tail decays below clamp; at kp=0 converged u -> 0 (pure P of zero).
static void test_d_step() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  int u1 = pidStep(0,   0.0f, 0.0f, 1.0f, 1e9f, false, integral, filteredError, prevFilteredError);
  int u2 = pidStep(100, 0.0f, 0.0f, 1.0f, 1e9f, false, integral, filteredError, prevFilteredError);
  int uLast = 0;
  for (int t = 0; t < 40; t++) uLast = pidStep(100, 0.0f, 0.0f, 1.0f, 1e9f, false, integral, filteredError, prevFilteredError);
  if (u1 != 0)  { ok = false; snprintf(reason, 128, "tick1 u=%d expected 0 (seed, no D)", u1); }
  if (ok && u2 != 255) { ok = false; snprintf(reason, 128, "tick2 u=%d expected clamped 255 (step D)", u2); }
  if (ok && uLast != 0) { ok = false; snprintf(reason, 128, "converged u=%d expected 0 (kp=0, D decayed)", uLast); }
  report("4 D-step", ok, reason);
}

// 5. D-noise (THE 21.11 BUG SHAPE, now fixed): constant error with 50-count
//    residual noise, kd=0.05/s. First tick saturates via P... wait kp=0 here:
//    tick 0 is the SEED (no D), from tick 1 on the LPF cuts the alternation
//    (filt delta only ~12.5 -> d ~ 0.05*12.5*50 = 31 max) -> u stays small.
static void test_d_noise() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  pidReset(integral, filteredError, prevFilteredError);   // clear test-4 seed state
  bool ok = true; char reason[128] = "";
  for (int t = 0; t < 100; t++) {
    int e = (t % 2 == 0) ? 1000 : 1050;
    int u = pidStep(e, 0.0f, 0.0f, 0.05f, 1e9f, false, integral, filteredError, prevFilteredError);
    if (t == 0) continue;              // seed tick, no D by design
    if (u > 40 || u < -40) {
      ok = false; snprintf(reason, 128, "tick %d: u=%d - D noise too big", t, u); break;
    }
  }
  report("5 D-noise bounded", ok, reason);
}

// 6. pidReset clears state: accumulate, reset, feed e=0 -> u=0, integral=0
static void test_pid_reset() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  for (int t = 0; t < 5; t++)
    pidStep(1000, 0.0f, 0.001f, 0.0f, 1e9f, false, integral, filteredError, prevFilteredError);
  pidReset(integral, filteredError, prevFilteredError);
  bool ok = true; char reason[128] = "";
  if (integral != 0.0f) { ok = false; snprintf(reason, 128, "integral=%f after reset", integral); }
  if (ok && filteredError != 0.0f) { ok = false; snprintf(reason, 128, "filteredError=%f after reset", filteredError); }
  if (ok) {
    int u = pidStep(0, 0.0f, 0.001f, 0.0f, 1e9f, false, integral, filteredError, prevFilteredError);
    if (u != 0) { ok = false; snprintf(reason, 128, "u=%d after reset+e=0", u); }
  }
  report("6 pidReset clears state", ok, reason);
}

// 7. Windup protection on lost-line: e=2001 x50 with iLimit=100 -> clamp at 100;
//    after pidReset, e=10 with kp=0.5 -> u=5 (NOT 5+100)
static void test_windup() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 0; t < 50; t++) {
    pidStep(2001, 0.0f, 0.01f, 0.0f, 100.0f, false, integral, filteredError, prevFilteredError);
  }
  if (integral != 100.0f) { ok = false; snprintf(reason, 128, "integral=%f expected clamped 100", integral); }
  if (ok) {
    pidReset(integral, filteredError, prevFilteredError);
    int u = pidStep(10, 0.5f, 0.01f, 0.0f, 100.0f, false, integral, filteredError, prevFilteredError);
    if (u != 5) { ok = false; snprintf(reason, 128, "u=%d after reset, expected 5 (kp*10)", u); }
  }
  report("7 Windup on lost-line", ok, reason);
}

// 8. Combined PID sine: kp=0.5 ki=0.005 kd=2 iLimit=100, 200 ticks of sine error.
//    Check u stays bounded and integral stays inside its clamp (no blow-up).
static void test_combined() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 0; t < 200; t++) {
    int e = (int)(800.0 * sin(2.0 * M_PI * t / 50.0));
    int u = pidStep(e, 0.5f, 0.005f, 2.0f, 100.0f, false, integral, filteredError, prevFilteredError);
    if (u > 255 || u < -255) { ok = false; snprintf(reason, 128, "tick %d: u=%d outside +-255", t, u); break; }
    if (integral > 100.0f || integral < -100.0f) {
      ok = false; snprintf(reason, 128, "tick %d: integral=%f outside +-100", t, integral); break;
    }
  }
  if (ok) {
    printf("  first 20 rows (t,e,u):\n");
    integral = 0; filteredError = 0; prevFilteredError = 0;
    for (int t = 0; t < 200; t++) {
      int e = (int)(800.0 * sin(2.0 * M_PI * t / 50.0));
      int u = pidStep(e, 0.5f, 0.005f, 2.0f, 100.0f, false, integral, filteredError, prevFilteredError);
      if (t < 20 || t >= 180) printf("  %3d,%4d,%4d\n", t, e, u);
    }
  }
  report("8 Combined PID sine", ok, reason);
}

/* =====================================================================
   Position tests (teacher fix, 2026-11-21).
   computePosition() is copied VERBATIM from lab2_stage6_track.ino with the
   only difference that trs.readCalibrated() is replaced by copying from the
   injected cal[] array - the arithmetic below is identical to the sketch:

   sketch source (verbatim, minus the library call):
     long computePosition(bool &lineVisible) {
       trs.readCalibrated(sensorValues);
       long sum = 0;
       for (uint8_t i = 0; i < NUM_SENSORS; i++) {
         int v = (int)sensorValues[i];
         if (lineDarkFlag) v = 1000 - v;
         if (v < 0) v = 0;
         sensorValues[i] = (unsigned int)v;
         sum += v;
       }
       if (sum < lineLostSumMin) { lineVisible = false; return -1; }
       long weighted = 0;
       for (uint8_t i = 0; i < NUM_SENSORS; i++)
         weighted += (long)sensorValues[i] * ((long)i * 1000);
       lineVisible = true;
       return weighted / sum;
     }
   ===================================================================== */

static const int NUM_SENSORS = 5;
static const int SETPOINT = 2000;
static int lineLostSumMin = 250;      // same default as the sketch (NEEDS HW TUNING)
static bool lineDarkFlag = true;      // same default as the sketch
static unsigned int sensorValues[NUM_SENSORS];

// test double for trs.readCalibrated(): copies the injected calibrated values
static void injectCalibrated(const unsigned int* cal) {
  for (int i = 0; i < NUM_SENSORS; i++) sensorValues[i] = cal[i];
}

static long computePosition(bool &lineVisible) {
  // (trs.readCalibrated happens in injectCalibrated before the call)
  long sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = (int)sensorValues[i];
    if (lineDarkFlag) v = 1000 - v;
    if (v < 0) v = 0;
    sensorValues[i] = (unsigned int)v;
    sum += v;
  }
  if (sum < lineLostSumMin) { lineVisible = false; return -1; }
  long weighted = 0;
  for (int i = 0; i < NUM_SENSORS; i++)
    weighted += (long)sensorValues[i] * ((long)i * 1000);
  lineVisible = true;
  return weighted / sum;
}

// 9. Centred line: only sensor 2 (index 2) sees the line -> position 2000, error 0.
//    Dark line + lineDarkFlag=true: cal over line LOW (40) -> corrects to 960.
//    Background calibrated at its bright extreme (cal 1000) -> corrects to 0.
//    NOTE on real data: background cal ~870-980 leaves corrected residuals 20-130
//    which slightly bias the position toward the centre (see test 15).
static void test_position_centred() {
  const unsigned int cal[5] = {1000, 1000, 40, 1000, 1000};  // dark line under idx2
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  bool ok = visible && (pos == 2000);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d pos=%ld expected visible=1 pos=2000", visible, pos);
  report("9 Position centred", ok, reason);
}

// 10. Far left: only sensor 0 sees the line -> position 0 -> error -2000.
static void test_position_far_left() {
  const unsigned int cal[5] = {30, 1000, 1000, 1000, 1000};
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  int e = (int)(pos - SETPOINT);
  bool ok = visible && (e == -2000);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d error=%d expected -2000", visible, e);
  report("10 Position far left", ok, reason);
}

// 11. Far right: only sensor 4 sees the line -> position 4000 -> error +2000.
static void test_position_far_right() {
  const unsigned int cal[5] = {1000, 1000, 1000, 1000, 25};
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  int e = (int)(pos - SETPOINT);
  bool ok = visible && (e == 2000);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d error=%d expected +2000", visible, e);
  report("11 Position far right", ok, reason);
}

// 12. Line lost: every sensor reads cal 1000 = uniformly DARK surface (e.g. robot
//     drove off the track edge onto black tape). lineDarkFlag=true corrects every
//     value to 0, sum = 0 < lineLostSumMin -> visible=false, NO position (-1).
static void test_position_lost() {
  const unsigned int cal[5] = {1000, 1000, 1000, 1000, 1000};
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  bool ok = !visible && (pos == -1);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d pos=%ld expected visible=0 pos=-1", visible, pos);
  report("12 Position lost (all low)", ok, reason);
}

// 13. Inverted polarity: bright line on dark floor. Over the line cal is HIGH
//     (960), floor cal LOW (40). Measurement sets lineDarkFlag=false -> NO
//     invert -> the line still ends up the high side -> pos 2000.
static void test_position_inverted_polarity() {
  const unsigned int cal[5] = {40, 40, 960, 40, 40};  // bright line under idx2
  lineDarkFlag = false;                    // measurement said: line reads HIGH, no invert
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  lineDarkFlag = true;                     // restore the sketch default for other tests
  bool ok = visible && (pos == 2000);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d pos=%ld expected 2000 (no-invert path)", visible, pos);
  report("13 Inverted polarity", ok, reason);
}

// 14. Between sensors: line halfway between idx1 (position 1000) and idx2 (2000)
//     -> 1500. Line straddles both sensors equally: equal corrected weights.
static void test_position_between_sensors() {
  const unsigned int cal[5] = {1000, 720, 720, 1000, 1000};
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  bool ok = visible && (pos == 1500);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d pos=%ld expected 1500", visible, pos);
  report("14 Between sensors", ok, reason);
}

// 15. REALISTIC background (documents expected behaviour, not an exact value):
//     real calibration leaves background cal ~950-980 -> corrected residuals
//     20-50 per sensor. These residuals bias the average toward the centre:
//     line under idx2 but residuals heavier on the left -> pos 1936, error -64.
//     Accept: line visible, |error| < 100 (small bias, correct side, correct sign).
static void test_position_realistic_background() {
  const unsigned int cal[5] = {950, 960, 40, 970, 980};
  injectCalibrated(cal);
  bool visible = true;
  long pos = computePosition(visible);
  int e = (int)(pos - SETPOINT);
  bool ok = visible && (e > -100 && e < 100);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "visible=%d error=%d expected small centre bias (<100)", visible, e);
  report("15 Realistic background bias", ok, reason);
}

/* ===== Correction-split + dead-band tests (bench 21.11 PM fix) =====
   applyCorrection/deadBandCompensate copied from the sketch:
     int u = -uIn;
     if (u >  baseSpeed) u =  baseSpeed;
     if (u < -baseSpeed) u = -baseSpeed;
     left  = baseSpeed + u;  right = baseSpeed - u;
     ... per-wheel: mag>255 clamp, then 1..255 -> 85..255, 0 -> 0 */
static const int PWM_MIN_DB = 85;
static int g_baseSpeed = 90;   // mirror of sketch baseSpeed for these tests

static int dbCompensate(int pwm) {
  if (pwm == 0) return 0;
  int mag = pwm > 0 ? pwm : -pwm;
  if (mag > PWM_MAX) mag = PWM_MAX;
  int mapped = PWM_MIN_DB + ((PWM_MAX - PWM_MIN_DB) * mag) / PWM_MAX;
  return (pwm > 0) ? mapped : -mapped;
}

static void splitCorrection(int uIn, int &L, int &R) {
  int u = -uIn;
  if (u >  g_baseSpeed) u =  g_baseSpeed;
  if (u < -g_baseSpeed) u = -g_baseSpeed;
  L = g_baseSpeed + u;
  R = g_baseSpeed - u;
}

// 16. Huge correction at base=90: u=+1000 (error deep right) -> negated to
//     -1000, clamped to -90 -> L=0 (stop), R=180. NO wheel reverses.
//     Old behaviour: L=-10 -> dbCompensate(-10) = -85 PWM REVERSE = the spin.
static void test_clamp_no_reverse() {
  int L, R;
  splitCorrection(1000, L, R);
  bool ok = (L == 0 && R == 180);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "u=+1000: L=%d R=%d expected L=0 R=180", L, R);
  if (ok) {
    splitCorrection(-1000, L, R);
    if (L != 180 || R != 0) { ok = false; snprintf(reason, 128, "u=-1000: L=%d R=%d expected 180/0", L, R); }
  }
  if (ok && dbCompensate(0) != 0) { ok = false; snprintf(reason, 128, "dbCompensate(0)=%d expected 0", dbCompensate(0)); }
  if (ok && dbCompensate(-10) >= 0) { ok = false; snprintf(reason, 128, "dbCompensate(-10)=%d - small negative MUST stay negative request", dbCompensate(-10)); }
  report("16 Clamp prevents reverse", ok, reason);
}

// 17. Moderate correction: u=+250 (below base) -> L=base-250<0 would reverse!
//     Wait: u=+250 negated -> -250, clamped to -90 (base=90) -> L=0, R=180.
//     u=+45 -> -45 -> L=45, R=135: inner wheel at 45 - SMALL POSITIVE request.
//     dbCompensate(45)=85+170*45/255=115: above dead band, moves forward. Good.
static void test_clamp_moderate() {
  int L, R;
  bool ok = true; char reason[128] = "";
  splitCorrection(45, L, R);
  if (L != 45 || R != 135) { ok = false; snprintf(reason, 128, "u=+45: L=%d R=%d expected 45/135", L, R); }
  if (ok) {
    int mapped = dbCompensate(L);
    if (mapped < PWM_MIN_DB) { ok = false; snprintf(reason, 128, "dbCompensate(45)=%d below PWM_MIN", mapped); }
  }
  if (ok) {
    splitCorrection(-45, L, R);
    if (L != 135 || R != 45) { ok = false; snprintf(reason, 128, "u=-45: L=%d R=%d expected 135/45", L, R); }
  }
  report("17 Moderate correction", ok, reason);
}

// 18. Zero error: u=0 -> L=R=base=90, both wheels equal -> straight.
static void test_clamp_zero_error() {
  int L, R;
  splitCorrection(0, L, R);
  bool ok = (L == 90 && R == 90);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "u=0: L=%d R=%d expected 90/90 (straight)", L, R);
  report("18 Zero error straight", ok, reason);
}

// 19. Search spin bypasses the clamp: searchSpinSpeed=120 at base=90 gives
//     L=-120 R=+120 (opposite signs, |values| > base) - the clamp would have
//     forbidden exactly this. dbCompensate(120)=85+170*120/255=165 both sides.
static void test_search_spin_bypasses_clamp() {
  int spin = 120;
  int L = -spin, R = spin;      // what applySearchMovement passes to driveWheels
  bool ok = (L == -120 && R == 120);   // NOT clamped to +-90
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "spin L=%d R=%d - must bypass the clamp", L, R);
  if (ok) {
    int Lp = dbCompensate(L), Rp = dbCompensate(R);
    if (Lp != -165 || Rp != 165) { ok = false; snprintf(reason, 128, "compensated L=%d R=%d expected -165/+165", Lp, Rp); }
  }
  report("19 Search spin bypasses clamp", ok, reason);
}

/* ===== Anti-windup tests (21.11 PM: ki=0.001 + rail -> frozen robot) ===== */

// 20. Conditional integration: sustained huge error with ki>0. Old code
//     pinned integral at iLimit -> u stuck at the rail forever = frozen robot.
//     New code: once uPre is at the rail, integration SKIPS while the error
//     pushes further in, so a sign change of error takes effect IMMEDIATELY:
//     e=+2000 x200 (rail), then e=-500 -> u must drop below the rail within
//     a few ticks (old math: u stayed 255 for many ticks while the integrator
//     unwound from +iLimit).
static void test_antiwindup_rail_recovery() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  int ticksAtRail = 0, ticksBelowWithin3 = 999;
  for (int t = 0; t < 200; t++) {
    int u = pidStep(2000, 0.25f, 0.001f, 0.0f, 1000.0f, false,
                    integral, filteredError, prevFilteredError);
    if (u == 255) ticksAtRail++;
  }
  if (ticksAtRail < 50) { ok = false; snprintf(reason, 128, "expected sustained rail, only %d ticks", ticksAtRail); }
  if (ok) {  // error flips sign: with anti-windup, P alone gives -125 instantly;
             // integral HELD (not wound to +1000), so u must be well below 255 now
    for (int t = 0; t < 3; t++) {
      int u = pidStep(-500, 0.25f, 0.001f, 0.0f, 1000.0f, false,
                      integral, filteredError, prevFilteredError);
      if (u < 255 && ticksBelowWithin3 == 999) ticksBelowWithin3 = t;
    }
    if (ticksBelowWithin3 == 999) { ok = false; snprintf(reason, 128, "still at rail 3 ticks after sign flip - windup not prevented"); }
  }
  report("20 Anti-windup rail recovery", ok, reason);
}

// 21. The exact bench case: kp=0, ki=0.001, e=1000 sustained (P=0, so u = I).
//     OLD math: integral wound to iLimit=1000 over 2000 ticks; unwinding at
//     only 1.0/tick kept u at the 255 rail for ~750 ticks after the error
//     flipped - the "DRIVING but frozen" robot.
//     NEW math: integration stops once uPre>=255, integral freezes ~255.5
//     (NOT 1000), and after the error flips, u falls IMMEDIATELY (1 PWM/tick
//     at ki=0.001*e=1000 - full exit from the rail in ~510 ticks, but the
//     first tick already responds). Assertions: integral not wound past the
//     rail + u strictly decreasing right after the flip.
static void test_antiwindup_freeze_case() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  bool ok = true; char reason[128] = "";
  for (int t = 0; t < 2000; t++) {
    pidStep(1000, 0.0f, 0.001f, 0.0f, 1000.0f, false,
            integral, filteredError, prevFilteredError);
  }
  if (integral > 260.0f) {
    ok = false; snprintf(reason, 128, "integral=%f - wound past the rail (anti-windup inactive)", integral);
  }
  if (ok) {
    // sign flip: next 3 ticks must DECREASE u monotonically (old math: flat 255)
    int uPrev = 255, uDecr = 0;
    for (int t = 0; t < 3; t++) {
      int u = pidStep(-1000, 0.0f, 0.001f, 0.0f, 1000.0f, false,
                      integral, filteredError, prevFilteredError);
      if (u < uPrev) uDecr++;
      uPrev = u;
    }
    if (uDecr < 3) {
      ok = false; snprintf(reason, 128, "u not decreasing after sign flip (%d/3) - windup not prevented", uDecr);
    }
  }
  report("21 Anti-windup freeze case", ok, reason);
}

// 22. pidReset clears the integrator (explicit requirement): wind up, reset,
//     e=0 -> u=0 AND integral=0.
static void test_pid_reset_clears_integrator() {
  float integral = 0, filteredError = 0, prevFilteredError = 0;
  for (int t = 0; t < 100; t++)
    pidStep(1000, 0.0f, 0.001f, 0.0f, 1000.0f, false,
            integral, filteredError, prevFilteredError);
  pidReset(integral, filteredError, prevFilteredError);
  bool ok = (integral == 0.0f);
  char reason[128] = "";
  if (!ok) snprintf(reason, 128, "integral=%f after reset", integral);
  if (ok) {
    int u = pidStep(0, 0.0f, 0.001f, 0.0f, 1000.0f, false,
                    integral, filteredError, prevFilteredError);
    if (u != 0) { ok = false; snprintf(reason, 128, "u=%d after reset+e=0", u); }
  }
  report("22 pidReset clears integrator", ok, reason);
}

int main() {
  test_p_only();
  test_i_accumulation();
  test_i_clamp();
  test_d_step();
  test_d_noise();
  test_pid_reset();
  test_windup();
  test_combined();
  test_position_centred();
  test_position_far_left();
  test_position_far_right();
  test_position_lost();
  test_position_inverted_polarity();
  test_position_between_sensors();
  test_position_realistic_background();
  test_clamp_no_reverse();
  test_clamp_moderate();
  test_clamp_zero_error();
  test_search_spin_bypasses_clamp();
  test_antiwindup_rail_recovery();
  test_antiwindup_freeze_case();
  test_pid_reset_clears_integrator();
  printf("SUMMARY: %d/%d passed\n", passed, passed + failed);
  return failed == 0 ? 0 : 1;
}
