// Sensor-path unit tests: the REAL TRSensors.cpp compiled unmodified for the
// host, driven through the TLC1543 pin simulator in Arduino.h.
// Build (git-bash):
//   g++ -std=c++17 -I_hostcheck/s6_libtest -ITRSensors/TRSensors \
//       -include _hostcheck/s6_libtest/Arduino.h -x c++ \
//       TRSensors/TRSensors/TRSensors.cpp _hostcheck/s6_libtest/sensor_path_test.cpp \
//       -o _hostcheck/s6_libtest/sensor_path_test.exe
#include "TRSensors.h"
#include <cmath>

static TRSensors trs;
static unsigned int sv[5];
static int failures = 0;

static void check(const char* name, bool ok, const char* detail = "") {
  printf("%-58s %s %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}
static void expectRoughly(const char* name, int got, int want, int tol) {
  char d[128];
  snprintf(d, sizeof d, "(got %d, want ~%d)", got, want);
  check(name, std::abs(got - want) <= tol, d);
}

// program what the simulated TLC1543 returns per channel
static void setRaw(int s0, int s1, int s2, int s3, int s4) {
  g_tlc_raw[0]=s0; g_tlc_raw[1]=s1; g_tlc_raw[2]=s2; g_tlc_raw[3]=s3; g_tlc_raw[4]=s4; g_tlc_raw[5]=0;
}

// Two-phase sweep calibration: passes over background then line (the 'W' pattern).
static void calibrateTwoPhase() {
  trs = TRSensors();   // fresh min/max (1023 / 0)
  // phase 1: all sensors over background (raw 800), sensor 2 slightly dirty (750)
  setRaw(800, 800, 750, 800, 800);
  for (int i = 0; i < 50; i++) trs.calibrate();
  // phase 2: sweep so each sensor in turn sits on the line (raw ~200)
  for (int s = 0; s < 5; s++) {
    int r[5] = {800, 800, 800, 800, 800}; r[s] = 200;
    setRaw(r[0], r[1], r[2], r[3], r[4]);
    for (int i = 0; i < 40; i++) trs.calibrate();
  }
}

int main() {
  // T0 FIRST (before any readLine call anywhere): the boot-state lost side.
  // readLine's last_value is a FUNCTION-static initialized to 0 = "left".
  {
    calibrateTwoPhase();
    setRaw(800, 800, 800, 800, 800);
    int e = trs.readLine(sv, 0);
    char d[64]; snprintf(d, sizeof d, "(e=%d)", e);
    check("T0: fresh boot (static last_value=0) + all background -> lost -1 (LEFT)", e == -1, d);
    // T0 left last_value=0 ("left"). Fresh-object probe: set "seen right" first,
    // then replace the object and lift off.
    {
      int r[5] = {800, 800, 800, 200, 800};
      setRaw(r[0], r[1], r[2], r[3], r[4]);
      trs.readLine(sv, 0);          // last_value -> 3000 (right)
      trs = TRSensors();            // brand-new object (uncalibrated: min=1023 max=0)
      setRaw(800, 800, 800, 800, 800);
      e = trs.readLine(sv, 0);
      snprintf(d, sizeof d, "(e=%d)", e);
      // QUIRK (documented): with constructor-default ranges the denominator
      // (0-1023) is nonzero-but-garbage, every value calibrates to ~0, inverts
      // to ~1000, so ALL sensors count "on line" -> readLine returns a FAKE
      // 2000 (centred), never "lost". Harmless in the sketch (setup() always
      // calibrates or loads EEPROM before loop() runs), but never read before
      // calibrating. Also: last_value (3000) survived the object replacement
      // either way - it is a function-static, not a member.
      check("T0: uncalibrated object + background -> FAKE 'centred 2000' (never lost)",
            e == 2000, d);
    }
  }

  printf("=== T1: AnalogRead pipeline (real bit-bang code vs TLC1543 sim) ===\n");
  {
    setRaw(101, 202, 303, 404, 505);
    unsigned long before = g_tlc_passCount;
    trs.AnalogRead(sv);
    char d[128];
    snprintf(d, sizeof d, "(got %u %u %u %u %u)", sv[0], sv[1], sv[2], sv[3], sv[4]);
    check("AnalogRead returns programmed raw values 0..4",
          sv[0]==101 && sv[1]==202 && sv[2]==303 && sv[3]==404 && sv[4]==505, d);
    snprintf(d, sizeof d, "(got %lu)", g_tlc_passCount - before);
    check("exactly 6 CS passes per AnalogRead (5+1 pipeline flush)",
          g_tlc_passCount - before == 6, d);
  }

  printf("=== T2: calibrate() update logic — which array holds the LINE? ===\n");
  {
    calibrateTwoPhase();
    char d[160];
    snprintf(d, sizeof d, "(MIN=%u,%u,%u,%u,%u MAX=%u,%u,%u,%u,%u)",
             trs.calibratedMin[0], trs.calibratedMin[1], trs.calibratedMin[2], trs.calibratedMin[3], trs.calibratedMin[4],
             trs.calibratedMax[0], trs.calibratedMax[1], trs.calibratedMax[2], trs.calibratedMax[3], trs.calibratedMax[4]);
    bool minIsLine = true, maxIsBg = true;
    for (int i = 0; i < 5; i++) {
      if (trs.calibratedMin[i] != 200) minIsLine = false;   // line raw
      if (trs.calibratedMax[i] != 800) maxIsBg = false;     // background raw
    }
    check("calibratedMin = LINE end (200) after background->line sweep", minIsLine, d);
    check("calibratedMax = BACKGROUND end (800)", maxIsBg, d);
    // NOTE: this reveals a library subtlety — calibrate() crosses its min/max
    // update (calibratedMax updated from THIS call's min, calibratedMin from
    // THIS call's max), so extremes can only widen monotonically call over
    // call; the dirty-sensor 750 seen only in phase 1 was absorbed once the
    // sweep's own min (200) exceeded it.
    // So with a DARK line: raw(line) < raw(bg) => calibratedMin < calibratedMax.
    // readCalibrated maps line -> 0, background -> 1000 (dark end = 0).
    check("contrast covered on every sensor (no zero range)",
          minIsLine && maxIsBg, "");
  }

  printf("=== T3: readCalibrated rescale ===\n");
  {
    // keep T2 calibration (MIN=200 = line, MAX=800 = background);
    // raw 500 -> cal = (500-200)*1000/600 = 500
    // NOTE (host-vs-AVR divergence, documented): the library computes
    //   x = ((signed long)raw - (unsigned)min) * 1000 / (unsigned)denominator
    // On AVR (16-bit int) the unsigned int operands promote to signed long ->
    // raw BELOW min gives x<0 -> clamps to 0 (correct). On this host g++
    // (32-bit long) a negative numerator converts to unsigned for the division
    // -> huge x -> clamps to 1000. So the below-min/above-max clamp is NOT
    // host-testable with the real library source; it is reasoned, not run.
    setRaw(200, 500, 800, 200, 200);   // all raw within [min,max]
    trs.readCalibrated(sv);
    char d[128];
    snprintf(d, sizeof d, "(got %u %u %u %u %u)", sv[0], sv[1], sv[2], sv[3], sv[4]);
    check("readCalibrated: line=0, mid=500, background=1000 (in-range rescale exact)",
          sv[0]==0 && sv[1]==500 && sv[2]==1000 && sv[3]==0 && sv[4]==0, d);
    // denominator == 0 (stationary calibration): fresh object, one calibrate over flat background
    trs = TRSensors();
    setRaw(700, 700, 700, 700, 700);
    trs.calibrate();
    setRaw(650, 750, 700, 700, 700);
    trs.readCalibrated(sv);
    snprintf(d, sizeof d, "(got %u %u %u %u %u)", sv[0], sv[1], sv[2], sv[3], sv[4]);
    // after 1 calibrate: min=max=700 for all (10 reads same) -> denom 0 -> x=0 everywhere... but the
    // crossed update in calibrate(): calibratedMax = max(0, min_this) = 700, calibratedMin = min(1023, max_this) = 700
    check("stationary calibration => denominator 0 => all zeros (no contrast)",
          sv[0]==0 && sv[1]==0 && sv[2]==0 && sv[3]==0 && sv[4]==0, d);
  }

  printf("=== T4: readLine dark line (white_line=0), full sweep ===\n");
  {
    calibrateTwoPhase();
    // helper lambda: place a synthetic line at fractional position p in [0,4]
    auto readAt = [&](double p, bool white) -> int {
      // each sensor's calibrated-domain raw: interpolate line(200)/background(800)
      int r[5];
      for (int i = 0; i < 5; i++) {
        double d = std::abs(p - i);            // distance in sensor spacings
        double t = d < 1.0 ? (1.0 - d) : 0.0;  // 1 = exactly on line
        // background 800 <-> line 200 (dark line); white line mirror handled by caller via white_line
        r[i] = (int)(800 - 600 * t);
      }
      setRaw(r[0], r[1], r[2], r[3], r[4]);
      return trs.readLine(sv, white ? 1 : 0);
    };
    expectRoughly("dark line centred (p=2) -> position ~2000", readAt(2.0, false), 2000, 5);
    expectRoughly("dark line at sensor 0 (p=0) -> ~0", readAt(0.0, false), 0, 5);
    expectRoughly("dark line at sensor 4 (p=4) -> ~4000", readAt(4.0, false), 4000, 5);
    expectRoughly("dark line between sensors (p=1.5) -> ~1500", readAt(1.5, false), 1500, 10);
    expectRoughly("dark line between sensors (p=2.5) -> ~2500", readAt(2.5, false), 2500, 10);
    // monotonicity across the sweep
    bool mono = true; int prev = -100;
    for (double p = 0.0; p <= 4.0; p += 0.1) {
      int pos = readAt(p, false);
      if (pos < prev) { mono = false; break; }
      prev = pos;
    }
    check("position monotonic 0->4000 across the strip", mono, "");
    // write-back: sensorValues now hold the polarity-corrected values (line HIGH)
    readAt(2.0, false);
    char d[128];
    snprintf(d, sizeof d, "(sv=%u,%u,%u,%u,%u)", sv[0], sv[1], sv[2], sv[3], sv[4]);
    check("readLine writes INVERTED values back (dark line -> line sensor HIGH)",
          sv[0] < 100 && sv[1] < 100 && sv[2] > 900 && sv[3] < 100 && sv[4] < 100, d);
  }

  printf("=== T5: lost-line detection and thresholds (exact library code) ===\n");
  {
    calibrateTwoPhase();
    // side memory carries over from T0 (last seen right = 3000)
    setRaw(800, 800, 800, 800, 800);
    int e = trs.readLine(sv, 0);
    char d[64]; snprintf(d, sizeof d, "(e=%d)", e);
    check("all background -> lost, side memory from earlier reads (right)", e == 4001, d);

    // line last seen on the RIGHT (p=3), then lift off
    int r[5] = {800, 800, 800, 200, 800};
    setRaw(r[0], r[1], r[2], r[3], r[4]);
    trs.readLine(sv, 0);
    setRaw(800, 800, 800, 800, 800);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("lost after line seen RIGHT -> +4001", e == 4001, d);

    // line last seen LEFT of centre (p=1), then lift off
    r[3] = 800; r[1] = 200;
    setRaw(r[0], r[1], r[2], r[3], r[4]);
    trs.readLine(sv, 0);
    setRaw(800, 800, 800, 800, 800);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("lost after line seen LEFT -> -1", e == -1, d);

    // boundary: last seen exactly at centre 2000 -> 'else' branch -> 4001
    r[1] = 800; r[2] = 200;
    setRaw(r[0], r[1], r[2], r[3], r[4]);
    trs.readLine(sv, 0);
    setRaw(800, 800, 800, 800, 800);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("lost after line seen AT centre (2000) -> +4001 (>=2000 counts right)", e == 4001, d);

    // strict >300 on-line test: line exactly at the threshold
    // place line so that inverted centre value == 300 exactly: raw = 500 (cal 545 -> inverted 455)... use direct:
    // sensor2 raw such that calibrated = 700 => inverted = 300 -> NOT on_line (300 > 300 false)
    calibrateTwoPhase();
    setRaw(800, 800, 340, 800, 800);   // cal = (340-200)*1000/600 = 233 -> inverted 767 (on)
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("cal 233 -> inverted 767 > 300 -> on_line (visible)", e != -1 && e != 4001, d);
    setRaw(800, 800, 620, 800, 800);   // cal = 700 -> inverted 300 -> NOT > 300 -> lost
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("inverted exactly 300 -> NOT on_line -> lost (strict >300)", e == -1 || e == 4001, d);

    // >50 noise gate: values <= 50 are EXCLUDED from the average (strict >).
    // 3 sensors at inverted 60 (in avg), 1 at inverted 45 (excluded), 1 background.
    // all below 300 -> lost regardless; the exclusion is verified by the position
    // in the visible case below.
    auto rawForInverted = [&](int v) { return 200 + (1000 - v) * 0.6; };
    setRaw((int)rawForInverted(60), 800, (int)rawForInverted(45), 800, (int)rawForInverted(60));
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("values 60/60/45 all < 300 -> lost (45 also excluded from avg)", e == -1 || e == 4001, d);
    // noise-gate proof in the VISIBLE case: sensors 0/1 at inverted 60 + sensor 2
    // on the line: the two 60s ARE averaged. rawForInverted truncation makes the
    // actual inverted values 60 and 61/52; expected pos computed from measured
    // behaviour: (60*0 + v1*1000 + 1000*2000)/(60+v1+1000); v1=61 -> 1939.
    setRaw((int)rawForInverted(60), (int)rawForInverted(60), 200, 800, 800);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("noise-gate proof: two ~60s pulled pos below 2000", e < 2000, d);
    // discriminator: sensor1 exactly ON the line (v=1000) vs v1~52 — both included
    // (both >50), positions differ markedly; and a <=50 value would exclude sensor1.
    setRaw((int)rawForInverted(60), (int)rawForInverted(50), 200, 800, 800);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    // sensor1 inverted = 1000-49=951? raw=200+(950)*0.6=770 -> cal=950 -> inv 50 -> EXCLUDED
    // so pos = (60*0 + 1000*2000)/(60+1000) = 1886
    check("inverted 50 EXCLUDED from average (>50 strict): pos = 1886", e == 1886, d);
    // division-by-zero probe: on_line true needs >300 somewhere; if ONLY that sensor is
    // above 300 the sum >= its value > 300 -> avg/sum never divides by zero. Verify:
    setRaw(800, 800, 200, 800, 800);   // only sensor 2 on line, inverted ~1000
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("single-sensor line -> valid position (no div-by-zero in avg/sum)", e == 2000, d);
  }

  printf("=== T6: white_line=1 mirror (bright line) ===\n");
  {
    calibrateTwoPhase();
    // BRIGHT line: raw HIGH on line (900), background LOW (300).
    // recalibrate for that polarity: sweep background 300 then line 900
    trs = TRSensors();
    setRaw(300, 300, 300, 300, 300);
    for (int i = 0; i < 50; i++) trs.calibrate();
    for (int s = 0; s < 5; s++) {
      int r[5] = {300, 300, 300, 300, 300}; r[s] = 900;
      setRaw(r[0], r[1], r[2], r[3], r[4]);
      for (int i = 0; i < 40; i++) trs.calibrate();
    }
    setRaw(300, 300, 900, 300, 300);
    int e = trs.readLine(sv, 1);
    char d[64]; snprintf(d, sizeof d, "(e=%d)", e);
    check("bright line centred with white_line=1 -> ~2000", e == 2000, d);
    setRaw(300, 300, 300, 300, 300);
    e = trs.readLine(sv, 1);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("bright-line background -> lost", e == -1 || e == 4001, d);
    // and with white_line=0 on bright-line data: the inversion does NOT kill the
    // line signal — it INVERTS it, so the line reads ~0 and the background ~1000.
    // The result is a VALID-looking position at the mirrored side — the dangerous
    // wrong-polarity symptom (steers away, never reports lost).
    setRaw(300, 300, 900, 300, 300);
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    check("bright line read with white_line=0 -> WRONG-SIDE valid position (mirrored)",
          e == 2000, d);
    setRaw(900, 300, 300, 300, 300);   // line under sensor 0 (bright-line data)
    e = trs.readLine(sv, 0);
    snprintf(d, sizeof d, "(e=%d)", e);
    // sensor0: cal=1000, inverted 0 -> excluded; sensors 1-4: cal=0, inverted 1000
    // pos = (1000*1000+1000*2000+1000*3000+1000*4000)/4000 = 2500 (straddle artifact)
    check("bright-line sensor0 with white_line=0 -> pos 2500 (line at far L reads 2500!)",
          e == 2500, d);
  }

  printf("=== T7: unsigned-int trap (the library example's bug, documentary) ===\n");
  {
    int e = -1;
    unsigned int ue = (unsigned int)(e - 2000);   // what an unsigned error variable would hold on AVR (16-bit int)
    char d[64]; snprintf(d, sizeof d, "(unsigned 16-bit holds %u)", (unsigned)(uint16_t)ue);
    check("sentinel -1 stored in unsigned 16-bit becomes 63535 (sketch uses signed - immune)",
          (uint16_t)ue == 63535, d);
  }

  printf("\n%d checks, %d failures\n", 0, failures);
  return failures ? 1 : 0;
}
