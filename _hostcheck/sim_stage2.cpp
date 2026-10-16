// sim_stage2: runs the UNCHANGED stage-2 sketch against the virtual alphabot
// build: PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h \
//          -x c++ lab1_stage2_encoders/lab1_stage2_encoders.ino _hostcheck/sim_stage2.cpp -o _hostcheck/sim2.exe
// run:   _hostcheck/sim2.exe
#include "sim_robot.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>

SerialMock Serial;            // the sketch's Serial
void setup();                 // provided by the .ino (compiled as C++)
void loop();

// one control step: deliver due serial input, advance the clock, integrate physics.
// loop() runs on EVERY tick like real hardware; the rx queue is DRAINED (one loop() call
// per pending char, matching how fast loop() spins on a 16 MHz UNO between physics steps)
static void tick(unsigned dtMs) {
  sim::setTime(sim::now() + dtMs);
  loop();                                 // sketch bookkeeping (speed window, snapshots)
  while (Serial.available() > 0) loop();  // handle ALL due input, not just one char
  sim::stepPhysics(dtMs);
}

static void runFor(unsigned ms) {
  tick(0);
  for (unsigned t = 0; t < ms; t += 5) tick(5);
}

static int fails = 0;
static void check(bool ok, const char* what) {
  sim::simOut(std::string(ok ? "PASS: " : "FAIL: ") + what);
  if (!ok) fails++;
}

// parse "enc L=<cnt>cnt ... | R=<cnt>cnt ..." out of the tx log
static bool parseEnc(const std::string& log, long& cl, long& cr) {
  size_t p = log.find("L=");
  if (p == std::string::npos) return false;
  cl = std::atol(log.c_str() + p + 2);
  size_t q = log.find("R=", p);
  if (q == std::string::npos) return false;
  cr = std::atol(log.c_str() + q + 2);
  return true;
}

static std::string ask(const char* keys) {    // send keys, run a tick, return and clear the answer
  sim::inject(0, keys);
  tick(5);
  std::string ans = sim::txLog;
  sim::txLog.clear();
  return ans;
}

static double kModel() {                      // mm per count the sim model uses
  return 3.14159265358979 * sim::WHEEL_D_MM / (sim::SLOTS * sim::EDGES);
}

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  // ===== S1. bench single channel: hand turn left wheel only, right counter stays put =====
  sim::simOut("== bench: single channel ==");
  ask("xz");                                    // bench mode on, counters zeroed
  sim::handTurn(3.0, 0.0);                      // exactly 3 revs by hand, left wheel only
  std::string ans = ask("e");
  long cl = 0, cr = 0;
  check(parseEnc(ans, cl, cr), "enc line parseable");
  check(cl == 3 * 2 * sim::SLOTS && cr == 0, "left counts 3 revs, right stays put");

  // ===== S2. bench counts per revolution: 10 revs == 10*2S both wheels =====
  sim::simOut("== bench: counts per revolution ==");
  ask("z");
  sim::handTurn(10.0, 10.0);
  ans = ask("e");
  parseEnc(ans, cl, cr);
  check(cl == 10 * 2 * sim::SLOTS && cr == 10 * 2 * sim::SLOTS, "10 revs -> 20S counts both wheels");

  // ===== S3. bench sign: reverse hand turn changes the COUNT by the right amount =====
  // NOTE: the SIGN of a hand turn is unknowable by design - a single-channel sensor has no
  // direction information, and the sketch derives sign from the last COMMANDED direction.
  // That limitation is exactly the discussion point the guide asks for in the report.
  // The sign check that matters is the DRIVEN one, in S4.
  sim::simOut("== bench: reverse hand turn magnitude ==");
  ask("z");
  long preL = 0, preR = 0;
  parseEnc(ask("e"), preL, preR);
  sim::handTurn(-2.0, -2.0);
  parseEnc(ask("e"), cl, cr);
  check(std::labs(cl - preL) == 2 * 2 * sim::SLOTS && std::labs(cr - preR) == 2 * 2 * sim::SLOTS,
        "reverse hand turn moves both counters by exactly 2 revs (sign is context, see report)");

  // ===== S4. driven sign: fwd -> positive, back -> counts decrease =====
  sim::simOut("== driven sign ==");
  ask("xz");                                    // live mode, zero
  // NOTE: the sim model has SIGN_FLIP=true (real-robot wiring). The stage-2 sketch has
  // SIGN=+1, so its '2' (bridge backward) is what physically rolls forward. We drive '2'
  // and expect positive counts: dirR/dirL come from the sketch's command, +1 for '1' only.
  // Actually with SIGN=+1 '2' sets dir=-1 and counts go negative; '1' sets dir=+1 and the
  // model rolls backward -> counts positive (sign = commanded). Either way the CHECK below
  // verifies the sketch contract, not the chassis direction - that is stage 3's floor test.
  ask("1");
  runFor(1000);
  parseEnc(ask("e"), cl, cr);
  check(cl >= 10 && cr >= 10, "fwd drive -> positive counts both wheels (sign = commanded dir)");
  ask("2");
  runFor(1200);
  long cl2 = 0, cr2 = 0;
  parseEnc(ask("e"), cl2, cr2);
  check(cl2 < cl && cr2 < cr, "reverse drive -> counts decrease");
  ask("c");

  // ===== S5. coast: counter grows during run-on, freezes after COAST_MS =====
  sim::simOut("== coast window ==");
  sim::resetRobot();
  ask("1");
  runFor(1500);
  long before = 0; cr = 0;
  parseEnc(ask("e"), before, cr);
  ask("c");                                     // drive off, robot coasts
  runFor(300);                                  // inside the coast window: run-on still rolling
  long during = 0; cr = 0;
  parseEnc(ask("e"), during, cr);
  check(during > before, "run-on still counts inside the coast window");
  runFor(sim::COAST_MS + 300);                  // window closes, snapshot taken; keep rolling
  long frozen = 0; cr = 0;
  parseEnc(ask("e"), frozen, cr);
  check(frozen >= during, "counter only grew until the snapshot, then froze");
  runFor(300);                                  // any residual motion must not count now
  long after = 0; cr = 0;
  parseEnc(ask("e"), after, cr);
  check(after == frozen, "post-freeze motion not counted");
  check(std::abs(sim::poseX) > 0, "sim confirms run-on was real (model sanity)");

  // ===== S6. standstill vibration: jitter with drive off must not count in live mode =====
  // NOTE the wiggle is asymmetric on purpose (forward crossing slightly deeper than back),
  // mimicking a disc that settles past an edge - exactly the false-count case the freeze exists for
  sim::simOut("== vibration vs freeze ==");
  sim::jitterWheel(200);
  long vib = 0; cr = 0;
  parseEnc(ask("e"), vib, cr);
  check(vib == frozen, "live mode: vibration produces no counts");
  ask("x");                                     // bench mode: raw edges, freeze off
  sim::jitterWheel(200);
  long vibBench = 0; cr = 0;
  parseEnc(ask("e"), vibBench, cr);
  check(vibBench > vib, "bench mode sees the same edges (freeze was the active ingredient)");
  ask("x");                                     // back to live

  // ===== S7. speed window sanity at pwm 30/60/90 =====
  sim::simOut("== speed window ==");
  struct PwmCase { int pwm; double tolPct; };
  PwmCase cases[] = { {30, 25.0}, {60, 15.0}, {90, 15.0} };
  for (auto& c : cases) {
    sim::resetRobot();                          // resets PHYSICS; sketch state (curPwm) persists
    if (c.pwm < 30) {                           // normalize pwm down first (max() clamps at 0)
      char dn[32]; memset(dn, '-', 30); dn[30] = 0;
      ask(dn);
    }
    char up[32];
    int presses = c.pwm / 5;
    memset(up, '+', presses); up[presses] = 0;
    ask(up);                                    // pwm -> target from 0
    ask("1");                                   // fwd at target pwm
    runFor(2500);                               // spool up + several full windows
    ans = ask("s");
    double trueV = std::abs(0.5 * (sim::wl.v_mm_s + sim::wr.v_mm_s));   // |speed| (direction is chassis-relative under SIGN_FLIP)
    float repL = 0, repR = 0;
    if (sscanf(ans.c_str(), "spd L=%fmm/s R=%fmm/s", &repL, &repR) != 2) {
      check(false, "speed line parseable");
      continue;
    }
    double lo = trueV * (1 - c.tolPct / 100.0), hi = trueV * (1 + c.tolPct / 100.0);
    bool okL = repL >= lo && repL <= hi, okR = repR >= lo && repR <= hi;
    char name[128];
    snprintf(name, sizeof name, "pwm %d: window speed within %d%% of truth (L %.1f R %.1f, truth %.1f)",
             c.pwm, (int)c.tolPct, repL, repR, trueV);
    check(okL && okR, name);
    ask("c");
  }

  // ===== S8. counts-per-window argument at pwm 35 (dead band edge) =====
  sim::simOut("== counts-per-window argument ==");
  {
    sim::resetRobot();
    char dn[32]; memset(dn, '-', 30); dn[30] = 0;
    ask(dn);                                    // pwm -> 0 (curPwm persists across resetRobot)
    ask("+++++++");                             // pwm 35, the model's dead band edge
    ask("1");
    runFor(3000);                               // several full 500 ms windows
    ans = ask("s");
    float repL = 0, repR = 0;
    if (sscanf(ans.c_str(), "spd L=%fmm/s R=%fmm/s", &repL, &repR) == 2) {
      double cntsL = repL * 0.5 / kModel();     // counts per 500 ms window
      double cntsR = repR * 0.5 / kModel();
      char name[96];
      snprintf(name, sizeof name, "pwm 35: %.1f/%.1f counts per 500ms window (>=5 wanted)", cntsL, cntsR);
      check(cntsL >= 5 && cntsR >= 5, name);
    } else {
      check(false, "speed line parseable at pwm 35");
    }
    ask("c");
  }

  // ===== S9. k sanity: reported mm matches sim wheel travel within quantisation =====
  sim::simOut("== k sanity: reported mm vs sim truth ==");
  {
    sim::resetRobot();
    char dn[32]; memset(dn, '-', 30); dn[30] = 0;
    ask(dn);                                    // pwm -> 0 (curPwm persists across resetRobot)
    ask("++++++++++++++++++++++++++++");        // pwm 90 (18 presses x 5)
    ask("z");
    ask("1");
    runFor(4000);                               // ~1 m at pwm 90 in the model
    ans = ask("e");
    long cl3 = 0, cr3 = 0;
    parseEnc(ans, cl3, cr3);
    double mmL = std::abs((double)cl3 * kModel());
    double trueL = std::abs(sim::wl.x_mm);
    char name[128];
    snprintf(name, sizeof name, "reported %.1f mm vs true wheel travel %.1f mm (<=1.5 counts quantisation)", mmL, trueL);
    check(std::abs(mmL - trueL) <= 1.5 * kModel(), name);
    ask("c");
  }

  std::printf("\n== stage2 scenario finished, fails=%d ==\n", fails);
  return fails ? 1 : 0;
}
