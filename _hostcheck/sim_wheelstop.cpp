// sim_wheelstop: verifies the one-wheel-stopped watchdog in the stage-3 sketch -
// when ONE wheel stops rotating while the other still turns (fault injection: per-wheel
// dead band override), BOTH wheels must stop within ~WHEEL_STOP_MS + margin, and the
// distance counters must freeze at (approximately) the same physical stop.
// build: PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h \
//          -x c++ lab1_stage3_straight/lab1_stage3_straight.ino _hostcheck/sim_wheelstop.cpp -o _hostcheck/simwheelstop.exe
#include "sim_robot.h"
#include <cstdio>
#include <cmath>
#include <string>

SerialMock Serial;
void setup();
void loop();

static void tick(unsigned dtMs) {
  sim::setTime(sim::now() + dtMs);
  loop();
  while (Serial.available() > 0) loop();
  sim::stepPhysics(dtMs);
}
static void runFor(unsigned ms) {
  tick(0);
  for (unsigned t = 0; t < ms; t += 5) tick(5);
}
static int fails = 0;
static void check(bool ok, const char* what) {
  std::printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}
static bool parseEnc(const std::string& log, long& cl, long& cr) {
  size_t p = log.find("L=");
  if (p == std::string::npos) return false;
  cl = std::atol(log.c_str() + p + 2);
  size_t q = log.find("R=", p);
  if (q == std::string::npos) return false;
  cr = std::atol(log.c_str() + q + 2);
  return true;
}
static std::string ask(const char* keys) {
  sim::inject(0, keys);
  tick(5);
  std::string ans = sim::txLog;
  sim::txLog.clear();
  return ans;
}
// tick until the sketch prints the needle or the budget runs out; returns true on hit
static bool waitLine(const char* needle, unsigned budgetMs, std::string& hit) {
  for (unsigned t = 0; t < budgetMs; t += 5) {
    tick(5);
    size_t p = sim::txLog.find(needle);
    if (p != std::string::npos) {
      size_t e = sim::txLog.find('\n', p);
      hit = sim::txLog.substr(p, e == std::string::npos ? std::string::npos : e - p);
      sim::txLog.clear();
      return true;
    }
  }
  sim::txLog.clear();
  return false;
}

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  // ---- S1: RIGHT wheel jammed mid-drive -> both stop, line printed, counters freeze ----
  sim::simOut("== S1: right wheel stops mid-drive ==");
  ask("z");                                   // zero counters, idle
  sim::inject(0, "F2000");                    // long drive: the fault must land mid-motion
  runFor(2500);                               // spool up through the ramp to cruise, both rolling
  long cl0 = 0, cr0 = 0;
  parseEnc(ask("e"), cl0, cr0);
  check(cl0 > 30 && cr0 > 30, "both wheels well into the drive before the fault (>30 counts)");
  check(std::abs(sim::poseX) > 100.0, "robot actually rolling before the fault");
  sim::DEADBAND_R = 255;                      // FAULT: right wheel can no longer turn (duty <= 255)
  std::string line;
  bool fired = waitLine("wheel stopped", 3000, line);
  check(fired, "watchdog fired: 'err: wheel stopped (R ...)' within 3 s");
  check(line.find("(R") != std::string::npos, "the stopped wheel is identified as R");
  // timing: the watchdog window is 400 ms; after the fault the right wheel decays to zero
  // in ~0.12 s, silent from then on -> fire ~400-550 ms after injection
  runFor(600);                                // give brake + coast window time to settle
  long cl1 = 0, cr1 = 0;
  parseEnc(ask("e"), cl1, cr1);
  runFor(600);                                // any later growth means counting did not freeze
  long cl2 = 0, cr2 = 0;
  parseEnc(ask("e"), cl2, cr2);
  check(cl2 == cl1 && cr2 == cr1, "counters frozen after the stop (no growth in 600 ms)");
  check(std::abs(sim::wl.v_mm_s) < 2.0 && std::abs(sim::wr.v_mm_s) < 2.0,
        "both wheels physically stopped (|v| < 2 mm/s)");
  check(std::abs(sim::poseY) < 40.0,
        "lateral drift during the fault window stayed small (< 40 mm)");
  sim::DEADBAND_R = -1;                       // fault off
  ask("b");                                   // clear manual states

  // ---- S2: LEFT wheel jammed -> same behaviour, identified as L ----
  sim::simOut("== S2: left wheel stops mid-drive ==");
  sim::resetRobot();
  ask("z");
  sim::inject(0, "F300");
  runFor(400);
  sim::DEADBAND_L = 255;                      // FAULT: left wheel stuck
  bool fired2 = waitLine("wheel stopped", 3000, line);
  check(fired2, "watchdog fired for the left wheel within 3 s");
  check(line.find("(L") != std::string::npos, "the stopped wheel is identified as L");
  sim::DEADBAND_L = -1;
  ask("b");

  // ---- S3: false-positive check - a healthy drive must NEVER see the watchdog ----
  sim::simOut("== S3: healthy drive, no false positive ==");
  sim::resetRobot();
  ask("z");
  sim::inject(0, "F500");
  bool falseHit = false;
  for (unsigned t = 0; t < 6000; t += 5) {
    tick(5);
    if (sim::txLog.find("wheel stopped") != std::string::npos) { falseHit = true; break; }
    if (sim::txLog.find("done") != std::string::npos) break;
  }
  sim::txLog.clear();
  check(!falseHit, "healthy F500 completes with no 'wheel stopped' line");
  check(sim::txLog.find("done") != std::string::npos || std::abs(sim::poseX) > 400.0,
        "healthy drive actually covered ~500 mm");

  // ---- S4: recovery - after the abort, a new command drives normally again ----
  sim::simOut("== S4: recovery after watchdog abort ==");
  sim::resetRobot();
  ask("z");
  sim::DEADBAND_R = 255;                      // fault again...
  sim::inject(0, "F400");
  runFor(120);
  waitLine("wheel stopped", 3000, line);
  sim::DEADBAND_R = -1;                       // ...fault cleared
  sim::inject(0, "F150");                     // operator retries
  bool done4 = false;
  for (unsigned t = 0; t < 6000; t += 5) {
    tick(5);
    if (sim::txLog.find("done") != std::string::npos) { done4 = true; break; }
  }
  sim::txLog.clear();
  check(done4, "recovery: next drive completes with a done line");
  check(std::abs(sim::poseX) > 100.0, "recovery drive really moved the robot");

  // ---- S5: backwards drive, fault mid-way -> watchdog must work with negative counts ----
  sim::simOut("== S5: backward drive, right wheel stops ==");
  sim::resetRobot();
  ask("z");
  sim::inject(0, "B300");
  runFor(400);
  sim::DEADBAND_R = 255;
  bool fired5 = waitLine("wheel stopped", 3000, line);
  check(fired5, "watchdog fires in backward drive too (counts negative)");
  sim::DEADBAND_R = -1;
  ask("b");

  // ---- S6: HAND HOLD - the real-robot failure mode that broke v1 ----
  // the held wheel is not a clean stall: it jerks and produces occasional edges,
  // which reset a pure silence timer forever. model: right wheel mostly held but
  // jittering +/-1 count every ~100 ms. the ratio detector must catch it.
  sim::simOut("== S6: hand hold with jerk edges (the v1 failure mode) ==");
  sim::resetRobot();
  ask("z");
  sim::inject(0, "F2000");
  runFor(2500);                               // both wheels at cruise
  sim::DEADBAND_R = 255;                      // wheel held (firm hand: tyre does not slide freely)
  for (int i = 0; i < 40 && sim::txLog.find("wheel stopped") == std::string::npos; i++) {
    runFor(250);                              // hold for 250 ms...
    sim::emitEdges(sim::wr, 3.14159265358979 * sim::WHEEL_D_MM / (sim::SLOTS * sim::EDGES),
                   sim::isr1, sim::isr1Mode); // ...then ~1 count of jerk slip through the real isr
  }
  bool fired6 = sim::txLog.find("wheel stopped") != std::string::npos;
  sim::txLog.clear();
  check(fired6, "hand hold with jerk edges: guard fired within 4 s");
  sim::DEADBAND_R = -1;
  ask("b");

  // ---- S7: MANUAL MODE - guard must work there too (v1 only watched commanded drives) ----
  sim::simOut("== S7: manual mode, right wheel held ==");
  sim::resetRobot();
  ask("z");
  ask("1");                                   // manual fwd, open loop
  runFor(2500);                               // both wheels rolling
  sim::DEADBAND_R = 255;
  bool fired7 = false;
  for (int i = 0; i < 40 && !fired7; i++) {
    runFor(250);                              // firm hold, 1 jerk count per 250 ms
    sim::emitEdges(sim::wr, 3.14159265358979 * sim::WHEEL_D_MM / (sim::SLOTS * sim::EDGES),
                   sim::isr1, sim::isr1Mode);
    if (sim::txLog.find("wheel stopped") != std::string::npos) fired7 = true;
  }
  sim::txLog.clear();
  check(fired7, "manual mode: guard fired on the held wheel");
  check(std::abs(sim::poseY) < 60.0, "manual mode: robot did not pivot far (stopped early)");
  sim::DEADBAND_R = -1;
  ask("0"); ask("b");

  // ---- S8: guard must NOT fire when a wheel is pushed by hand with the drive off ----
  sim::simOut("== S8: drive off, hand push must not trip the guard ==");
  sim::resetRobot();
  ask("z");
  ask("b");                                   // brake, drive off
  runFor(200);
  sim::handTurn(1.0, 0.3);                    // push: both wheels move, L more than R
  runFor(800);                                // long past any guard window
  check(sim::txLog.find("wheel stopped") == std::string::npos, "no guard trip on a hand push");
  sim::txLog.clear();
  ask("x");                                   // bench sanity: raw edges still count
  sim::handTurn(1.0, 1.0);
  long bl = 0, br = 0;
  parseEnc(ask("e"), bl, br);
  check(bl > 30 && br > 30, "bench mode still counts after all the guard traffic");
  ask("x");

  std::printf("\n%s (fails=%d)\n", fails == 0 ? "ALL SCENARIOS PASSED" : "FAILURES PRESENT", fails);
  return fails == 0 ? 0 : 1;
}
