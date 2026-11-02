// sim_fault: stage-7 wheel-sync / fault-recovery verification (host-side, virtual robot).
// build (MSYS2 g++, PATH first, ONE line):
//   PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -Wall -Wextra -I_hostcheck -include _hostcheck/Arduino.h -x c++ lab1_stage7_faultrecovery/lab1_stage7_faultrecovery.ino _hostcheck/sim_fault.cpp -o _hostcheck/simfault.exe
// scenarios:
//   S1 healthy G0.5 regression - done line, no fault output
//   S2 one-wheel stall mid-drive -> hold -> cause cleared -> resync -> ORIGINAL target done
//   S3 hand-hold WITH jerk edges (the guard-v1 killer) -> still detects via displacement
//   S4 re-command during hold (G0.3) -> fresh drive, done G0.3
//   S5 both wheels freeze mid-drive -> "drive blocked" abort, then recovery drive
//   S6 full breakaway slip (L spins ~5x, zero ground) -> rate detector, MIN-progress, done
//   S7 S key: none before any fault, meaningful record after
//   S8 turn regression: L90 still completes (turn path untouched by the stage-7 layer)
#include "Arduino.h"
#include <cstdio>
#include <cstring>
#include <string>

SerialMock Serial;
void setup();
void loop();

static int fails = 0;

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
static bool waitLog(const char* needle, unsigned timeoutMs) {
  for (unsigned t = 0; t < timeoutMs; t += 10) {
    tick(10);
    if (sim::txLog.find(needle) != std::string::npos) return true;
  }
  return false;
}
static void clearLog() { sim::txLog.clear(); }
static bool has(const char* needle) { return sim::txLog.find(needle) != std::string::npos; }

// pull "got=" out of the first done line (done ... got=<float>mm ...)
static bool gotValue(float& out) {
  size_t p = sim::txLog.find("done");
  if (p == std::string::npos) return false;
  p = sim::txLog.find("got=", p);
  if (p == std::string::npos) return false;
  out = (float)std::atof(sim::txLog.c_str() + p + 4);
  return true;
}

static void worldReset() {           // between scenarios: sim + robot back to a known state
  sim::MAX_MM_S = 592.0;             // stage-7 calibrated plant (see note at the top of main)
  sim::DEADBAND_PWM = 75; sim::DEADBAND_L = -1; sim::DEADBAND_R = -1;
  sim::SLIP_L = 0.0; sim::SLIP_R = 0.0;
  sim::resetRobot();
  clearLog();
}

static void check(bool ok, const char* name, const char* detail) {
  std::printf("%s: %s (%s)\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok) fails++;
}

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); clearLog();

  // stage-7 plant calibration: the frozen nominal plant (300 mm/s @ pwm 255) puts
  // pwm 85 at ~100 mm/s, which no absolute windowed detector could separate from a
  // stall; the floor-calibrated value (stage-2/3 map: pwm 130 -> 302 mm/s under load)
  // makes the sketch's PWM_PER_MMS true in the harness. ONLY this suite sets it -
  // the frozen suites keep their bit-identical calibration.
  sim::MAX_MM_S = 592.0;

  // --- S1 healthy drive regression ---------------------------------------
  worldReset();
  sim::inject(0, "G0.5");
  check(waitLog("done", 6000), "S1 healthy G0.5 done", "done line within 6 s");
  {
    float got = -1; bool ok = gotValue(got);
    char b[80]; std::snprintf(b, sizeof b, "got=%.1f poseX=%.1f", got, sim::poseX);
    check(ok && got > 450 && got < 550 && sim::poseX > 450 && sim::poseX < 550,
          "S1 healthy accuracy", b);
    check(!has("warn:") && !has("err:"), "S1 no false fault", has("warn:") ? "warn seen" : "clean");
  }
  // S key before any fault (must read state=none) - probed here because S1 is the
  // only place in the suite where no fault has happened yet
  clearLog();
  sim::inject(sim::now() + 10, "S");
  runFor(300);
  check(has("flt #0") && has("state=none"), "S7 S key state=none before faults",
        has("state=none") ? "none" : "unexpected");

  // --- S2 one-wheel stall -> hold -> resync -> original target -----------
  worldReset();
  sim::inject(0, "G0.6");
  runFor(500);                                   // ~130 mm of healthy travel
  sim::DEADBAND_L = 250;                         // LEFT wheel jams (duty 130 << 250)
  bool held = waitLog("wheel stopped", 3000);    // warn line names the side in line 1
  check(held, "S2 hold entered", held ? "warn line seen" : "NO hold within 3 s");
  check(has("warn: L wheel stopped"), "S2 names L",
        has("warn: L wheel stopped") ? "left named in warn" : "wrong/missing side");
  {
    long minCnt = std::labs(sim::wl.ticks) < std::labs(sim::wr.ticks)
                ? std::labs(sim::wl.ticks) : std::labs(sim::wr.ticks);
    char b[80]; std::snprintf(b, sizeof b, "MIN progress at hold = %ld cnt (~%.0f mm)",
                              minCnt, minCnt * 5.1087);
    check(minCnt > 15, "S2 held with real progress banked", b);
  }
  runFor(1200);                                  // hold: brake + alternating test pulses
  double poseAtHold = sim::poseX;
  sim::DEADBAND_L = -1;                          // cause gone: left wheel free again
  bool synced = waitLog("sync: fault cleared", 6000);
  check(synced, "S2 resync after cause cleared", synced ? "sync line seen" : "NO sync in 6 s");
  {
    double yDrift = sim::poseY;
    char b[80]; std::snprintf(b, sizeof b, "poseY=%.1f (pulses must not add yaw)", yDrift);
    check(yDrift < 60, "S2 hold did not yaw the chassis", b);
  }
  bool done2 = waitLog("done", 8000);
  {
    float got = -1; bool ok = done2 && gotValue(got);
    char b[120]; std::snprintf(b, sizeof b, "done=%d got=%.1f (target 600) poseX=%.1f",
                               (int)done2, got, sim::poseX);
    check(ok && got > 520 && got < 680, "S2 completes ORIGINAL target", b);
    check(sim::poseX > poseAtHold + 100, "S2 continued forward after resync", b);
  }

  // --- S3 hand-hold with jerk edges (v1 killer) --------------------------
  worldReset();
  sim::inject(0, "G0.5");
  runFor(400);
  sim::DEADBAND_L = 250;                         // wheel held...
  for (int i = 0; i < 8; i++) {                  // ...but jerking: ~1 edge every 300 ms
    runFor(300);
    sim::handTurn(0.0004, 0.0);                  // left-only twitch through the real ISR path
    if (has("wheel stopped")) break;
  }
  check(has("wheel stopped - both braked"), "S3 jerk hold detected",
        has("(L)") ? "L named despite jerk edges" : "no verdict");
  clearLog();
  sim::inject(sim::now() + 10, "c");             // operator abort out of hold
  runFor(300);
  check(!has("err:"), "S3 c aborts cleanly from hold", has("err:") ? "err seen" : "clean");

  // --- S4 re-command during hold -----------------------------------------
  worldReset();
  sim::inject(0, "G0.8");
  runFor(500);
  sim::DEADBAND_L = 250;
  check(waitLog("wheel stopped", 3000), "S4 hold entered", "warn seen");
  runFor(600);
  sim::DEADBAND_L = -1;                          // cause gone BEFORE the re-command: the fresh
  clearLog();                                    // drive must not inherit the open fault
  sim::inject(sim::now() + 10, "G0.3");          // operator re-commands mid-hold
  bool go4 = waitLog("go cmd=G0.300m", 2000);
  check(go4, "S4 fresh G accepted during hold", go4 ? "go line seen" : "NO go");
  bool done4 = waitLog("done", 6000);
  {
    float got = -1; bool ok = done4 && gotValue(got);
    char b[80]; std::snprintf(b, sizeof b, "done=%d got=%.1f (fresh target 300)", (int)done4, got);
    check(ok && got > 260 && got < 340, "S4 fresh drive completes", b);
  }

  // --- S5 both wheels freeze mid-drive ------------------------------------
  worldReset();
  sim::inject(0, "G0.6");
  runFor(500);
  sim::DEADBAND_PWM = 250;                       // operator grabs the whole robot
  bool blocked = waitLog("drive blocked (both wheels)", 3000);
  check(blocked, "S5 both-wheel freeze aborts", blocked ? "err line seen" : "NO abort in 3 s");
  double x5 = sim::poseX;
  runFor(500);
  char b5[80]; std::snprintf(b5, sizeof b5, "drift after abort = %.2f mm", sim::poseX - x5);
  check(sim::poseX - x5 < 5, "S5 braked, not coasting", b5);
  sim::DEADBAND_PWM = 75;
  clearLog();
  sim::inject(sim::now() + 10, "G0.2");          // recovery drive
  bool done5 = waitLog("done", 5000);
  check(done5, "S5 recovery drive after abort", done5 ? "done seen" : "NO done");

  // --- S6 full-breakaway slip on L ----------------------------------------
  worldReset();
  sim::inject(0, "G0.5");
  runFor(400);                                   // ~100 mm healthy
  sim::SLIP_L = 1.0;                             // L: spins ~5x (bench free rate), zero ground
  bool slip = waitLog("warn: slip L wheel", 3000);
  check(slip, "S6 slip rate detector fires", slip ? "warn seen" : "NO slip warn in 3 s");
  runFor(1200);                                  // cruise on MIN-progress while slipping
  bool done6 = waitLog("done", 8000);
  {
    float got = -1; bool ok = done6 && gotValue(got);
    // MIN progress cannot overshoot: robot must NOT drive 500 mm of wheel-spin past target
    char b[120]; std::snprintf(b, sizeof b, "done=%d got=%.1f poseX=%.1f (ground << wheel-spin)",
                               (int)done6, got, sim::poseX);
    check(ok && got > 380 && got < 560, "S6 completes on MIN-progress", b);
    check(got < 600, "S6 no overshoot from slip counts", b);
  }
  clearLog();
  sim::inject(sim::now() + 10, "S");
  runFor(300);
  check(has("flt #"), "S6 S key answers", has("flt #") ? "record printed" : "no S output");
  check(has("slip"), "S6 S names the slip", "state=slip-suppressed or similar");

  // --- S7 S key: state=none is checked inside S1 (before any fault exists);
  // here only that a printable record exists after the faults of S2..S6
  worldReset();
  clearLog();
  sim::inject(sim::now() + 10, "S");
  runFor(300);
  check(has("flt #"), "S7 S key record after faults", has("flt #") ? "record printed" : "no output");

  // --- S8 turn regression ---------------------------------------------------
  worldReset();
  sim::inject(0, "L90");
  bool t8 = waitLog("tturn", 8000);
  check(t8, "S8 L90 completes", t8 ? "tturn seen" : "NO tturn");
  {
    // turn-fix regression (30.10): the PRE-fix machine lost ~12 deg to the stop
    // kick + double-counted folds (79.5 deg; the old 75-80 band banded that
    // artifact). post-fix: lead stop + honest residual counting -> within 5 deg
    // of target, spin symmetry residual |NL+NR| <= 2 ticks (task acceptance).
    size_t p = sim::txLog.find("got=");
    float deg = p == std::string::npos ? 0 : (float)std::atof(sim::txLog.c_str() + p + 4);
    long lCnt = 0, rCnt = 0;
    size_t pl = sim::txLog.find("L="), pr = sim::txLog.find("R=");
    if (pl != std::string::npos) lCnt = std::atol(sim::txLog.c_str() + pl + 2);
    if (pr != std::string::npos) rCnt = std::atol(sim::txLog.c_str() + pr + 2);
    char b[120]; std::snprintf(b, sizeof b, "got=%.1f deg L=%ld R=%ld |NL+NR|=%ld (target 90)",
                               deg, lCnt, rCnt, std::labs(lCnt + rCnt));
    check(deg > 85 && deg < 95, "S8 turn accuracy post-fix", b);
    check(std::labs(lCnt + rCnt) <= 2, "S8 spin symmetry |NL+NR|<=2", b);
    check(!has("warn:") && !has("err:"), "S8 no fault output in turns", "clean");
  }

  std::printf("\n--- sim_fault summary: fails=%d ---\n", fails);
  return fails;
}
