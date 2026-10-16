// sim_stage3: runs the UNCHANGED stage-3 sketch against the virtual alphabot
// build: PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h \
//          -x c++ lab1_stage3_straight/lab1_stage3_straight.ino _hostcheck/sim_stage3.cpp -o _hostcheck/sim3.exe
// run:   _hostcheck/sim3.exe
#include "sim_robot.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>

SerialMock Serial;            // the sketch's Serial
void setup();                 // provided by the .ino (compiled as C++)
void loop();

// one control step: advance clock, run sketch bookkeeping, deliver due serial, integrate physics
static void tick(unsigned dtMs) {
  sim::setTime(sim::now() + dtMs);
  loop();                                 // control period + parser flushes live here
  while (Serial.available() > 0) loop();  // drain all due input
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

// send keys, let the 50 ms parser flush window pass, return everything printed meanwhile
static std::string sendAndWait(const char* keys, unsigned settleMs = 120) {
  sim::inject(0, keys);
  runFor(settleMs);
  std::string ans = sim::txLog;
  sim::txLog.clear();
  return ans;
}

// run until a "done" line appears (or timeout); returns the log, clears it
static std::string runUntilDone(unsigned timeoutMs) {
  std::string all;
  for (unsigned t = 0; t < timeoutMs; t += 25) {
    tick(25);
    if (sim::txLog.find("done") != std::string::npos) break;
  }
  all = sim::txLog;
  sim::txLog.clear();
  return all;
}

static bool hasDone(const std::string& s) { return s.find("done") != std::string::npos; }
static bool hasErr(const std::string& s)  { return s.find("err:") != std::string::npos; }

static double headingDeg() { return sim::heading * 180.0 / 3.14159265358979; }

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  // ===== 1. forward drive: completes, result line, odometry and pose agree =====
  sim::simOut("== F200 basic drive ==");
  sim::resetRobot();
  std::string ans = sendAndWait("F200");
  check(ans.find("go dir=fwd") != std::string::npos, "F200: go line with dir=fwd");
  ans += runUntilDone(8000);
  {
    size_t p = ans.find("got=");
    float got = p == std::string::npos ? -9999 : strtof(ans.c_str() + p + 4, nullptr);
    char name[128];
    snprintf(name, sizeof name, "F200: done line, odometry got=%.1f (want 200 +-10)", got);
    check(hasDone(ans) && got > 190 && got < 210, name);
    snprintf(name, sizeof name, "F200: true pose x=%.1f within 15 mm of target", sim::poseX);
    check(sim::poseX > 185 && sim::poseX < 215, name);
    check(std::count(ans.begin(), ans.end(), '\n') == 2, "F200: exactly go+done lines, nothing else");
    snprintf(name, sizeof name, "F200: straight - lateral y=%.1f mm, heading=%.2f deg", sim::poseY, headingDeg());
    check(std::abs(sim::poseY) < 8 && std::abs(headingDeg()) < 5, name);
  }

  // ===== 2. backward drive =====
  // NOTE: sendAndWait and runUntilDone are separate statements on purpose - with
  // `sendAndWait(x) + runUntilDone(y)` GCC may evaluate runUntilDone FIRST (order of
  // the two call results in + is unspecified), which delivers the command only after
  // the polling window has already timed out
  sim::simOut("== B150 backward ==");
  sim::resetRobot();
  sendAndWait("B150", 60);
  ans = runUntilDone(8000);
  {
    size_t p = ans.find("got=");
    float got = p == std::string::npos ? 9999 : strtof(ans.c_str() + p + 4, nullptr);
    char name[128];
    snprintf(name, sizeof name, "B150: done, got=%.1f (want -150 +-10), pose x=%.1f", got, sim::poseX);
    check(hasDone(ans) && got < -140 && got > -160 && sim::poseX < -135 && sim::poseX > -165, name);
  }

  // ===== 3. negative number after B: direction from the letter, abs of the number =====
  sim::simOut("== B-200 sign rule ==");
  sim::resetRobot();
  sendAndWait("B-200", 60);
  ans = runUntilDone(8000);
  check(hasDone(ans) && sim::poseX < -185, "B-200: drives backward 200 (letter carries direction)");

  // ===== 4. symmetry gain A/B/C: heading drift with K=0 vs 0.6 vs 3 =====
  sim::simOut("== gain tuning A/B ==");
  double hGain[3];
  const char* gcmd[3] = { "K0", "K0.6", "K3" };
  for (int g = 0; g < 3; g++) {
    sim::resetRobot();
    ans = sendAndWait(gcmd[g]);
    check(ans.find("gain=") != std::string::npos, "K command acknowledged");
    sendAndWait("F300", 60);
    runUntilDone(10000);
    hGain[g] = headingDeg();
    char name[96];
    snprintf(name, sizeof name, "K%s: heading after F300 = %.2f deg", gcmd[g] + 1, hGain[g]);
    sim::simOut(name);
  }
  check(std::abs(hGain[0]) > std::abs(hGain[1]) * 1.5, "K0 drifts clearly more than K0.6 (controller works)");
  check(std::abs(hGain[1]) > std::abs(hGain[2]), "K3 better than K0.6 (more gain, less drift)");
  check(std::abs(hGain[2]) < 2.0, "K3 holds the line under 2 deg over 300 mm");

  // ===== 5. consecutive drives without zeroing: start snapshot works =====
  sim::simOut("== consecutive drives ==");
  sim::resetRobot();
  sendAndWait("F120", 60); runUntilDone(8000);
  sendAndWait("F80", 60);  ans = runUntilDone(8000);
  check(hasDone(ans) && sim::poseX > 180 && sim::poseX < 220, "F120 then F80 -> ~200 mm total, no z needed");

  // ===== 6. short drive fully inside the ramps =====
  sim::simOut("== short drive ==");
  sim::resetRobot();
  sendAndWait("F30", 60);
  ans = runUntilDone(8000);
  check(hasDone(ans) && sim::poseX > 18 && sim::poseX < 42, "F30 completes (profile handles sub-ramp distances)");

  // ===== 7. telemetry toggle =====
  sim::simOut("== telemetry ==");
  sim::resetRobot();
  sendAndWait("t");                          // tel on
  sendAndWait("F150", 60);
  runFor(700);                               // mid-drive
  std::string midLog = sim::txLog; sim::txLog.clear();
  runUntilDone(8000);
  sendAndWait("t");                          // tel off
  check(midLog.find("tel prog=") != std::string::npos, "telemetry on: tel lines streamed during the drive");
  sim::resetRobot();
  sendAndWait("F150", 60);
  runFor(700);
  midLog = sim::txLog; sim::txLog.clear();
  runUntilDone(8000);
  check(midLog.find("tel prog=") == std::string::npos, "telemetry off: no tel lines in the log");

  // ===== 8. range rules: out of range rejected, bare letter rejected =====
  sim::simOut("== range rules ==");
  sim::resetRobot();
  ans = sendAndWait("F99999");
  check(hasErr(ans) && std::abs(sim::poseX) < 1, "F99999: rejected with err line, no motion");
  ans = sendAndWait("F0");
  check(hasErr(ans) && std::abs(sim::poseX) < 1, "F0: rejected, no motion");
  ans = sendAndWait("F");
  runFor(400);
  ans = sim::txLog; sim::txLog.clear();
  check(ans.find("without a number") != std::string::npos, "bare F: rejected after the quiet window");

  // ===== 9. garbage around a command: junk skipped, drive still executes =====
  sim::simOut("== garbage tolerance ==");
  sim::resetRobot();
  sendAndWait("qQ!F300w\n", 60);
  ans = runUntilDone(10000);
  check(hasDone(ans) && sim::poseX > 285 && sim::poseX < 315, "qQ!F300w: junk ignored, F300 executed");

  // ===== 10. manual abort: c mid-drive stops without a done line =====
  sim::simOut("== manual abort ==");
  sim::resetRobot();
  sendAndWait("F2000", 60);
  runFor(900);                               // mid-drive
  ans = sendAndWait("c");
  runFor(1500);                              // coast run-on decays
  double xA = sim::poseX;
  runFor(600);
  ans += sim::txLog; sim::txLog.clear();
  check(!hasDone(ans), "abort: no done line after manual stop");
  check(std::abs(sim::poseX - xA) < 2, "abort: robot actually stopped after run-on decay");

  // ===== 11. stage 2 regression: bench hand-turn still counts raw =====
  sim::simOut("== stage2 regression ==");
  sendAndWait("xz");
  sim::handTurn(1.0, 0.0);
  ans = sendAndWait("e");
  {
    size_t p = ans.find("L=");
    long cl = p == std::string::npos ? -1 : std::atol(ans.c_str() + p + 2);
    size_t q = ans.find("R=", p);
    long cr = q == std::string::npos ? -1 : std::atol(ans.c_str() + q + 2);
    check(cl == 2 * sim::SLOTS && cr == 0, "bench: 1 hand rev left = 40 counts, right 0");
  }
  sendAndWait("x");                          // back to live mode

  // ===== 12. scale sanity: odometry got vs sim ground truth over a long run =====
  sim::simOut("== scale error separation ==");
  sim::resetRobot();
  sendAndWait("F500", 60);
  ans = runUntilDone(12000);
  {
    size_t p = ans.find("got=");
    float got = p == std::string::npos ? 0 : strtof(ans.c_str() + p + 4, nullptr);
    char name[128];
    snprintf(name, sizeof name, "F500: odometry got=%.1f vs true %.1f (controller+quantisation bound)",
             got, sim::poseX);
    // controller error = got - 500 (odometric); scale error = poseX - got (model truth vs odometry)
    check(std::abs(got - 500) <= 10 && std::abs(sim::poseX - got) <= 15, name);
  }

  std::printf("\n== stage3 scenario finished, fails=%d ==\n", fails);
  return fails ? 1 : 0;
}
