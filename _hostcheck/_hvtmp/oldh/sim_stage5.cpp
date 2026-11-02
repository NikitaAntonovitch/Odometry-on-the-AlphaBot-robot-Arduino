// sim_stage5: runs the UNCHANGED stage-5 sketch against the virtual alphabot
// build: PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h \
//          -x c++ lab1_stage5_protocol/lab1_stage5_protocol.ino _hostcheck/sim_stage5.cpp -o _hostcheck/sim5.exe
// run:   _hostcheck/sim5.exe
// covers checkpoint 5: sequence as a single string, skip rule, sign/unit rules,
// out-of-range behaviour, one result line per command (turn carries the angle)
#include "sim_robot.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>

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
  sim::simOut(std::string(ok ? "PASS: " : "FAIL: ") + what);
  if (!ok) fails++;
}
static std::string sendAndWait(const char* keys, unsigned settleMs = 120) {
  sim::inject(0, keys);
  runFor(settleMs);
  std::string ans = sim::txLog;
  sim::txLog.clear();
  return ans;
}
// motion to completion (done/tturn/err line) or timeout; returns + clears the log
static std::string runUntilAny(unsigned timeoutMs) {
  for (unsigned t = 0; t < timeoutMs; t += 25) {
    tick(25);
    if (sim::txLog.find("done cmd=") != std::string::npos) break;
    if (sim::txLog.find("tturn cmd=") != std::string::npos) break;
    if (sim::txLog.find("err:") != std::string::npos) break;
  }
  std::string all = sim::txLog;
  sim::txLog.clear();
  return all;
}
// full single-command cycle: send, wait for the ack, wait for the result
static std::string runCmd(const char* keys, unsigned timeoutMs = 15000) {
  std::string ans = sendAndWait(keys, 60);
  ans += runUntilAny(timeoutMs);
  return ans;
}
static bool hasDone(const std::string& s)  { return s.find("done cmd=G") != std::string::npos; }
static bool hasTTurn(const std::string& s) { return s.find("tturn cmd=") != std::string::npos; }
static bool hasErr(const std::string& s)   { return s.find("err:") != std::string::npos; }
static float gotOf(const std::string& s) {           // first got= value on the line
  size_t p = s.find("got=");
  return p == std::string::npos ? -99999.0f : strtof(s.c_str() + p + 4, nullptr);
}
static int countTurns(const std::string& s) {
  int n = 0; size_t p = 0;
  while ((p = s.find("tturn cmd=", p)) != std::string::npos) { n++; p += 10; }
  return n;
}
static int countDone(const std::string& s) {
  int n = 0; size_t p = 0;
  while ((p = s.find("done cmd=G", p)) != std::string::npos) { n++; p += 10; }
  return n;
}
static double headingDeg() { return sim::heading * 180.0 / 3.14159265358979; }

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  // ===== 1. single G drive: ack + done line, odometry and pose agree =====
  sim::simOut("== G0.2 basic drive ==");
  sim::resetRobot();
  {
    std::string ans = runCmd("G0.2");
    float got = gotOf(ans);
    char name[128];
    snprintf(name, sizeof name, "G0.2: go+done, got=%.1f mm (want 200 +-10)", got);
    check(hasDone(ans) && got > 190 && got < 210, name);
    snprintf(name, sizeof name, "G0.2: pose x=%.1f (want ~200), y=%.1f straight", sim::poseX, sim::poseY);
    check(sim::poseX > 185 && sim::poseX < 215 && std::abs(sim::poseY) < 10, name);
    check(ans.find("go cmd=G0.200m dir=fwd") != std::string::npos, "G0.2: go ack echoes metres + direction");
  }

  // ===== 2. negative G: backwards =====
  sim::simOut("== G-0.2 backwards ==");
  sim::resetRobot();
  {
    std::string ans = runCmd("G-0.2");
    float got = gotOf(ans);
    check(hasDone(ans) && got < -190 && got > -210, "G-0.2: signed metres, negative drives BACK (got<0)");
    check(sim::poseX < -185 && sim::poseX > -215, "G-0.2: pose x negative (moved backwards)");
  }

  // ===== 3. L/R turns with the angle in the result line =====
  sim::simOut("== L90 / R45 ==");
  sim::resetRobot();
  {
    std::string ans = runCmd("L90");
    float got = gotOf(ans);
    check(hasTTurn(ans) && got > 85 && got < 95, "L90: tturn with got~90 (angle in result line)");
    check(headingDeg() > 82 && headingDeg() < 98, "L90: heading turned ccw");
  }
  sim::resetRobot();
  runFor(600);
  {
    std::string ans = runCmd("R45");
    float got = gotOf(ans);
    check(hasTTurn(ans) && got < -41 && got > -49, "R45: tturn got~-45 (cw)");
  }

  // ===== 4. THE stage-5 core: whole sequence as ONE string =====
  sim::simOut("== sequence G0.2 L90 G0.3 in one string ==");
  sim::resetRobot();
  {
    // send everything at once; each motion runs to completion before the next parses
    sim::inject(0, "G0.2 L90 G0.3");
    std::string ans;
    for (int i = 0; i < 3; i++) ans += runUntilAny(20000);
    char name[160];
    snprintf(name, sizeof name, "sequence: done/tturn/done = %d/%d/%d (3 commands executed back to back)",
             countDone(ans), countTurns(ans), countDone(ans) - (countDone(ans) ? 1 : 0));
    check(countDone(ans) == 2 && countTurns(ans) == 1, name);
    float got = gotOf(ans.substr(ans.find("tturn cmd=")));
    snprintf(name, sizeof name, "sequence: turn got=%.1f (inside the string, mid-route)", got);
    check(got > 85 && got < 95, name);
    // end state: 200 fwd, 90 ccw, 300 fwd -> pose ~ (200+300, 200) rotated frame
    double hx = sim::poseX, hy = sim::poseY;
    snprintf(name, sizeof name, "sequence: end pose (%.0f, %.0f) - drove the L-shaped route", hx, hy);
    // model truth: 0.2 fwd then L90 then 0.3 fwd -> x ~ 200, y ~ 300 (ccw left turn)
    check(hx > 170 && hx < 230 && hy > 270 && hy < 330, name);
  }

  // ===== 5. skip rule: garbage interleaved with real commands =====
  sim::simOut("== unrecognised characters skipped ==");
  sim::resetRobot();
  {
    std::string ans = runCmd("Xyz ,.! Q7  G0.1\n");
    check(hasDone(ans) && countTurns(ans) == 0,
          "garbage+G0.1: only the G executed, junk skipped with no side effect");
    check(!hasErr(ans), "garbage: no err lines for skipped characters");
  }
  sim::resetRobot();
  {
    // lowercase protocol letters: case-insensitive by design (documented).
    // full-wait variant: runUntilAny returns early on the go-line 'err:'? no - on
    // done/err only; but the FIRST command's done can arrive while the SECOND turn
    // is still spinning, so wait the sequence out with a generous fixed tail
    std::string ans = runCmd("g0.1 l45", 6000);
    runFor(1500);
    ans += sim::txLog; sim::txLog.clear();
    check(hasDone(ans) && countTurns(ans) == 1, "lowercase g/l: same commands (case-insensitive)");
  }

  // ===== 6. sign rules =====
  sim::simOut("== sign rules ==");
  sim::resetRobot();
  {
    std::string ans = runCmd("L-45");                 // negative after L: abs, letter decides
    check(headingDeg() > 41 && headingDeg() < 49, "L-45: turns LEFT 45 (abs rule, letter carries direction)");
  }
  sim::resetRobot();
  runFor(600);
  {
    // fixed-wait variant: got= drifts ~-47 (track 151 vs model 160 is the calibration
    // delta - the real gate for got= accuracy is the stage-4 1440 confirmation, not here)
    std::string ans = runCmd("R-45", 6000);
    check(headingDeg() < -41 && headingDeg() > -49, "R-45: turns RIGHT 45 (mirrored, true heading)");
  }

  // ===== 7. out-of-range: one err line, NO motion, string continues =====
  sim::simOut("== out-of-range handling ==");
  sim::resetRobot();
  {
    // out-of-range + valid turn in one string: err must not kill the rest
    sim::inject(0, "G99 L90");
    std::string all;
    for (int i = 0; i < 2; i++) {           // wait out BOTH the err line and the tturn
      all += runUntilAny(15000);
    }
    check(hasErr(all), "G99: out of range -> err line");
    check(countTurns(all) == 1, "G99 L90: the valid command AFTER a rejected one still executes");
    check(countDone(all) == 0, "G99: no drive motion happened for the rejected command");
  }
  sim::resetRobot();
  {
    std::string ans = runCmd("L3", 3000) + (runFor(800), sim::txLog); sim::txLog.clear();
    check(hasErr(ans) && countTurns(ans) == 0, "L3: below 5 deg -> err, no motion (log waited out)");
    ans = runCmd("L2000", 3000) + (runFor(800), sim::txLog); sim::txLog.clear();
    check(hasErr(ans) && countTurns(ans) == 0, "L2000: above 1440 deg -> err, no motion");
    ans = runCmd("G0.001", 3000) + (runFor(800), sim::txLog); sim::txLog.clear();
    check(hasErr(ans) && countDone(ans) == 0, "G0.001: below 5 mm -> err, no motion");
  }

  // ===== 8. busy-window rule: chars sent mid-motion wait, abort key works =====
  sim::simOut("== busy window: rx buffer holds the next command, c aborts ==");
  sim::resetRobot();
  {
    sim::inject(0, "G0.9");                 // long drive (~3 s at cruise)
    runUntilAny(999999);                    // wait for the go ack only... runUntilAny waits
                                            // for done/err too - instead: short wait, then
                                            // the next inject lands MID-DRIVE
    // simpler deterministic version: start a fresh drive, inject mid-flight, then abort
    sim::resetRobot();
    sendAndWait("G0.9", 60);
    runFor(700);                            // mid-drive now
    sim::inject(0, "L90");                  // must WAIT in the rx buffer (busy)
    runFor(300);
    check(sim::txLog.find("tturn") == std::string::npos, "mid-drive L90: no turn starts while busy (waits in rx buffer)");
    check(std::abs(sim::wl.v_mm_s) + std::abs(sim::wr.v_mm_s) > 50, "drive still running while L90 waits");
    std::string ans = sendAndWait("c");     // abort mid-motion
    runFor(1500);                           // coast decay tau = 0.35 s: needs >1 s to fall <5
    check(ans.find("mode=stop") != std::string::npos, "c: abort line served mid-motion");
    check(std::abs(sim::wl.v_mm_s) + std::abs(sim::wr.v_mm_s) < 5, "c: robot actually stopped");
    check(countTurns(sim::txLog) == 0, "aborted drive: no phantom turn executed");
    sim::txLog.clear();
    // c also FLUSHED the queue (stop means stop, not pause) - the deferred L90 is gone
    runFor(2000);
    check(countTurns(sim::txLog) == 0, "abort flushed the queue: L90 dropped (stop, not pause)");
  }

  // ===== 9. diag keys still alive: K gain + m mode + p state =====
  sim::simOut("== diag keys ==");
  sim::resetRobot();
  {
    std::string ans = sendAndWait("K1.5");
    check(ans.find("gain=1.50") != std::string::npos, "K1.5: gain setter answers");
    ans = sendAndWait("m");
    check(ans.find("pivotL") != std::string::npos, "m: mode cycle answers");
    ans = sendAndWait("p");
    check(ans.find("idle") != std::string::npos, "p: state answers idle");
    sendAndWait("m");                       // back to spin
  }

  // ===== 10. turn-in-sequence after turn: state machine resets between motions =====
  sim::simOut("== two turns back to back ==");
  sim::resetRobot();
  {
    sim::inject(0, "L90 R90");
    std::string ans = runUntilAny(20000);
    ans += runUntilAny(20000);
    check(countTurns(ans) == 2, "L90 R90: both turns executed from one string");
    snprintf(0, 0, "%s", ""); // no-op
    double h = headingDeg();
    char name[96];
    snprintf(name, sizeof name, "L90 R90: end heading %.1f (want ~0: full net cancellation)", h);
    check(std::abs(h) < 6, name);
  }

  std::printf("\n== stage5 scenario finished, fails=%d ==\n", fails);
  return fails ? 1 : 0;
}
