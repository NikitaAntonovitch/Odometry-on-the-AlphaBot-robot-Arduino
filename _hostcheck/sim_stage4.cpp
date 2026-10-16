// sim_stage4: runs the UNCHANGED stage-4 sketch against the virtual alphabot
// build: PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h \
//          -x c++ lab1_stage4_turns/lab1_stage4_turns.ino _hostcheck/sim_stage4.cpp -o _hostcheck/sim4.exe
// run:   _hostcheck/sim4.exe
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
static std::string runUntilDone(const char* marker, unsigned timeoutMs) {
  for (unsigned t = 0; t < timeoutMs; t += 25) {
    tick(25);
    if (sim::txLog.find(marker) != std::string::npos) break;
  }
  std::string all = sim::txLog;
  sim::txLog.clear();
  return all;
}
static bool hasTTurn(const std::string& s) { return s.find("tturn cmd=") != std::string::npos; }
static bool hasErr(const std::string& s)   { return s.find("err:") != std::string::npos; }

// parse "tturn cmd=L90.0 got=89.3deg L=-156cnt R=155cnt"
static bool parseTurn(const std::string& s, float& got) {
  size_t p = s.find("got=");
  if (p == std::string::npos) return false;
  got = strtof(s.c_str() + p + 4, nullptr);
  return true;
}

static double headingDeg() { return sim::heading * 180.0 / 3.14159265358979; }

// model truth for the commanded angle, given the sketch's track value at that moment
static double trackInUse = sim::TRACK_MM;

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  // ===== 1. basic spin turn L90: heading turns LEFT (ccw, positive), wheels opposite =====
  sim::simOut("== L90 spin basic ==");
  sim::resetRobot();
  sendAndWait("L90", 60);
  std::string ans = runUntilDone("tturn", 15000);
  {
    float got = -9999;
    bool parsed = parseTurn(ans, got);
    char name[128];
    snprintf(name, sizeof name, "L90 spin: done line with got=%.1f (want ~90)", got);
    check(hasTTurn(ans) && parsed && got > 85 && got < 95, name);
    snprintf(name, sizeof name, "L90 spin: true heading %.1f deg (ccw+, near +90)", headingDeg());
    check(headingDeg() > 82 && headingDeg() < 98, name);
    bool wheelsOpposite = sim::wl.ticks < 0 && sim::wr.ticks > 0;
    check(wheelsOpposite, "L90 spin: left wheel backward, right forward (ccw spin)");
  }

  // ===== 2. R90: mirrored =====
  sim::simOut("== R90 spin ==");
  sim::resetRobot();
  sendAndWait("R90", 60);
  ans = runUntilDone("tturn", 15000);
  {
    float got = 9999;
    parseTurn(ans, got);
    char name[128];
    snprintf(name, sizeof name, "R90 spin: got=%.1f (want ~-90), heading %.1f", got, headingDeg());
    check(hasTTurn(ans) && got < -85 && got > -95 && headingDeg() < -82 && headingDeg() > -98, name);
  }

  // ===== 3. angle sweep spin: 15/30/45 in both directions =====
  sim::simOut("== angle sweep spin ==");
  struct Case { float deg; const char* cmd; };
  Case cases[] = { {15, "L15"}, {30, "L30"}, {45, "L45"}, {15, "R15"}, {45, "R45"} };
  for (auto& c : cases) {
    sim::resetRobot();
    runFor(600);                                  // settle: previous turn's wheels fully stop
                                                  // (matches the operator re-placing the robot)
    sendAndWait(c.cmd, 60);
    ans = runUntilDone("tturn", 15000);
    float got = -9999;
    parseTurn(ans, got);
    char name[128];
    // tolerance = 1 encoder count in body-degrees (spin: 2k/b) + margin - tighter than
    // that is physically impossible (guide p.23: 15 deg lies AT the resolution limit)
    double oneCntDeg = 2.0 * 5.1087 / sim::TRACK_MM * 57.29578;
    double tol = oneCntDeg + 0.6;
    snprintf(name, sizeof name, "%s spin: got=%.1f (cmd %.0f +-%.1f = 1 count)", c.cmd, got, c.deg, tol);
    bool signOk = c.cmd[0] == 'L' ? got > 0 : got < 0;
    check(hasTTurn(ans) && signOk && std::abs(got) > c.deg - tol && std::abs(got) < c.deg + tol, name);
  }

  // ===== 4. multi-revolution track calibration probe: L1440 = 4 revs =====
  // the sketch turns by odometry with track=TRACK_MM; sim track is also TRACK_MM so the
  // achieved angle must match. On the real robot the deviation vs a protractor calibrates it.
  sim::simOut("== L1440 (4 revs) ==");
  sim::resetRobot();
  sendAndWait("L1440", 60);
  ans = runUntilDone("tturn", 60000);
  {
    float got = -9999;
    parseTurn(ans, got);
    char name[128];
    snprintf(name, sizeof name, "L1440: got=%.1f deg (want ~1440, multi-rev calib probe)", got);
    check(hasTTurn(ans) && got > 1400 && got < 1480, name);
  }

  // ===== 5. pivot mode: m toggle, one wheel planted =====
  sim::simOut("== pivot turns ==");
  sim::resetRobot();
  sendAndWait("mL45", 60);
  ans = runUntilDone("tturn", 15000);
  {
    float got = -9999;
    parseTurn(ans, got);
    char name[128];
    snprintf(name, sizeof name, "L45 pivot: got=%.1f (want ~45), left wheel planted=%s",
             got, sim::wl.ticks == 0 ? "yes" : "NO");
    check(hasTTurn(ans) && got > 41 && got < 49 && sim::wl.ticks == 0, name);
    // pivot halves... no: pivot DOUBLES the moving wheel path vs spin per-wheel. right wheel
    // should have done ~2x the spin-mode per-wheel distance: 2*b*theta/2 = b*theta
    double expectCnt = sim::TRACK_MM * (45.0 * 3.14159265 / 180.0) / (3.14159265 * sim::WHEEL_D_MM / (sim::SLOTS * sim::EDGES));
    snprintf(name, sizeof name, "L45 pivot: right wheel ~%.0f counts (double the spin share)", expectCnt);
    check(std::abs((double)sim::wr.ticks) > expectCnt - 3 && std::abs((double)sim::wr.ticks) < expectCnt + 3, name);
  }
  sim::resetRobot();
  sendAndWait("m", 60);                   // pivotL -> pivotR (turn about the RIGHT wheel now)
  sendAndWait("R90", 60);
  ans = runUntilDone("tturn", 15000);
  {
    float got = -9999;
    parseTurn(ans, got);
    // run-on after brake can add a count or two to the planted wheel via vibration-free
    // residual; the planted wheel must stay within +-2 counts of zero
    char name[128];
    snprintf(name, sizeof name, "R90 pivot: got=%.1f (want ~-90), right wheel planted (|%ld|<=2)",
             got, (long)sim::wr.ticks);
    check(hasTTurn(ans) && got < -86 && got > -94 && std::labs((long)sim::wr.ticks) <= 2, name);
  }
  sendAndWait("m");                       // back to spin for the later tests

  // ===== 6. symmetry in spin: force L faster via noise? use ASYM_R - controller must equalise =====
  sim::simOut("== spin symmetry equalisation ==");
  {
    sim::resetRobot();
    sendAndWait("K3", 60);                // strong gain
    long preL = sim::wl.ticks, preR = sim::wr.ticks;   // per-turn deltas: sim ticks accumulate
    sendAndWait("L360", 60);              // one full revolution
    ans = runUntilDone("tturn", 30000);
    float got = -9999;
    parseTurn(ans, got);
    long dL = std::labs((long)sim::wl.ticks - preL), dR = std::labs((long)sim::wr.ticks - preR);
    char name[96];
    snprintf(name, sizeof name, "L360 K3: got=%.1f, |L|-|R| per-turn = %ld cnt (<=3 with equalisation)",
             got, std::labs(dL - dR));
    check(hasTTurn(ans) && std::labs(dL - dR) <= 3, name);
    sendAndWait("K0.6", 60);              // restore default-ish
  }

  // ===== 7. range rules: too small / too big / bare letter =====
  sim::simOut("== range rules ==");
  sim::resetRobot();
  ans = sendAndWait("L3");                // 3 deg < 5 minimum
  check(hasErr(ans), "L3: rejected (below 5 deg)");
  ans = sendAndWait("L2000");
  check(hasErr(ans), "L2000: rejected (above 1440 deg)");
  ans = sendAndWait("L");
  runFor(400);
  ans = sim::txLog; sim::txLog.clear();
  check(ans.find("without a number") != std::string::npos, "bare L: rejected");
  ans = sendAndWait("L-45");              // negative number: abs, direction from letter
  runUntilDone("tturn", 15000);
  check(headingDeg() > 41 && headingDeg() < 49, "L-45: turns LEFT 45 (abs rule)");

  // ===== 8. manual abort mid-turn =====
  sim::simOut("== manual abort ==");
  sim::resetRobot();
  sendAndWait("L2000" /* rejected, use real */, 30);
  sendAndWait("L720", 60);
  runFor(1200);                           // mid-turn
  ans = sendAndWait("c");
  runFor(1500);
  double h1 = headingDeg();
  runFor(600);
  check(!hasTTurn(ans), "abort: no result line after manual stop");
  check(std::abs(headingDeg() - h1) < 0.5, "abort: robot actually stopped");

  // ===== 9. stage 3 regression: F/B still work? (sketch has no F/B - document the scope) =====
  sim::simOut("== scope note ==");
  ans = sendAndWait("F100");
  check(ans.find("err:") != std::string::npos || ans.find("go dir") == std::string::npos,
        "stage4 sketch is turns-only (F/B not implemented here - stages merge in stage 6)");
  runFor(300);

  std::printf("\n== stage4 scenario finished, fails=%d ==\n", fails);
  return fails ? 1 : 0;
}
