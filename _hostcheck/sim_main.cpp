// sim_main: runs the UNCHANGED stage-1 sketch against the virtual alphabot
// compiles: g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h -x c++ lab1_stage1_motor_test/lab1_stage1_motor_test.ino _hostcheck/sim_main.cpp -o _hostcheck/sim.exe
// run:      _hostcheck/sim.exe
#include "sim_robot.h"
#include <cstdio>
#include <cmath>

SerialMock Serial;            // the sketch's Serial
void setup();                 // provided by the .ino (compiled as C++)
void loop();

// one control step: deliver due serial input, advance the clock, integrate physics
static void tick(unsigned dtMs) {
  sim::setTime(sim::now() + dtMs);
  if (Serial.available() > 0) loop();     // the sketch handles what arrived (chars due at t <= now)
  sim::stepPhysics(dtMs);
}

// run the sketch for a while, delivering scheduled input as it becomes due
static void runFor(unsigned ms) {
  tick(0);                                // t=0 input lands immediately
  for (unsigned t = 0; t < ms; t += 5) tick(5);
}

static int fails = 0;
static void check(bool ok, const char* what) {
  sim::simOut(std::string(ok ? "PASS: " : "FAIL: ") + what);
  if (!ok) fails++;
}

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();   // bring-up menu

  // ===== 1. direction/mode check: each mode answers exactly one line and moves the robot =====
  sim::simOut("== direction check ==");
  struct Step { char key; const char* name; double expDX; double expDHead; };
  Step seq[] = {
    {'1', "fwd",    +1, 0}, {'c', 0, 0, 0},
    {'2', "back",   -1, 0}, {'c', 0, 0, 0},
    {'3', "spinL",   0, -1},{'c', 0, 0, 0},
    {'4', "spinR",   0, +1},{'c', 0, 0, 0},
    {'5', "pivotL",  0, -1},{'c', 0, 0, 0},
    {'6', "pivotR",  0, +1},{'c', 0, 0, 0},
  };
  for (auto& st : seq) {
    if (!st.name) continue;                    // the 'c' entries just coast between modes
    sim::resetRobot();
    sim::inject(0, std::string(1, st.key).c_str());
    tick(5);                                   // sketch consumes the char, applies the mode
    std::string ans = sim::txLog; sim::txLog.clear();
    bool lineOk = ans.find(st.name) != std::string::npos && std::count(ans.begin(), ans.end(), '\n') == 1;
    double x0 = sim::poseX, y0 = sim::poseY, h0 = sim::heading;
    runFor(1000);                              // 1s of motion
    double dx = sim::poseX - x0, dh = sim::heading - h0;
    double dxSigned = dx * (sim::SIGN_FLIP ? -1.0 : 1.0);   // bridge command -> chassis direction
    bool moved = st.expDX ? (dxSigned * st.expDX > 5.0) : std::abs(dh * st.expDHead) > 0.05;
    // secondary-motion criteria per mode: fwd/back must hold heading and stay on the line,
    // spins must keep the chassis centre in place, pivots must keep the pivot wheel planted
    bool clean;
    switch (st.key) {
      case '1': case '2': clean = std::abs(sim::poseY - y0) < 2 && std::abs(dh) < 0.05; break;
      case '3': case '4': clean = std::abs(sim::poseX - x0) < 3 && std::abs(sim::poseY - y0) < 3; break;
      case '5': clean = sim::wl.x_mm < 1.0; break;   // left wheel planted for pivotL
      case '6': clean = sim::wr.x_mm < 1.0; break;   // right wheel planted for pivotR
      default:  clean = true;
    }
    check(lineOk, (std::string("mode ") + st.name + ": answered exactly one line").c_str());
    check(moved,   (std::string("mode ") + st.name + ": robot moves as expected").c_str());
    check(clean,   (std::string("mode ") + st.name + ": no unintended secondary motion").c_str());
    sim::inject(0, "c"); tick(5); sim::txLog.clear();
  }

  // ===== 2. dead band search: pwm 90 -> 30 while driving, x must stop growing below 35 =====
  sim::simOut("== dead band search ==");
  sim::resetRobot();
  sim::inject(0, "1");
  tick(5); sim::txLog.clear();
  runFor(1000);                                // cruise at pwm 90
  double x0 = sim::poseX;
  for (int i = 0; i < 2; i++) {                // two '-' presses: 90 -> 80, still above the
    sim::inject(0, "-");                       // floor-measured dead band (75)
    tick(5); sim::txLog.clear();
    runFor(80);
  }
  double x1 = sim::poseX;
  check(std::abs(x1 - x0) > 10, "robot still moved at pwm 80 (|dx|: chassis dir under SIGN_FLIP)");
  for (int i = 0; i < 2; i++) {                // two more: 80 -> 70, BELOW the dead band now
    sim::inject(0, "-");
    tick(5); sim::txLog.clear();
    runFor(80);
  }
  runFor(1500);                                // pwm 70 < 75: the virtual wheels cannot turn
  double x2 = sim::poseX;
  runFor(800);                                 // decay window finished; distance must be frozen now
  check(std::abs(sim::poseX - x2) < 0.5, "robot stops below the dead band (pwm 70)");
  // restore cruise pwm for the run-on test: 70 -> 90 needs 4 '+' presses
  for (int i = 0; i < 4; i++) { sim::inject(0, "+"); tick(5); sim::txLog.clear(); }
  runFor(300);
  sim::inject(0, "c"); tick(5); sim::txLog.clear();

  // ===== 3. run-on: coast vs brake from the same cruise =====
  sim::simOut("== run-on comparison ==");
  double runon[2];
  const char* stopKey[2] = {"c", "b"};
  const char* stopName[2] = {"coast", "brake"};
  for (int v = 0; v < 2; v++) {
    sim::resetRobot();
    sim::inject(0, "1"); tick(5); sim::txLog.clear();
    runFor(2000);                              // steady cruise at pwm 90 (~100 mm/s)
    double xs = sim::poseX;
    sim::inject(0, stopKey[v]); tick(5); sim::txLog.clear();
    runFor(4000);                              // run-on window
    runon[v] = std::abs(sim::poseX - xs);   // magnitude: chassis direction is flipped (SIGN_FLIP)
    char buf[80]; snprintf(buf, sizeof buf, "%s run-on distance: %.0f mm", stopName[v], runon[v]);
    sim::simOut(buf);
  }
  check(runon[0] > runon[1] * 2.0, "coast run-on clearly larger than brake run-on");

  // ===== 4. garbage input must not move the robot =====
  sim::simOut("== garbage input test ==");
  sim::resetRobot();
  sim::inject(0, "x!@#$%^&*()\nZZQq9");
  runFor(1000);
  check(std::abs(sim::poseX) < 1 && std::abs(sim::heading) < 0.01, "garbage causes no motion");

  std::printf("\n== stage1 scenario finished, fails=%d ==\n", fails);
  return fails ? 1 : 0;
}
