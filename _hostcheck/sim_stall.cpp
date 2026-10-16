// sim_stall: verifies the stall watchdog - battery-off emulation (dead band raised so
// the wheels can never turn) must produce "err: stalled" within ~1.5 s and stop cleanly
// build: g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h -x c++ lab1_stage3_straight/lab1_stage3_straight.ino _hostcheck/sim_stall.cpp -o _hostcheck/simstall.exe
#include "sim_robot.h"
#include <cstdio>
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

int main() {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();
  sim::DEADBAND_PWM = 250;              // battery-off emulation: duty 55..130 << 250 -> no motion
  sim::inject(0, "F200");
  runFor(120);
  bool stalled = false;
  for (unsigned t = 0; t < 4000; t += 25) {
    tick(25);
    if (sim::txLog.find("stalled") != std::string::npos) { stalled = true; break; }
  }
  std::printf("%s: stall watchdog fired (%s), poseX=%.2f\n",
              stalled ? "PASS" : "FAIL", stalled ? "in time" : "NEVER", sim::poseX);
  // after the abort a new command must still work (dead band back to normal)
  sim::DEADBAND_PWM = 35;
  sim::inject(0, "F100");
  runFor(120);
  bool done = false;
  for (unsigned t = 0; t < 8000; t += 25) {
    tick(25);
    if (sim::txLog.find("done") != std::string::npos) { done = true; break; }
  }
  std::printf("%s: recovery drive after abort (%s), poseX=%.1f\n",
              done ? "PASS" : "FAIL", done ? "completes" : "NEVER", sim::poseX);
  return (stalled && done) ? 0 : 1;
}
