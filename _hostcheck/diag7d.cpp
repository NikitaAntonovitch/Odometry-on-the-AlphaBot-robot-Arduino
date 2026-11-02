// diag7d: suite-identical S6 (worldReset + runFor + waitLog) with slip-state prints
#include "Arduino.h"
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
static bool waitLog(const char* needle, unsigned timeoutMs) {
  for (unsigned t = 0; t < timeoutMs; t += 10) {
    tick(10);
    if (sim::txLog.find(needle) != std::string::npos) return true;
  }
  return false;
}
static void clearLog() { sim::txLog.clear(); }
int main() {
  setup();
  clearLog();
  // suite S1 first (identical), then S6's worldReset path
  sim::inject(0, "G0.5");
  waitLog("done", 6000);
  std::printf("S1 done. SLIP_L=%.2f GAIN=%.2f DBL=%d poseX=%.1f\n",
              sim::SLIP_L, sim::SLIP_FREE_GAIN, sim::DEADBAND_L, sim::poseX);
  sim::resetRobot(); clearLog();
  sim::SLIP_L = 0.0; sim::SLIP_R = 0.0;
  sim::inject(0, "G0.5");
  runFor(400);
  sim::SLIP_L = 1.0;
  std::printf("slip set: SLIP_L=%.2f GAIN=%.2f DBL=%d DB=%d\n",
              sim::SLIP_L, sim::SLIP_FREE_GAIN, sim::DEADBAND_L, sim::DEADBAND_PWM);
  bool w = waitLog("warn: slip L wheel", 3000);
  std::printf("rate warn=%d  cL=%ld cR=%ld\n", (int)w, sim::wl.ticks, sim::wr.ticks);
  std::printf("tx: %s\n", sim::txLog.c_str());
  clearLog();
  bool d = waitLog("done", 8000);
  std::printf("done=%d cL=%ld cR=%ld poseX=%.1f\n  tx: %s\n", (int)d, sim::wl.ticks, sim::wr.ticks, sim::poseX, sim::txLog.c_str());
  return 0;
}
