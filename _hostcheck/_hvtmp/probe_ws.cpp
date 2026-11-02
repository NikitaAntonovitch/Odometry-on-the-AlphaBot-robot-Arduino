
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
  std::printf("BOOTLOG:[%s]\n", sim::txLog.c_str()); sim::txLog.clear();
  sim::inject(0, "z");
  tick(5);
  std::printf("AFTER-Z:[%s]\n", sim::txLog.c_str()); sim::txLog.clear();
  sim::inject(0, "F300");
  runFor(400);
  std::printf("AFTER-400ms simMs=%lu txlog=[%s]\n", sim::now(), sim::txLog.c_str()); sim::txLog.clear();
  std::printf("sim ticks L=%ld R=%ld vL=%.1f vR=%.1f poseX=%.1f\n",
      sim::wl.ticks, sim::wr.ticks, sim::wl.v_mm_s, sim::wr.v_mm_s, sim::poseX);
  return 0;
}
