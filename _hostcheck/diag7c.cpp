// diag7c: slip-rate detector probe - which warn prints, when?
#include "Arduino.h"
#include <cstdio>
SerialMock Serial;
void setup();
void loop();
static void tick(unsigned dtMs) {
  sim::setTime(sim::now() + dtMs);
  loop();
  while (Serial.available() > 0) loop();
  sim::stepPhysics(dtMs);
}
int main() {
  setup();
  sim::txLog.clear();
  sim::inject(0, "G0.5");
  for (unsigned t = 0; t <= 400; t += 5) tick(5);
  sim::SLIP_L = 1.0;
  for (unsigned t = 405; t <= 1500; t += 5) {
    tick(5);
    if (t % 100 == 0) {
      std::printf("t=%4d cL~%ld cR~%ld\n", t, sim::wl.ticks, sim::wr.ticks);
      std::printf("   tx: %s\n", sim::txLog.c_str());
      sim::txLog.clear();
    }
  }
  return 0;
}
