// diag7b: full pin trace around the fault window
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
  for (unsigned t = 0; t <= 1200; t += 10) {
    tick(10);
    if (t % 50 == 0)
      std::printf("t=%4d poseX=%6.1f encL=%3ld encR=%3ld pwm5=%3d pwm6=%3d d14=%d d15=%d d16=%d d17=%d d5=%d d6=%d\n",
        t, sim::poseX, sim::wl.ticks, sim::wr.ticks,
        sim::pinPwm(5), sim::pinPwm(6),
        sim::pinDig[14], sim::pinDig[15], sim::pinDig[16], sim::pinDig[17],
        sim::pinDig[5], sim::pinDig[6]);
  }
  return 0;
}
