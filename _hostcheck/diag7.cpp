// diag7: one-off probe for the stage-7 drive stall. traces enc pins + pwm every 200ms.
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
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();
  sim::inject(0, "G0.5");
  for (unsigned t = 0; t <= 2000; t += 20) {
    tick(20);
    if (t % 200 == 0)
      std::printf("t=%4d poseX=%7.1f encL=%5ld encR=%5ld pwm5=%3d pwm6=%3d d14=%d d15=%d d16=%d d17=%d\n",
        t, sim::poseX, sim::wl.ticks, sim::wr.ticks,
        sim::pinPwm(5), sim::pinPwm(6),
        sim::pinDig[14], sim::pinDig[15], sim::pinDig[16], sim::pinDig[17]);
  }
  std::printf("---- txLog ----\n%s\n", sim::txLog.c_str());
  return 0;
}
