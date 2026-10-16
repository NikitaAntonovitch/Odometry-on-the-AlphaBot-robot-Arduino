
#include "Arduino.h"
SerialMock Serial;
void setup();
void loop();
#include "_hostcheck/probe_inc.h"
int main(){
  setup();
  std::printf("[probe] after setup: ENA pwm=%d IN1=%d IN2=%d\n", sim::pinPwm(5), sim::pinLevel(14), sim::pinLevel(15));
  sim::inject(0, "1");
  sim::setTime(10);
  for (int i=0;i<3;i++) loop();
  std::printf("[probe] after '1': ENA pwm=%d IN1=%d IN2=%d tx=%s\n", sim::pinPwm(5), sim::pinLevel(14), sim::pinLevel(15), sim::txLog.c_str());
  sim::stepPhysics(100);
  std::printf("[probe] after 100ms: pose x=%.1f tickL=%ld pwm5=%d dig5=%d\n", sim::poseX, sim::wl.ticks, sim::pinPwm(5), sim::pinDig[5]);
  return 0;
}
