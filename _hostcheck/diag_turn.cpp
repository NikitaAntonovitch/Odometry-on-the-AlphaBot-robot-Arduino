// diag_turn v5: arbitrary command line from argv, verbatim output + ground truth
// usage: diagturn "L90" | diagturn "G0.8 G1.2 G0.6" | diagturn "G0.8 L90 G1.2 R45 G0.6"
#include "Arduino.h"
#include <cstdio>
#include <cstring>
#include <string>

SerialMock Serial;
void setup();
void loop();

static void tick1() {
  sim::setTime(sim::now() + 1);
  loop();
  while (Serial.available() > 0) loop();
  sim::stepPhysics(1);
}

int main(int argc, char** argv) {
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();
  sim::MAX_MM_S = 592.0;
  std::string what = argc > 1 ? argv[1] : "L90";
  sim::inject(0, what.c_str());
  for (unsigned t = 0; t < 30000; t += 1) tick1();   // full budget: every command settles
  tick1(); tick1();
  std::printf("%s", sim::txLog.c_str());
  sim::dumpPose();
  return 0;
}
