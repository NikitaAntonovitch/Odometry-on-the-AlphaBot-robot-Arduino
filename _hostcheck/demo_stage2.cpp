// demo_stage2: scripted operator session against the UNCHANGED stage-2 sketch.
// prints exactly what the Arduino serial monitor would show (sketch lines verbatim),
// with [host] annotations explaining the step - for when you have no robot at home.
// build: g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h -x c++ \
//          lab1_stage2_encoders/lab1_stage2_encoders.ino _hostcheck/demo_stage2.cpp -o _hostcheck/demo2.exe
#include "sim_robot.h"
#include <cstdio>
#include <cstring>
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

static void keys(const char* s) {           // type keys, show the sketch's answer lines
  sim::inject(0, s);
  tick(5);
  std::printf("%s", sim::txLog.c_str());
  sim::txLog.clear();
}

static void say(const char* s) { std::printf("\n[host] %s\n", s); }

int main() {
  printf("======== virtual serial monitor - lab1 stage2 ========\n");
  setup();
  std::printf("%s", sim::txLog.c_str()); sim::txLog.clear();

  say("step 1: bench mode ON ('x') and counters to zero ('z')");
  keys("xz");

  say("step 2: hand-rotate the LEFT wheel exactly 3 revolutions forward");
  sim::handTurn(3.0, 0.0);

  say("step 3: read the encoders ('e') - expect L=120 counts (3 rev x 2S), R=0");
  keys("e");

  say("step 4: hand-rotate the RIGHT wheel 2 revs backward - expect R=-80 (sign is last command context)");
  sim::handTurn(0.0, -2.0);
  keys("e");

  say("step 5: counts-per-revolution check - zero ('z'), turn BOTH wheels 10 revs");
  keys("z");
  sim::handTurn(10.0, 10.0);
  keys("e");

  say("step 6: back to LIVE mode ('x'), zero, drive forward at pwm 90 for 2 s");
  keys("xz");
  keys("1");
  runFor(2000);

  say("step 7: read encoders while driving - counts grow, mm should match the run");
  keys("e");

  say("step 8: coast ('c') - read DURING the coast window: run-on still counts");
  keys("c");
  runFor(300);
  keys("e");

  say("step 9: wait out the coast window (2000 ms), read again - counter frozen");
  runFor(sim::COAST_MS + 200);
  keys("e");

  say("step 10: 500 ms later - still frozen, no drift");
  runFor(500);
  keys("e");

  say("step 11: window speed ('s') while cruising - sketch value vs sim truth");
  keys("1");
  runFor(2500);
  keys("s");
  std::printf("[host] sim truth speed: L=%.1f mm/s R=%.1f mm/s\n", sim::wl.v_mm_s, sim::wr.v_mm_s);

  say("step 12: standstill vibration test - coast, freeze, then wiggle the disc 300x");
  keys("c");
  runFor(sim::COAST_MS + 200);
  keys("e");
  sim::jitterWheel(300);
  keys("e");

  say("step 13: same wiggle in BENCH mode ('x' on) - raw edges DO land (freeze was the guard)");
  keys("x");
  sim::jitterWheel(300);
  keys("e");
  keys("x");

  say("step 14: where is the robot? (sim truth, not visible on the real UNO)");
  sim::dumpPose();

  printf("\n======== demo finished ========\n");
  return 0;
}
