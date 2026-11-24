// Probe: expose the unclamped PID output u difference between the two sketches
// on noisy-background inputs (dry-run trace), where the +-baseSpeed wheel clamp
// hides it in the pin-level differential.
#include <string>
#include <cstdio>

static void sendLine(const char* cmd) {
  for (const char* p = cmd; *p; p++) Serial.rx.push_back(*p);
  Serial.rx.push_back('\n');
  loop();
}
static void setRaw(int a,int b,int c,int d,int e){ g_tlc_raw[0]=a;g_tlc_raw[1]=b;g_tlc_raw[2]=c;g_tlc_raw[3]=d;g_tlc_raw[4]=e;g_tlc_raw[5]=0; }

#if defined(PRIME_OLD)
#include "prime_old.inc"
#elif defined(PRIME_NEW)
#include "prime_new.inc"
#endif

// shared prime bodies live in .inc files generated below (identical to diff_driver)
static void lineAtP(double p, int bg) {
  int r[5];
  for (int i = 0; i < 5; i++) {
    double d = p - i; if (d < 0) d = -d;
    double t = d < 1.0 ? (1.0 - d) : 0.0;
    r[i] = (int)(bg - (double)(bg - 200) * t);
  }
  setRaw(r[0], r[1], r[2], r[3], r[4]);
}

static void run(const char* tag) {
  Serial.tx.clear(); Serial.rx.clear();
  g_ms = 0; g_serialAutoAdvanceMs = 0;
  for (int i = 0; i < 32; i++) { g_pinPwm[i] = 0; g_pinLevel[i] = 0; g_pinDir[i] = 0; }
  memset(EEPROM.mem, 0, sizeof(EEPROM.mem));
  primeEEPROM();
  setup();
  Serial.tx.clear();
  setRaw(800,800,200,800,800);
  for (int i = 0; i < 3; i++) { g_ms += 20; loop(); }
  sendLine("t");            // trace on
  // dry-run start: 'g' sets dryRun=0; toggle back with 'd' AFTER g
  g_serialAutoAdvanceMs = 20;
  sendLine("g");
  g_serialAutoAdvanceMs = 0;
  sendLine("d");            // dryRun back on -> DRY prints, no driving
  Serial.tx.clear();
  // sweep the line across with a realistic noisy background (bg 790 -> residual 17)
  for (double p = 2.0; p >= 1.0; p -= 0.25) {
    lineAtP(p, 790);
    g_ms += 20; loop();
    Serial.tx.clear();
    g_ms += 20; loop();     // second tick on the same input: D settles, u = P-dominant
    // extract the last DRY u= line
    size_t pos = Serial.tx.rfind("DRY u=");
    if (pos != std::string::npos) {
      char buf[64]; size_t n = 0;
      while (pos < Serial.tx.size() && n < 63) {
        char c = Serial.tx[pos++];
        if (c == '\r' || c == '\n') break;
        buf[n++] = c;
      }
      buf[n] = 0;
      printf("%s p=%.2f %s\n", tag, p, buf);
    }
  }
  // now with a SMALL gain so the clamp never saturates: kp=0.02 -> e~130 -> u~3
  sendLine("k0.02");
  Serial.tx.clear();
  for (double p = 2.0; p >= 1.0; p -= 0.25) {
    lineAtP(p, 790);
    g_ms += 20; loop();
    Serial.tx.clear();
    g_ms += 20; loop();
    size_t pos = Serial.tx.rfind("DRY u=");
    if (pos != std::string::npos) {
      char buf[64]; size_t n = 0;
      while (pos < Serial.tx.size() && n < 63) {
        char c = Serial.tx[pos++];
        if (c == '\r' || c == '\n') break;
        buf[n++] = c;
      }
      buf[n] = 0;
      printf("%s kp0.02 p=%.2f %s\n", tag, p, buf);
    }
  }
}

int main(int argc, char** argv) {
  run(argv[1] ? argv[1] : "SKETCH");
  return 0;
}
