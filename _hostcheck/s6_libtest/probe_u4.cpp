// Probe 4: raw u comparison with kp=0.005 (e~130 -> u<1) and step changes so the
// error stays inside the unsaturated band on both sketches.
#include <string>
#include <cstdio>
#include <vector>

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

static void lineAtP(double p, int bg) {
  int r[5];
  for (int i = 0; i < 5; i++) {
    double d = p - i; if (d < 0) d = -d;
    double t = d < 1.0 ? (1.0 - d) : 0.0;
    r[i] = (int)(bg - (double)(bg - 200) * t);
  }
  setRaw(r[0], r[1], r[2], r[3], r[4]);
}
static std::vector<std::string> splitLines(const std::string& s) {
  std::vector<std::string> out; std::string cur;
  for (char c : s) { if (c == '\n') { out.push_back(cur); cur.clear(); } else if (c != '\r') cur += c; }
  if (!cur.empty()) out.push_back(cur);
  return out;
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
  sendLine("M"); Serial.tx.clear();
  setRaw(800,800,200,800,800);
  for (int i = 0; i < 3; i++) { g_ms += 20; loop(); }
  sendLine("s"); Serial.tx.clear();
  g_serialAutoAdvanceMs = 20;
  sendLine("g");
  g_serialAutoAdvanceMs = 0;
  sendLine("k0.005");       // tiny kp: e=130 -> u=0.65 -> raw u in +-1..2 range
  sendLine("R");
  // stay near-centred with the noisy background: error ~ tens of counts
  for (int i = 0; i < 8; i++) { lineAtP(2.0, 790); g_ms += 20; loop(); }
  for (int i = 0; i < 8; i++) { lineAtP(1.8, 790); g_ms += 20; loop(); }
  for (int i = 0; i < 8; i++) { lineAtP(1.6, 790); g_ms += 20; loop(); }
  sendLine("R");
  Serial.tx.clear();
  sendLine("D");
  printf("=== %s dump (t,e,u,L,R,st) ===\n", tag);
  for (const std::string& L : splitLines(Serial.tx))
    if (!L.empty() && (L[0] >= '0' && L[0] <= '9')) printf("  %s\n", L.c_str());
}

int main(int argc, char** argv) {
  run(argv[1] ? argv[1] : "SKETCH");
  return 0;
}
