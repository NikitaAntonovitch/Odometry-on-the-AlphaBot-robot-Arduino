// Differential driver: feeds the SAME raw-sensor scenario to EITHER sketch
// (compile with -DPRIME_OLD for lab2_stage6_track / -DPRIME_NEW for
// lab2_stage6_trsensors) and dumps the robot-observable outputs per tick:
// ms, pwmL(5), pwmR(6), IN1..IN4 (A0..A3), LED(8).
// The two CSV files are then diffed.
#include <string>
#include <cstdio>
#include <cstdlib>

static void sendLine(const char* cmd) {
  for (const char* p = cmd; *p; p++) Serial.rx.push_back(*p);
  Serial.rx.push_back('\n');
  loop();
}
static void setRaw(int a,int b,int c,int d,int e){ g_tlc_raw[0]=a;g_tlc_raw[1]=b;g_tlc_raw[2]=c;g_tlc_raw[3]=d;g_tlc_raw[4]=e;g_tlc_raw[5]=0; }

// ---- EEPROM primers, one per sketch layout, equivalent physics ----
// dark line (raw line 200 / bg 800), signs +1/+1, kp=0.25 ki=0 kd=0.05 base=90
#if defined(PRIME_OLD)
static void primeEEPROM() {
  // v4: magic 0xA7; byte1 lineDarkFlag=1 (dark line); cmin 2..11; cmax 12..21;
  // lineLostSumMin=250 at 22 (2B LE); signs 24/25; kp 26; ki 30; kd 34; base 38; CRC 40
  uint8_t* m = EEPROM.mem;
  memset(m, 0, 64);
  m[0] = 0xA7;
  m[1] = 1;                                    // lineDarkFlag = 1 (dark line)
  for (int i = 0; i < 5; i++) { m[2+i*2] = 200; m[3+i*2] = 0; }
  for (int i = 0; i < 5; i++) { m[12+i*2] = 800 & 0xFF; m[13+i*2] = (800 >> 8) & 0xFF; }
  m[22] = 250 & 0xFF; m[23] = (250 >> 8) & 0xFF;
  m[24] = 0x01; m[25] = 0x01;                  // signs +1/+1
  float kp=0.25f, ki=0.0f, kd=0.05f; int base=90;
  memcpy(m+26, &kp, 4); memcpy(m+30, &ki, 4); memcpy(m+34, &kd, 4);
  m[38] = 90 & 0xFF; m[39] = (90 >> 8) & 0xFF;
  // CRC exactly as the OLD loader computes it on THIS host (host temporaries):
  // wl(1) + cmin(10 bytes of host-uint array) + cmax(10) + lt(2) + kp(4) + ki(4)
  // + kd(4) + base(2) + sl + sr
  unsigned int cmin[5], cmax[5];
  for (int i = 0; i < 5; i++) { cmin[i] = 200; cmax[i] = 800; }
  int lt = 250;
  uint8_t crc = 0;
  auto up = [&](uint8_t b){ crc ^= b; for (int k=0;k<8;k++) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0x07) : (uint8_t)(crc<<1); };
  up(m[1]);
  { uint8_t* p = (uint8_t*)cmin; for (int i = 0; i < 10; i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)cmax; for (int i = 0; i < 10; i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&lt;  for (int i = 0; i < 2;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&kp;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&ki;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&kd;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&base; for (int i = 0; i < 2;  i++) up(p[i]); }
  up(m[24]); up(m[25]);
  m[40] = crc;
}
#elif defined(PRIME_NEW)
static void primeEEPROM() {
  // v5: magic 0xA8; byte1 whiteLine=0 (dark line); cmin 2..11; cmax 12..21;
  // signs 22/23; kp 24; ki 28; kd 32; base 36; CRC 38
  uint8_t* m = EEPROM.mem;
  memset(m, 0, 64);
  m[0] = 0xA8;
  m[1] = 0;                                    // whiteLine = 0 (dark line)
  for (int i = 0; i < 5; i++) { m[2+i*2] = 200; m[3+i*2] = 0; }
  for (int i = 0; i < 5; i++) { m[12+i*2] = 800 & 0xFF; m[13+i*2] = (800 >> 8) & 0xFF; }
  m[22] = 0x01; m[23] = 0x01;
  float kp=0.25f, ki=0.0f, kd=0.05f; int base=90;
  memcpy(m+24, &kp, 4); memcpy(m+28, &ki, 4); memcpy(m+32, &kd, 4); memcpy(m+36, &base, 2);
  // CRC as the NEW loader computes it on this host
  unsigned int cmin[5], cmax[5];
  for (int i = 0; i < 5; i++) { cmin[i] = 200; cmax[i] = 800; }
  uint8_t crc = 0;
  auto up = [&](uint8_t b){ crc ^= b; for (int k=0;k<8;k++) crc = (crc & 0x80) ? (uint8_t)((crc<<1)^0x07) : (uint8_t)(crc<<1); };
  up(m[1]);
  { uint8_t* p = (uint8_t*)cmin; for (int i = 0; i < 10; i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)cmax; for (int i = 0; i < 10; i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&kp;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&ki;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&kd;  for (int i = 0; i < 4;  i++) up(p[i]); }
  { uint8_t* p = (uint8_t*)&base; for (int i = 0; i < 2;  i++) up(p[i]); }
  up(m[22]); up(m[23]);
  m[38] = crc;
}
#else
#error "define PRIME_OLD or PRIME_NEW"
#endif

// line position in sensor spacings (0..4) -> raw tuple (dark line 200, bg 800)
static void lineAtP(double p, int bg) {
  int r[5];
  for (int i = 0; i < 5; i++) {
    double d = p - i; if (d < 0) d = -d;
    double t = d < 1.0 ? (1.0 - d) : 0.0;
    r[i] = (int)(bg - (double)(bg - 200) * t);
  }
  setRaw(r[0], r[1], r[2], r[3], r[4]);
}
static void crossingStripe(int bg) { setRaw(200, bg, 200, bg, 200); }

int main(int argc, char** argv) {
  const char* outPath = (argc > 1) ? argv[1] : "diff_out.csv";
  FILE* out = fopen(outPath, "w");
  if (!out) { printf("cannot open %s\n", outPath); return 2; }

  Serial.tx.clear(); Serial.rx.clear();
  g_ms = 0;
  for (int i = 0; i < 32; i++) { g_pinPwm[i] = 0; g_pinLevel[i] = 0; g_pinDir[i] = 0; }
  memset(EEPROM.mem, 0, sizeof(EEPROM.mem));
  primeEEPROM();
  setup();                       // boots with loaded calibration, no boot calibration
  fprintf(out, "# boot-tx-contained-loaded=%d\n",
          (int)(Serial.tx.find("calibration loaded") != std::string::npos));
  Serial.tx.clear();

  // tick helper: advance 20 ms, call loop(), dump observables
  enum Mode { LINE, STRIPE, LOST };
  auto tick = [&](Mode m, double p, int bg) {
    if (m == LINE) lineAtP(p, bg);
    else if (m == STRIPE) crossingStripe(bg);
    else setRaw(bg, bg, bg, bg, bg);        // LOST: uniform background, no line anywhere
    g_ms += 20;
    loop();
    fprintf(out, "%lu,%u,%u,%d,%d,%d,%d,%d\n",
            g_ms, g_pinPwm[5], g_pinPwm[6],
            g_pinLevel[14], g_pinLevel[15], g_pinLevel[16], g_pinLevel[17],
            g_pinLevel[8]);
  };

  // start driving (countdown consumed by auto-advance inside millis()/available())
  g_serialAutoAdvanceMs = 20;
  sendLine("g");
  g_serialAutoAdvanceMs = 0;

  // PHASE 0: centred, clean background (bg=800)
  for (int i = 0; i < 15; i++) tick(LINE, 2.0, 800);
  // PHASE 1: slide the line LEFT in small steps to the edge and off
  for (double p = 1.8; p >= -1.0; p -= 0.1) tick(LINE, p, 800);
  // PHASE 2: truly LOST for 15 ticks (search spin expected, left side memory)
  for (int i = 0; i < 15; i++) tick(LOST, 0.0, 800);
  // PHASE 3: line reappears at sensor 4 (right), 12 ticks -> re-acquire
  for (int i = 0; i < 12; i++) tick(LINE, 4.0, 800);
  // PHASE 4: crossing stripe over a centred robot, 6 ticks
  for (int i = 0; i < 6; i++) tick(STRIPE, 0.0, 800);
  // PHASE 5: back to centred, 25 ticks (hold window expires, PID resumes)
  for (int i = 0; i < 25; i++) tick(LINE, 2.0, 800);
  // PHASE 6: noisy background run (bg raw 790 -> calibrated ~983 -> inverted ~17)
  for (double p = 2.0; p >= 0.8; p -= 0.2)       // drift line left over noisy bg
    for (int i = 0; i < 3; i++) tick(LINE, p, 790);
  for (int i = 0; i < 10; i++) tick(LINE, 1.0, 790);
  // PHASE 7: stop
  sendLine("s");
  g_ms += 20; loop();
  fprintf(out, "%lu,%u,%u,%d,%d,%d,%d,%d\n",
          g_ms, g_pinPwm[5], g_pinPwm[6],
          g_pinLevel[14], g_pinLevel[15], g_pinLevel[16], g_pinLevel[17],
          g_pinLevel[8]);
  fclose(out);
  printf("wrote %s\n", outPath);
  return 0;
}
