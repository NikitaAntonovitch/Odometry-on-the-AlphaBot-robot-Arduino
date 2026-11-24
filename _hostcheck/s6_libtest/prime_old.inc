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
