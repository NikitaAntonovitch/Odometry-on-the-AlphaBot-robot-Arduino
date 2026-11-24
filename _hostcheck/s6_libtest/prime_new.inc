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
