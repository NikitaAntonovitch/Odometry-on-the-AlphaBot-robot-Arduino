// iss lab1 - stage 2: wheel distance and speed from the encoders
// stage 1 motor layer unchanged, encoder module added: signed counts -> millimetres + speed
// forbidden: AlphaBot.h and anything from the waveshare examples - everything here is written from scratch

/* ===== interface speeds =====
   kept in one place, a mismatch here gives the symptom "nothing works" */
static const unsigned long SERIAL_BAUD = 9600;   // IDE monitor default - zero-touch link speed,
                                                 // plenty for one-char commands + short lines

/* ===== pin map ===== */
static const uint8_t PIN_ENA  = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // channel A pwm + direction
static const uint8_t PIN_ENB  = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // channel B pwm + direction
static const uint8_t PIN_CNTL = 2,  PIN_CNTR = 3;                 // encoders, INT0 and INT1

static const int8_t SIGN_L = +1;   // set to -1 if the left wheel turns backwards on the stand
static const int8_t SIGN_R = +1;   // set to -1 if the right wheel turns backwards on the stand

static const int PWM_MAX  = 255;   // full duty
static const int PWM_STEP = 5;     // step for the dead band search with + / -

/* ===== odometry constants =====
   S and D come from the robot, they are measured not assumed (guide: count the slots!) */
static const uint8_t  SLOTS   = 20;      // measured: 20 slots on both discs, electrical 1 rev = 40 counts
static const uint8_t  N_REV   = 2*SLOTS; // counts per wheel rev: CHANGE fires on both edges
static const float    WHEEL_D = 65.0f;   // mm, measured across the tyre (exactly 6.5 cm)
static const float    K_MM    = 5.1087f;  // mm per count, calibrated: 1000 mm section, 4 runs, spread 1.54 %
                                          // (geometry pi*D/2S = 5.105, agreement +0.07 %)
static const uint16_t COAST_MS = 2000;   // edges accepted this long after drive off; conservative bound,
                                         // far above the visible run-on at cruise (report stage2 sec.4)
static const uint16_t SPD_WIN  = 500;    // speed window ms, >=10 counts at cruising pwm 90
static const uint16_t DEBOUNCE_US = 800; // min edge spacing: real max is 1 edge / ~8 ms at top speed,
                                         // sensor chatter (LM393 at a slot edge, not a schmitt trigger
                                         // - guide p.7 discrepancy) oscillates much faster -> filtered

/* ===== encoder state =====
   the isr touches these, the main loop reads them -> volatile, and 32 bit on an 8 bit
   core means every read in the loop must be atomic or it can tear */
static volatile int32_t cntL = 0, cntR = 0;   // signed counts, sign = commanded direction
static volatile int8_t  dirL  = +1, dirR  = +1; // last commanded direction of each wheel
static volatile uint32_t lastDriveMs = 0;     // millis() of the last moment the drive was on
static volatile int32_t snapL = 0, snapR = 0; // frozen snapshot taken when the window closes
static bool benchMode = false;                // bench = count every edge (hand checks), live = gated

/* ===== isr layer =====
   shortest possible: one window check, one sign fix, one increment, nothing else.
   live mode: the window check lives here, so standstill vibration CANNOT reach the
   counter, while genuine run-on rotation during the window still counts.
   bench mode: no gating, hand-turn checks count raw edges with the drive off.
   debounce: an edge closer than DEBOUNCE_US to the previous one on the SAME channel is
   sensor chatter (LM393 oscillating at a slot edge), not a real slot - one real edge
   every ~8 ms at top speed, chatter is far denser */
static volatile uint32_t lastEdgeUsL = 0, lastEdgeUsR = 0;

void isrL() {
  uint32_t t = micros();
  if (t - lastEdgeUsL < DEBOUNCE_US) return;        // chatter, not a slot edge
  lastEdgeUsL = t;
  if (!benchMode && millis() - lastDriveMs >= COAST_MS) return;   // window closed -> freeze
  if (dirL >= 0) cntL++; else cntL--;               // sensor cannot tell direction, the motor layer does
}
void isrR() {
  uint32_t t = micros();
  if (t - lastEdgeUsR < DEBOUNCE_US) return;
  lastEdgeUsR = t;
  if (!benchMode && millis() - lastDriveMs >= COAST_MS) return;
  if (dirR >= 0) cntR++; else cntR--;
}

bool encCounting() {                          // main loop mirror of the isr gate, for display
  return benchMode || (millis() - lastDriveMs) < COAST_MS;
}

int32_t atomicRead(volatile int32_t &v) {         // no tearing on a 32 bit value on an 8 bit core
  uint8_t s = SREG;                               // remember the interrupt state
  noInterrupts();                                 // down, but ONLY for the read itself
  int32_t t = v;                                  // single copy out
  SREG = s;                                       // restore EXACTLY the previous state - if interrupts
  return t;                                       // were already off they stay off (guide requirement)
}

int32_t encCountL() {                             // main loop reads the counter through this only
  return encCounting() ? atomicRead(cntL) : atomicRead(snapL);
}

int32_t encCountR() {
  return encCounting() ? atomicRead(cntR) : atomicRead(snapR);
}

void zeroEncoders() {                             // manual re-zero from the serial menu
  noInterrupts(); cntL = 0; cntR = 0; snapL = 0; snapR = 0; interrupts();
}

/* ===== motor layer =====
   the only place in the program that touches the bridge pins,
   the rest of the program only says "wheel L at this signed speed" */
void motorL(int pwm) {                          // sign = direction, magnitude = duty
  pwm *= SIGN_L;                                // apply the measured direction correction
  dirL = (pwm >= 0) ? +1 : -1;                  // sign source for the encoder: last commanded direction
  digitalWrite(PIN_IN1, pwm >= 0);              // nominal forward for A: IN1=1 IN2=0
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX)); // speed lives on the enable pin only
  if (pwm) lastDriveMs = millis();              // driven NOW -> window stays open while pwm lasts
}

void motorR(int pwm) {                          // int covers -255..255, a char would not
  pwm *= SIGN_R;
  dirR = (pwm >= 0) ? +1 : -1;
  digitalWrite(PIN_IN3, pwm <  0);              // nominal forward for B: IN3=0 IN4=1
  digitalWrite(PIN_IN4, pwm >= 0);
  analogWrite(PIN_ENB, min(abs(pwm), PWM_MAX));
  if (pwm) lastDriveMs = millis();
}

void coast() {                                  // en low = all switches open, wheels spin free
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);
}

void brake() {                                  // en high + both inputs low = low side short
  digitalWrite(PIN_IN1, LOW);                   // winding shorted -> dynamic braking
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENB, HIGH);
}

/* ===== odometry output =====
   counting rule (the compromise the guide asks to design):
   while driven or within COAST_MS after the last drive command the isr is trusted,
   so run-on distance still lands in the counter; after that the counter freezes,
   so standstill vibration cannot invent distance. bench mode (x key) suspends the
   freeze so the hand-turn checks can count raw edges with the drive off */

float encMM(int32_t c) { return (float)c * K_MM; }   // counts -> mm, main loop only, not in isr

/* ===== test state ===== */
static int curPwm = 90;            // starting duty, comfortably above the expected dead band
static uint8_t curMode = 0;        // 0 stop, 1 fwd, 2 back, 3 spinL, 4 spinR, 5 pivotL, 6 pivotR
static uint32_t winStart = 0;      // speed window bookkeeping
static float spdL = 0.0f, spdR = 0.0f;           // latest window speed, mm/s

void applyMode() {                              // wheel speeds for the current mode
  int l = 0, r = 0;
  switch (curMode) {
    case 1: l = +curPwm; r = +curPwm; break;    // straight forward
    case 2: l = -curPwm; r = -curPwm; break;    // straight backward
    case 3: l = -curPwm; r = +curPwm; break;    // spin turn left, wheels counter-rotate
    case 4: l = +curPwm; r = -curPwm; break;    // spin turn right
    case 5: l = 0;       r = +curPwm; break;    // pivot about the left wheel
    case 6: l = +curPwm; r = 0;       break;    // pivot about the right wheel
    default: coast(); return;                   // mode 0 = stopped
  }
  motorL(l);
  motorR(r);
}

/* ===== serial ui =====
   no printing inside loop, one line per command so results stay copyable */
const __FlashStringHelper* modeName(uint8_t m) {
  switch (m) {
    case 1: return F("fwd");
    case 2: return F("back");
    case 3: return F("spinL");
    case 4: return F("spinR");
    case 5: return F("pivotL");
    case 6: return F("pivotR");
  }
  return F("stop");
}

void printState() {
  Serial.print(F("mode="));
  Serial.print(modeName(curMode));
  Serial.print(F(" pwm="));
  Serial.println(curPwm);
}

void printEnc() {                                // e - one line, copyable, both wheels
  Serial.print(F("enc L="));
  Serial.print((long)encCountL());
  Serial.print(F("cnt "));
  Serial.print(encMM(encCountL()), 1);
  Serial.print(F("mm | R="));
  Serial.print((long)encCountR());
  Serial.print(F("cnt "));
  Serial.print(encMM(encCountR()), 1);
  Serial.print(F("mm "));
  Serial.println(encCounting() ? F("counting") : F("frozen"));
}

void printSpeed() {                              // s - window speed, the characteristic input
  Serial.print(F("spd L="));
  Serial.print(spdL, 1);
  Serial.print(F("mm/s R="));
  Serial.print(spdR, 1);
  Serial.print(F("mm/s pwm="));
  Serial.println(curPwm);
}

void printMenu() {
  Serial.println(F("== stage2 encoders =="));
  Serial.println(F("1 straight fwd    2 straight back"));
  Serial.println(F("3 spin left       4 spin right"));
  Serial.println(F("5 pivot left      6 pivot right"));
  Serial.println(F("+ / - pwm up/down for dead band search"));
  Serial.println(F("e encoder counts+mm  z zero counters  s window speed"));
  Serial.println(F("x bench mode on/off (hand-turn checks)"));
  Serial.println(F("p print state     c coast     b brake"));
  Serial.println(F("h this menu"));
  printState();
}

void handleChar(char ch) {                      // called once per received character
  switch (ch) {
    case '1': case '2': case '3': case '4': case '5': case '6':
      curMode = ch - '0';                       // digit becomes the mode number
      applyMode();
      printState();
      break;
    case 'c':
      curMode = 0;
      coast();
      printState();
      break;
    case 'b':
      curMode = 0;
      brake();
      printState();
      break;
    case '+':
      curPwm = min(curPwm + PWM_STEP, PWM_MAX);
      if (curMode) applyMode();                 // live change while a mode is running
      printState();
      break;
    case '-':
      curPwm = max(curPwm - PWM_STEP, 0);
      if (curMode) applyMode();
      printState();
      break;
    case 'e':
      printEnc();
      break;
    case 'z':
      zeroEncoders();
      printEnc();
      break;
    case 's':
      printSpeed();
      break;
    case 'x':                                   // x - bench/live toggle, explained in the guide report
      benchMode = !benchMode;
      Serial.print(F("bench="));
      Serial.println(benchMode ? F("on (raw edges, no freeze)") : F("off (coast window gating)"));
      break;
    case 'p':
      printState();
      break;
    case 'h': case '?':
      printMenu();
      break;
    default:
      break;                                    // unknown characters are skipped silently
  }
}

void setup() {
  // order matters: pin directions first, then the safe drive state, only then the link
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_CNTL, INPUT_PULLUP);              // sensor outputs are inputs for the mcu;
  pinMode(PIN_CNTR, INPUT_PULLUP);              // pull-ups stop FLOATING pin interrupts when
                                                // the sensor cable is unplugged or its driver
                                                // is off (LM393 open collector still needs bias)
  coast();                                      // drive disabled before anything can move it
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("== lab1 stage2: encoders + odometry =="));  // greeting names the loaded program
  attachInterrupt(digitalPinToInterrupt(PIN_CNTL), isrL, CHANGE);  // both edges: N_rev = 2S
  attachInterrupt(digitalPinToInterrupt(PIN_CNTR), isrR, CHANGE);
  winStart = millis();                          // speed window starts with the clock
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {                 // non blocking, never waits
    handleChar((char)Serial.read());
  }
  // drive alive -> keep the counting window open while a mode is actively driving
  if (curMode != 0) {
    lastDriveMs = millis();                     // driven NOW, the gate in the isr stays open
  }
  // snapshot when the counting window closes: the frozen value the user keeps seeing
  static bool wasCounting = true;
  bool nowCounting = encCounting();
  if (wasCounting && !nowCounting) {            // edge: window just closed
    noInterrupts(); snapL = cntL; snapR = cntR; interrupts();   // copy the final counts
  }
  wasCounting = nowCounting;
  // speed window: counts in a 200 ms window, evaluated on every pass, cheap
  uint32_t t = millis();
  if (t - winStart >= SPD_WIN) {
    static int32_t prevL = 0, prevR = 0;        // window snapshots, main loop side only
    int32_t cl = encCountL(), cr = encCountR(); // atomic reads, never torn
    spdL = (cl - prevL) * K_MM * 1000.0f / (t - winStart);
    spdR = (cr - prevR) * K_MM * 1000.0f / (t - winStart);
    prevL = cl; prevR = cr;
    winStart = t;
  }
}
