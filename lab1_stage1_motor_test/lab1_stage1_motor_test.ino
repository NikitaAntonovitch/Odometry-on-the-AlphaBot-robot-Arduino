// iss lab1 - stage 1: open loop drive test (no encoder feedback yet)
// you type one character in the serial monitor, the robot answers with one copyable line
// forbidden: AlphaBot.h and anything from the waveshare examples - everything here is written from scratch

/* ===== interface speeds =====
   kept in one place, a mismatch here gives the symptom "nothing works" */
static const unsigned long SERIAL_BAUD = 115200;   // must match the serial monitor setting

/* ===== pin map ===== */
static const uint8_t PIN_ENA  = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // channel A pwm + direction
static const uint8_t PIN_ENB  = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // channel B pwm + direction

static const int8_t SIGN_L = +1;   // set to -1 if the left wheel turns backwards on the stand
static const int8_t SIGN_R = +1;   // set to -1 if the right wheel turns backwards on the stand
// note: channel-to-wheel wiring is NOT assumed, the guide says to confirm it by measurement
// a code correction like this is explicit and easy to document, re-plugging motors is not

static const int PWM_MAX  = 255;   // full duty
static const int PWM_STEP = 5;     // step for the dead band search with + / -

/* ===== test state ===== */
static int curPwm = 90;            // starting duty, comfortably above the expected dead band
static uint8_t curMode = 0;        // 0 stop, 1 fwd, 2 back, 3 spinL, 4 spinR, 5 pivotL, 6 pivotR

/* ===== motor layer =====
   the only place in the program that touches the bridge pins,
   the rest of the program only says "wheel L at this signed speed" */
void motorL(int pwm) {                          // sign = direction, magnitude = duty
  pwm *= SIGN_L;                                // apply the measured direction correction
  digitalWrite(PIN_IN1, pwm >= 0);              // nominal forward for A: IN1=1 IN2=0
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX)); // speed lives on the enable pin only
}

void motorR(int pwm) {                          // int covers -255..255, a char would not
  pwm *= SIGN_R;
  digitalWrite(PIN_IN3, pwm <  0);              // nominal forward for B: IN3=0 IN4=1
  digitalWrite(PIN_IN4, pwm >= 0);
  analogWrite(PIN_ENB, min(abs(pwm), PWM_MAX));
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

void printMenu() {
  Serial.println(F("== stage1 drive test =="));
  Serial.println(F("1 straight fwd    2 straight back"));
  Serial.println(F("3 spin left       4 spin right"));
  Serial.println(F("5 pivot left      6 pivot right"));
  Serial.println(F("+ / - pwm up/down for dead band search"));
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
  coast();                                      // drive disabled before anything can move it
  Serial.begin(SERIAL_BAUD);
  printMenu();
}

void loop() {
  if (Serial.available() > 0) {                 // non blocking, never waits
    handleChar((char)Serial.read());
  }
}
