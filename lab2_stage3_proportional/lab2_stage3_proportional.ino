// iss lab2 - stage 3: proportional controller - FIRST MOVING stage
// builds on stage 2: calibration, readError(), lineLost() unchanged
// adds pStep()/applyCorrection(), fixed-period 50 Hz millis loop (LAB1 pattern, no delay)
// SAFETY: motors OFF until 'g' AND 'y' within 2 s; dry-run ON by default so the
// control loop runs and prints (u, L, R) with wheels stopped - sign check first
// motor layer from LAB1 stage7: ENA=5 IN1=A0 IN2=A1, ENB=6 IN3=A2 IN4=A3,
// SIGN_L=SIGN_R=-1 handled inside driveWheels() exactly like LAB1 does
// status LED on D8 - NOT D13, D13 is the TLC1543 CLOCK pin owned by the library

#include <TRSensors.h>

static const unsigned long SERIAL_BAUD = 9600;    // same link as LAB1
static const uint8_t NUM_SENSORS = 5;
static const int SETPOINT = 2000;                 // line under strip centre
static const int LOST_LIMIT = 2000;               // |error| beyond this = no line
static bool white_line = false;   // set from Stage 1 polarity measurement; flip with 'w' if unsure at school

static const int PWM_MAX = 255;     // full duty
static const int PWM_MIN = 85;      // LAB1 floor-measured dead band (75) + margin
static const unsigned long LOOP_PERIOD_MS = 20;   // 50 Hz, LAB1 CTRL_MS

// tunables (assignment: change by terminal command, no recompile)
static float kp        = 0.0f;   // tunable live; 0 = no motion
static int   baseSpeed = 0;     // tunable live; 0 = no motion
static bool  driving   = false; // motors OFF until I send 'g'
static bool  dryRun    = true;  // dry-run ON by default: loop runs, motors OFF
static bool  trace     = true;  // ON in dry-run, auto-OFF when driving starts

// motor pins copied from LAB1 stage7 pin map (frozen values)
static const uint8_t PIN_ENA = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // left pwm + dir
static const uint8_t PIN_ENB = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // right pwm + dir
static const uint8_t PIN_LED = 8;                                // calib indicator
static const int8_t SIGN_L = -1;   // floor-verified 14.10: bridge-fwd rolls backward,
static const int8_t SIGN_R = -1;   // both signs flipped (same correction as LAB1)

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int rawValues[NUM_SENSORS];
static int lastLinePos = 0;   // raw readLine() return (kept from stage 2)

// ===== stage 2 functions, unchanged =====
// SIGNED int — never change this type, the lost-line sign depends on it
int readError() {
  lastLinePos = trs.readLine(sensorValues, white_line);
  return lastLinePos - SETPOINT;
}

bool lineLost(int e) {
  return e > LOST_LIMIT || e < -LOST_LIMIT;   // sign of e survives for the search stage
}

// ===== stage 3: proportional controller =====
// ONE step, no state - P has no memory. u = kp * error, clamped to full PWM range
int pStep(int error) {
  long u = (long)(kp * error);
  if (u >  PWM_MAX) u =  PWM_MAX;
  if (u < -PWM_MAX) u = -PWM_MAX;
  return (int)u;
}

// expected direction (matches the dry-run printout): line RIGHT of centre -> error>0 ->
// u>0 -> leftSpeed = base+u is LARGER, rightSpeed = base-u is SMALLER -> robot turns
// right, TOWARD the line (L>R turns right). Printout shows the opposite -> sign is
// wrong: flip with a negative kp, do not rewire
void applyCorrection(int u) {
  int leftSpeed  = baseSpeed + u;
  int rightSpeed = baseSpeed - u;

  // clip-only decision (stage 3): clamp each wheel into [-255,255], leave the other alone
  if (leftSpeed  >  PWM_MAX) leftSpeed  =  PWM_MAX;
  if (leftSpeed  < -PWM_MAX) leftSpeed  = -PWM_MAX;
  if (rightSpeed >  PWM_MAX) rightSpeed =  PWM_MAX;
  if (rightSpeed < -PWM_MAX) rightSpeed = -PWM_MAX;

  // dead band guard: below pwmMin a wheel buzzes but does not turn - refuse to drive
  if (baseSpeed < PWM_MIN) {
    motorsStop();
    if (trace) Serial.println("GUARD baseSpeed < pwmMin, motors held");
    return;
  }

  if (!driving || dryRun) {
    // dry-run prints the would-be speeds so the sign can be verified before motion
    if (trace) {
      Serial.print("DRY u="); Serial.print(u);
      Serial.print(" L="); Serial.print(leftSpeed);
      Serial.print(" R="); Serial.println(rightSpeed);
    }
    motorsStop();
    return;
  }

  driveWheels(leftSpeed, rightSpeed);
}

// ===== motor layer (LAB1 pattern incl. the floor-verified wiring sign flip) =====
void motorL(int pwm) {
  pwm *= SIGN_L;
  digitalWrite(PIN_IN1, pwm >= 0);
  digitalWrite(PIN_IN2, pwm <  0);
  analogWrite(PIN_ENA, min(abs(pwm), PWM_MAX));
}
void motorR(int pwm) {
  pwm *= SIGN_R;
  digitalWrite(PIN_IN3, pwm <  0);
  digitalWrite(PIN_IN4, pwm >= 0);
  analogWrite(PIN_ENB, min(abs(pwm), PWM_MAX));
}
void driveWheels(int leftSpeed, int rightSpeed) {
  motorL(leftSpeed);
  motorR(rightSpeed);
}
void motorsStop() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// ===== one control tick at the fixed 50 Hz cadence =====
void tickController() {
  int e = readError();
  if (lineLost(e)) {
    // no line = no error: do not feed the out-of-range number to the controller
    motorsStop();
    if (trace) {
      Serial.print("LOST side=");
      Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
    }
    return;
  }
  int u = pStep(e);
  applyCorrection(u);
  if (trace && driving && !dryRun) {
    Serial.print("TRACE t="); Serial.print(millis());
    Serial.print(" e="); Serial.print(e);
    Serial.print(" u="); Serial.print(u);
    Serial.print(" L="); Serial.print(baseSpeed + u);
    Serial.print(" R="); Serial.println(baseSpeed - u);
  }
}

// ===== serial command line parsing =====
// read the whole line up to \n; first char = command; k takes a float, b an int
static char lineBuf[24];
static uint8_t lineLen = 0;

void handleLine(const char* buf) {
  char cmd = buf[0];
  const char* arg = buf + 1;
  while (*arg == ' ') arg++;   // ignore whitespace before the number

  switch (cmd) {
    case 'r':
      trs.AnalogRead(rawValues);
      printValues("RAW  : ", rawValues);
      break;
    case 'c':
      trs.readCalibrated(sensorValues);
      printValues("CAL  : ", sensorValues);
      break;
    case 'p': {
      trs.AnalogRead(rawValues);
      int e = readError();
      Serial.print("POS raw=");
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(rawValues[i]);
        if (i < NUM_SENSORS - 1) Serial.print(',');
      }
      Serial.print(" linePos="); Serial.print(lastLinePos);
      Serial.print(" error=");   Serial.print(e);
      Serial.print(" lost=");    Serial.print(lineLost(e) ? 1 : 0);
      Serial.print(" side=");    Serial.println(e < 0 ? 'L' : (e > 0 ? 'R' : '?'));
      break;
    }
    case 'd':
      dryRun = !dryRun;
      Serial.print("dryRun="); Serial.println(dryRun ? 1 : 0);
      break;
    case 'g':
      // dead-man's switch: 'g' alone does nothing - a second char 'y' must arrive
      // within 2 s, otherwise the start is aborted (a typo must not launch the robot)
      Serial.println("CONFIRM: send y within 2 s to start driving");
      {
        unsigned long t0 = millis();
        while (millis() - t0 < 2000) {
          if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'y') {
              driving = true;
              dryRun = false;
              trace = false;   // printing inside the loop fights the controller for time
              Serial.print("DRIVING kp="); Serial.print(kp, 3);
              Serial.print(" base="); Serial.println(baseSpeed);
              return;
            }
            break;   // any other char aborts
          }
        }
        Serial.println("ABORTED");
      }
      break;
    case 's':
      driving = false;
      dryRun = true;
      trace = true;
      motorsStop();
      Serial.println("STOPPED");
      break;
    case 'k':
      kp = atof(arg);
      Serial.print("kp="); Serial.println(kp, 3);
      break;
    case 'b':
      baseSpeed = atoi(arg);
      Serial.print("baseSpeed="); Serial.println(baseSpeed);
      break;
    case 't':
      trace = !trace;
      Serial.print("trace="); Serial.println(trace ? 1 : 0);
      break;
    case 'h':
      Serial.println("Cmds: r=raw c=cal p=pos d=dry-run g=go s=stop k<val>=kp b<val>=base t=trace h=help");
      break;
    case '\r':
    case '\n':
    case 0:
      break;   // empty line - ignore
    default:
      Serial.print("? "); Serial.println(cmd);
      break;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ===== stage 1/2 helpers kept verbatim =====
void printValues(const char* label, unsigned int* v) {
  Serial.print(label);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(v[i]);
    if (i < NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void setup() {
  // motor pins to a safe stopped state BEFORE anything else moves
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  motorsStop();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(SERIAL_BAUD);

  Serial.println("LAB2 Stage 3 - proportional");
  Serial.println("CALIBRATING... rotate the robot so the strip crosses the line");
  digitalWrite(PIN_LED, HIGH);

  // 400 calls x 10 reads each = ~10 s, sweep line + background in that window
  for (int i = 0; i < 400; i++) {
    trs.calibrate();
  }

  digitalWrite(PIN_LED, LOW);
  Serial.println("CALIBRATION DONE");

  printValues("MIN 0..4: ", trs.calibratedMin);
  printValues("MAX 0..4: ", trs.calibratedMax);
  Serial.println("Cmds: r=raw c=cal p=pos d=dry-run g=go s=stop k<val>=kp b<val>=base t=trace h=help");
  Serial.print("dryRun="); Serial.print(dryRun ? 1 : 0);
  Serial.print(" driving="); Serial.println(driving ? 1 : 0);
  Serial.println("kp=0 base=0 - set k<val> and b<val>, verify sign in dry-run, then g+y");

  motorsStop();   // last line of setup: safe state
}

// fixed-period non-blocking loop, LAB1 pattern: lastTick += period keeps the
// cadence drift-free even if a tick overruns
static unsigned long lastTick = 0;

void loop() {
  readSerialCommands();   // non-blocking, drains the line buffer
  if (millis() - lastTick >= LOOP_PERIOD_MS) {
    lastTick += LOOP_PERIOD_MS;
    tickController();
  }
}
