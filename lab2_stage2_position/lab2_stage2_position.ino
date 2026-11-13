// iss lab2 - stage 2: line position + lost-line detection (no PID, no motor motion)
// builds on stage 1: same calibration block, same raw/cal prints, adds readError()/lineLost()
// robot stays on the bench - I move it by hand, wheels stay stopped the whole run
// motor pins from LAB1 stage7 map, held in the safe stopped state (PWM 0 + bridge off)
// status LED on D8 - NOT D13, D13 is the TLC1543 CLOCK pin owned by the library

#include <TRSensors.h>

static const unsigned long SERIAL_BAUD = 9600;    // same link as LAB1
static const uint8_t NUM_SENSORS = 5;
static const int SETPOINT = 2000;                 // line under strip centre
static const int LOST_LIMIT = 2000;               // |error| beyond this = no line
static bool white_line = false;   // set from Stage 1 polarity measurement; flip with 'w' if unsure at school
// library source note: white_line=0 INVERTS the calibrated values (1000-v), so it tracks
// the LIGHT areas - for black line on white band the final value is likely true, confirm with 'p'

// motor pins copied from LAB1 stage7 pin map (frozen values)
static const uint8_t PIN_ENA = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // left pwm + dir
static const uint8_t PIN_ENB = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // right pwm + dir
static const uint8_t PIN_LED = 8;                                // calib indicator

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int rawValues[NUM_SENSORS];
static int lastLinePos = 0;   // raw readLine() return kept for printouts and later stages

void motorsStop() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// SIGNED int — never change this type, the lost-line sign depends on it
// library returns -1 or 4001 when the line is lost -> error -2001 / +2001,
// the sign says which side the line was LAST seen on (search stage needs it)
int readError() {
  lastLinePos = trs.readLine(sensorValues, white_line);
  return lastLinePos - SETPOINT;
}

bool lineLost(int e) {
  // explicit compare, not abs(): the sign of e is never touched here
  return e > LOST_LIMIT || e < -LOST_LIMIT;
}

char sideOf(int e) {
  // sign of the ERROR decides, not the sign of linePos (linePos -1/4001 is the lost mark)
  if (e < 0) return 'L';
  if (e > 0) return 'R';
  return '?';   // dead centre, or lost with e==0 (should not happen)
}

void printValues(const char* label, unsigned int* v) {
  // one line, comma separated, easy to copy from the monitor
  Serial.print(label);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(v[i]);
    if (i < NUM_SENSORS - 1) Serial.print(',');
  }
  Serial.println();
}

void printRow(const char* tag, unsigned long t, int pos, int e) {
  Serial.print(tag);
  Serial.print(" t=");   Serial.print(t);
  Serial.print(" linePos="); Serial.print(pos);
  Serial.print(" error=");   Serial.print(e);
  Serial.print(" lost=");    Serial.println(lineLost(e) ? 1 : 0);
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

  Serial.println("LAB2 Stage 2 - position + lost-line");
  Serial.println("CALIBRATING... rotate the robot so the strip crosses the line");
  digitalWrite(PIN_LED, HIGH);

  // 400 calls x 10 reads each = ~10 s, sweep line + background in that window
  for (int i = 0; i < 400; i++) {
    trs.calibrate();
  }

  digitalWrite(PIN_LED, LOW);
  Serial.println("CALIBRATION DONE");

  // stored extremes per sensor - no zero range here means calibration worked
  printValues("MIN 0..4: ", trs.calibratedMin);
  printValues("MAX 0..4: ", trs.calibratedMax);
  Serial.println("Commands: r=raw, c=cal, p=position, s=sweep, e=edge, w=flip white_line, h=help");
}

void loop() {
  // every output is one block per received character - no continuous printing
  if (Serial.available() <= 0) return;
  char cmd = Serial.read();

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
      trs.AnalogRead(rawValues);   // raw sample for the printout (readError re-reads ~2-3 ms later)
      int e = readError();         // readLine lives ONLY inside readError()
      Serial.print("POS raw=");
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(rawValues[i]);
        if (i < NUM_SENSORS - 1) Serial.print(',');
      }
      Serial.print(" linePos="); Serial.print(lastLinePos);
      Serial.print(" error=");   Serial.print(e);
      Serial.print(" lost=");    Serial.print(lineLost(e) ? 1 : 0);
      Serial.print(" side=");    Serial.println(sideOf(e));
      break;
    }

    case 's': {
      // 60 reads nominal every 50 ms; at 9600 one line prints in ~45 ms so the
      // real spacing is ~95-100 ms - the t= column carries the true elapsed ms
      Serial.println("SWEEP start - slide the line across the strip slowly");
      unsigned long t0 = millis();
      for (int i = 0; i < 60; i++) {
        int e = readError();
        printRow("SWEEP", millis() - t0, lastLinePos, e);
        delay(50);
      }
      Serial.println("SWEEP DONE");
      break;
    }

    case 'e': {
      // 40 reads nominal every 100 ms, same print-time caveat, t= is truthful
      Serial.println("EDGE start - slide the line slowly off the strip edge");
      unsigned long t0 = millis();
      for (int i = 0; i < 40; i++) {
        int e = readError();
        printRow("EDGE", millis() - t0, lastLinePos, e);
        delay(100);
      }
      Serial.println("EDGE DONE");
      break;
    }

    case 'w':
      white_line = !white_line;
      Serial.print("white_line="); Serial.println(white_line ? 1 : 0);
      Serial.println("re-check with 'p': line right of centre must give side=R, left must give side=L");
      break;

    case 'h':
      Serial.println("Commands: r=raw, c=cal, p=position, s=sweep, e=edge, w=flip white_line, h=help");
      break;

    default:
      break;   // swallow newline/CR and unknown keys silently
  }
}
