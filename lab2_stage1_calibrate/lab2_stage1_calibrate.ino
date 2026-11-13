// iss lab2 - stage 1: sensor read + calibration (no PID, no readLine yet)
// motors wired from LAB1 stage7: ENA=5 IN1=A0 IN2=A1, ENB=6 IN3=A2 IN4=A3
// SIGN_L=SIGN_R=-1 in LAB1 -> both wheel "stopped" here means PWM 0 + bridge off
// calibration is BY HAND: I rotate the robot so the strip sweeps the line,
// no motor motion (safer for the first run - kickoff decision)
// indicator LED on D8 - NOT D13, D13 is the TLC1543 CLOCK pin owned by the library

#include <TRSensors.h>

static const unsigned long SERIAL_BAUD = 9600;    // same link as LAB1
static const uint8_t NUM_SENSORS = 5;

// motor pins copied from LAB1 stage7 pin map (frozen values)
static const uint8_t PIN_ENA = 5,  PIN_IN1 = A0, PIN_IN2 = A1;   // left pwm + dir
static const uint8_t PIN_ENB = 6,  PIN_IN3 = A2, PIN_IN4 = A3;   // right pwm + dir
static const uint8_t PIN_LED = 8;                                // calib indicator

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int rawValues[NUM_SENSORS];

// one safe-stopped state helper: pwm 0 + both bridge inputs low
void motorsStop() {
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
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

  Serial.println("LAB2 Stage 1 - calibration");
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
  Serial.println("send any character for one RAW+CAL reading");
}

void loop() {
  // print only once per received character - continuous spam is uncopiable
  if (Serial.available() > 0) {
    Serial.read();  // drain the character, it only gates the reading

    trs.AnalogRead(rawValues);          // raw converter values 0..1023
    trs.readCalibrated(sensorValues);   // rescaled 0..1000 by calibration

    // note: readCalibrated re-reads the ADC, so CAL is a second sample ~2-3 ms
    // after RAW - fine for hand-held polarity checks at this stage
    printValues("RAW  : ", rawValues);
    printValues("CAL  : ", sensorValues);
  }
}
