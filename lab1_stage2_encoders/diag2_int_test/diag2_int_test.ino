// diag2: link test + both encoder interrupts attached, no motors.
// if diag1 is clean but THIS corrupts -> the encoder lines/interrupts kill the link
void isrL() { volatile static int32_t c = 0; c++; }
void isrR() { volatile static int32_t c = 0; c++; }

void setup() {
  Serial.begin(115200);
  Serial.println(F("== diag2: link + interrupt test =="));
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), isrR, CHANGE);
}

void loop() {
  static unsigned long n = 0;
  static unsigned long last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    Serial.print(F("alive "));
    Serial.println(++n);
  }
}
