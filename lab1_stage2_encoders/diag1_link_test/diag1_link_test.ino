// diag1: minimal link test - prints a counter every second, nothing else
// if THIS prints clean at 115200, board + usb cable + serial monitor are proven good
void setup() {
  Serial.begin(115200);
  Serial.println(F("== diag1: link test =="));
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
