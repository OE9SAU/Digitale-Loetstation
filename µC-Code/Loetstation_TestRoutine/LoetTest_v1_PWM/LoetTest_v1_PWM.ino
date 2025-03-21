#define PIN_HEAT     5  // PWM-Ausgang für die Heizung

void setup() {
  TCCR1A = 0b00100011;
  TCCR1B = 0b00000001;
  OCR1B  = 0;
}

void loop() {
  pwmRampe();
}

void pwmRampe() {
  const int pwmMin = 0, pwmMax = 1023;
  const int rampTime = 20000;
  const int stepDelay = rampTime / (2 * pwmMax);

  for (int i = pwmMin; i <= pwmMax; i++) {
    OCR1B = i;
    delay(stepDelay);
  }
  for (int i = pwmMax; i >= pwmMin; i--) {
    OCR1B = i;
    delay(stepDelay);
  }
}