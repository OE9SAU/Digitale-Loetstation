#include <EEPROM.h>

#define PIN_HEAT     5  // PWM-Ausgang für die Heizung
#define REGELZEIT    100
#define P_KOEFF_P    7
#define P_KOEFF_PID  9.0
#define I_KOEFF_PID  0.2
#define D_KOEFF_PID  19.5

int a_tist = 0, a_tsoll = 60, pwm = 0, lr_tist = 0;
unsigned long lzp_regler = 0;

void setup() {
  TCCR1A = 0b00100011;
  TCCR1B = 0b00000001;
  OCR1B  = 0;
}

void loop() {
  pwmRampe();
  reglerTemperatur();
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

void reglerTemperatur() {
  if (millis() < lzp_regler + REGELZEIT) return;
  pwm = P_KOEFF_PID * (a_tsoll - a_tist) + I_KOEFF_PID * pwm - D_KOEFF_PID * (a_tist - lr_tist);
  pwm = int(min(max(pwm, 0), 1023));
  OCR1B = pwm;
  lr_tist = a_tist;
  lzp_regler = millis();
}
