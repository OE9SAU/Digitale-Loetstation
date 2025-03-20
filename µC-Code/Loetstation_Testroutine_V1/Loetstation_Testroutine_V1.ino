// ==============================================
// Dieser Code dient dazu die SMD-Lötstation zu testen.
// Entwickler: OE9SAU // Chatgpt
// Version: 1.0 20.03.2025
// ==============================================

#include <TinyWireM.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define PIN_ROT_A    0
#define PIN_ROT_B    1
#define PIN_ROT_PUSH 2
#define PIN_HEAT     5
#define PIN_TEMP     A3
#define PIN_STBY     10
#define PIN_PIEZO    8
#define LCDI2CADR    0x27

#define PROGRAMM_VERSION "1.0"

LiquidCrystal_I2C lcd(LCDI2CADR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

int pwmValue = 0;

void piezoError() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_PIEZO, HIGH);
        delay(100);
        digitalWrite(PIN_PIEZO, LOW);
        delay(100);
    }
}

void setup() {
    pinMode(PIN_ROT_A, INPUT);
    pinMode(PIN_ROT_B, INPUT);
    pinMode(PIN_ROT_PUSH, INPUT_PULLUP);
    pinMode(PIN_HEAT, OUTPUT);
    pinMode(PIN_TEMP, INPUT);
    pinMode(PIN_STBY, INPUT_PULLUP);
    pinMode(PIN_PIEZO, OUTPUT);
    
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("HW Test Routine");
    lcd.setCursor(0, 1);
    lcd.print("Version: ");
    lcd.print(PROGRAMM_VERSION);
    delay(2000);
    lcd.clear();
}

void loop() {
    lcd.print("EEPROM-Test");
    lcd.setCursor(0, 1);
    lcd.print("Druecke Taster...");
    while (digitalRead(PIN_ROT_PUSH) == HIGH);
    delay(300);
    lcd.clear();
    EEPROM.write(0, 42);
    if (EEPROM.read(0) == 42) {
        lcd.print("OK");
    } else {
        lcd.print("Fehler!");
        piezoError();
    }
    delay(1000);
    lcd.clear();
    
    lcd.print("Piezo-Test");
    lcd.setCursor(0, 1);
    lcd.print("Druecke Taster...");
    while (digitalRead(PIN_ROT_PUSH) == HIGH);
    delay(300);
    lcd.clear();
    digitalWrite(PIN_PIEZO, HIGH);
    delay(500);
    digitalWrite(PIN_PIEZO, LOW);
    lcd.print("OK");
    delay(1000);
    lcd.clear();
    
    lcd.print("Taster-Test");
    lcd.setCursor(0, 1);
    lcd.print("Druecke Taster...");
    while (digitalRead(PIN_ROT_PUSH) == HIGH);
    delay(300);
    lcd.clear();
    if (digitalRead(PIN_ROT_PUSH) == LOW) {
        lcd.print("OK");
    } else {
        lcd.print("Fehler!");
        piezoError();
    }
    delay(1000);
    lcd.clear();
    
    lcd.print("PWM mit Drehgeber");
    lcd.setCursor(0, 1);
    lcd.print("Druecke Taster...");
    while (digitalRead(PIN_ROT_PUSH) == HIGH);
    delay(300);
    lcd.clear();
    
    while (true) {
        int stateA = digitalRead(PIN_ROT_A);
        int stateB = digitalRead(PIN_ROT_B);
        
        if (stateA == HIGH && stateB == LOW) {
            pwmValue += 10;
            if (pwmValue > 255) pwmValue = 255;
        } else if (stateA == LOW && stateB == HIGH) {
            pwmValue -= 10;
            if (pwmValue < 0) pwmValue = 0;
        }
        
        analogWrite(PIN_HEAT, pwmValue);
        lcd.setCursor(0, 0);
        lcd.print("PWM: ");
        lcd.print(pwmValue);
        lcd.print("   ");
        
        if (digitalRead(PIN_ROT_PUSH) == LOW) break;
        delay(100);
    }
    lcd.clear();
    
    lcd.print("Temp-Sensor-Test");
    lcd.setCursor(0, 1);
    lcd.print("Druecke Taster...");
    while (digitalRead(PIN_ROT_PUSH) == HIGH);
    delay(300);
    lcd.clear();
    int tempValue = analogRead(PIN_TEMP);
    if (tempValue > 0) {
        lcd.print("OK: ");
        lcd.print(tempValue);
    } else {
        lcd.print("Fehler!");
        piezoError();
    }
    delay(1000);
    lcd.clear();
    
    lcd.print("ALLE TESTS OK");
    while (1);
}
