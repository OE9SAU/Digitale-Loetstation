// Testroutine für Lötkolben Projekt
// zum Überprüfen der externen Verdrahtung und PCB Bestückung
// v1_OE9SAU_CHATGPT

// Testroutine für Lötkolben Projekt
// zum Überprüfen der externen Verdrahtung und PCB Bestückung
// v5_OE9SAU_CHATGPT

#include <TinyWireM.h>
#include <LiquidCrystal_I2C.h>

#define LCDI2CADR 0x27
LiquidCrystal_I2C lcd(LCDI2CADR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define PIN_ROT_A    0
#define PIN_ROT_B    1
#define PIN_ROT_PUSH 2
#define PIN_HEAT     5  // Pin für HEAT (PWM)
#define PIN_TEMP     A3
#define PIN_STBY     10
#define PIN_PIEZO    9
#define PIN_LED      8

unsigned long standbyStart = 0;
bool inStandby = false;
int lastRotA = HIGH;

void setup() {
  lcd.begin(16, 2);
  lcd.setBacklight(true);
  lcd.setCursor(0, 0);
  lcd.print("HW TEST v1");

  pinMode(PIN_ROT_A, INPUT_PULLUP);
  pinMode(PIN_ROT_B, INPUT_PULLUP);
  pinMode(PIN_ROT_PUSH, INPUT_PULLUP);
  pinMode(PIN_STBY, INPUT_PULLUP);
  pinMode(PIN_HEAT, OUTPUT);  // Pin HEAT als Ausgang setzen
  pinMode(PIN_PIEZO, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  TCCR1A = 0b00100011;                                // 10-bit PWM => 1023 als Maximalwert (=100% Heizung), toggle OC1B
  TCCR1B = 0b00000001;                                // Timer 1 ein, kein Prescaler => 1 MHz -> 496 Hz
  OCR1B  = 0;                                         // Heizung aus (= Duty Cycle 0%)

  delay(2000);
  lcd.clear();
}

void loop() {
  waitForButtonPress();
  standbyMode();
  
  // Piezo-Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Piezo Test");
  playTone(1000, 500);
  delay(500);
  waitForButtonPress();

  // HEAT-Test mit analoger Spannungsausgabe
  lcd.clear();
  lcd.print("HEAT Test");

  OCR1B = 100;  
  lcd.print(".");  // Erster Punkt direkt hinter "HEAT Test"
  delay(2500);   

  OCR1B = 250;  
  lcd.print(".");  // Zweiter Punkt
  delay(2500);   

  OCR1B = 500;  
  lcd.print(".");  // Dritter Punkt
  delay(2500);   

  // HEAT ausschalten (Ausgabe auf 0V)
  OCR1B = 0;  
  lcd.clear();
  lcd.print("HEAT OFF");
  waitForButtonPress();

  // Encoder-Test
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Encoder Test");
  while (true) {
    int currentRotA = digitalRead(PIN_ROT_A);
    if (currentRotA != lastRotA && currentRotA == LOW) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Encoder Test");
      if (digitalRead(PIN_ROT_B) == LOW) {
        lcd.setCursor(0, 1);
        lcd.print("Dreht RECHTS \76");
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Dreht LINKS  \74");
      }
      delay(500);
    }
    lastRotA = currentRotA;

    if (digitalRead(PIN_ROT_PUSH) == LOW) {  // Bestätigung durch Knopfdruck
      while (digitalRead(PIN_ROT_PUSH) == LOW);  // Warten, bis losgelassen wird
      delay(100);
      break;
    }
  }

  // Test abgeschlossen, Text ausgeben
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Test Ende");
  while (true);  // Halte den Code an, um das Display anzuzeigen
}

void standbyMode() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Standby Test");
  
  // Zeige den aktuellen Zustand des Standby-Eingangs
  lcd.setCursor(0, 1);
  lcd.print("STBY: " + String(digitalRead(PIN_STBY)));
  
  // LED folgt dem Zustand des Standby-Eingangs
  if (digitalRead(PIN_STBY) == LOW) {
    digitalWrite(PIN_LED, HIGH);  // LED ein, wenn STBY LOW
  } else {
    digitalWrite(PIN_LED, LOW);   // LED aus, wenn STBY HIGH
  }

  standbyStart = millis();
  inStandby = true;

  while (true) {
    // Aktualisiere den Zustand des Standby-Eingangs
    lcd.setCursor(6, 1);
    lcd.print(String(digitalRead(PIN_STBY)) + "  ");
    
    // LED folgt weiterhin dem Zustand des Standby-Eingangs
    if (digitalRead(PIN_STBY) == LOW) {
      digitalWrite(PIN_LED, HIGH);
    } else {
      digitalWrite(PIN_LED, LOW);
    }

    // Warten auf Bestätigung durch Knopfdruck
    if (digitalRead(PIN_ROT_PUSH) == LOW) { 
      while (digitalRead(PIN_ROT_PUSH) == LOW); 
      delay(100);
      break;
    }
  }
  
   inStandby = false;
}

void playTone(int frequency, int duration) {
  int period = 1000000 / frequency;
  int halfPeriod = period / 2;
  unsigned long startMillis = millis();
  while (millis() - startMillis < duration) {
    digitalWrite(PIN_PIEZO, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(PIN_PIEZO, LOW);
    delayMicroseconds(halfPeriod);
  }
}

void waitForButtonPress() {
  lcd.setCursor(0, 1);
  lcd.print("Druecke Knopf...");
  while (digitalRead(PIN_ROT_PUSH) == HIGH);
  while (digitalRead(PIN_ROT_PUSH) == LOW);
  delay(100);
}
