//Testroutine für Lötkolben Projekt
//zum Überprüfen der externen Verdrahtung und PCB Bestückung
//v4_OE9SAU_CHATGPT

#include <TinyWireM.h>
#include <LiquidCrystal_I2C.h>

#define LCDI2CADR 0x27
LiquidCrystal_I2C lcd(LCDI2CADR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define PIN_ROT_A    0
#define PIN_ROT_B    1
#define PIN_ROT_PUSH 2
#define PIN_HEAT     5
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
  lcd.print("HW TEST v3");

  pinMode(PIN_ROT_A, INPUT_PULLUP);
  pinMode(PIN_ROT_B, INPUT_PULLUP);
  pinMode(PIN_ROT_PUSH, INPUT_PULLUP);
  pinMode(PIN_STBY, INPUT_PULLUP);
  pinMode(PIN_HEAT, OUTPUT);
  digitalWrite(PIN_HEAT, LOW);
  pinMode(PIN_PIEZO, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
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
  lcd.setCursor(0, 0);
  lcd.print("HEAT Test");
  for (int i = 0; i < 5; i++) {
    lcd.setCursor(5 + i, 0);
    lcd.print(".");
    delay(600);
  }

  // 2V für 2 Sekunden
  analogWrite(PIN_HEAT, 102);  // PWM-Wert für 2V
  delay(2000);  // 2 Sekunden warten

  // 4V für 2 Sekunden
  analogWrite(PIN_HEAT, 204);  // PWM-Wert für 4V
  delay(2000);  // 2 Sekunden warten

  // HEAT ausschalten (Ausgabe auf 0V)
  analogWrite(PIN_HEAT, 0);  // PWM-Wert für 0V
  lcd.clear();
  lcd.setCursor(0, 0);
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
