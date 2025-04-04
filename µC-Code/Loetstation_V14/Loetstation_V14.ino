/* Loetstation copyright (C) 2017 OE1CGS Chris
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
   
   Firmware fuer ATtiny84
   Pin-Belegung:    Pin  Port   Nr.	    Type    Verbinden mit
                     1                  Vcc     +5V
                     2   PB0    10      DIN10   STANDBY
                     3   PB1    9               PIEZO
                    13   PA0    0       DIN0    ROTARY_A
                    12   PA1    1       DIN1    ROTARY_B
                    11   PA2    2       DIN2    ROTARY_PUSH
                    10   PA3    A3      ADC3    TEMP_SENSOR
                     7   PA6    4       SDA     I2C/SDA
                     8   PA5    5       OC1B    HEAT
                     9   PA4    6       SCL     I2C/SCL
                    14                  GND     Ground

                    
   Compiler:     ATtiny Core by David Mellis, https://github.com/damellis/attiny
   Taktfrequenz: Intern 1 MHz
   Autor:        Christoph Schwaerzler, OE1CGS
   Version:      1.2  vom 27.07.2017
   Version       1.3 vom 19.10.2024 / OE9SAU
                 *Pin-Belegung: 
                  Pin Port Nr.
                   5  PB2  8   LOGO LED
                 *Programmanpassung für LOGO LED
                 *Adresse LCD 0x3F

   Version       1.4 vom 08.11.2024 / OE9SAU
                 *Adresse LCD 0x27
                 *alt TONEINDIFF   4
*/

#include <TinyWireM.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define PIN_ROT_A    0                                // IC-Pin 13 ist der Anschluss fuer den Drehschalter-Ausgang A
#define PIN_ROT_B    1                                // IC-Pin 12 ist der Anschluss fuer den Drehschalter-Ausgang B
#define PIN_ROT_PUSH 2                                // IC-Pin 11 ist der Anschluss fuer den Drehschalter-Drucktaster
#define PIN_HEAT     5                                // IC-Pin 8 ist als OC1B PWM-faehig (Timer1, da Timer0 bereits fuer delay())
#define PIN_TEMP     A3                               // IC-Pin 10 ist der Anschluss (=ADC) fuer den Temperatursensor
#define PIN_STBY     10                               // IC-Pin 2 ist der Anschluss fuer den Standby-Schalter gegen Masse
#define PIN_PIEZO    9                                // IC-Pin 3 als Ausgang zur Ansteuerung des Piezos
#define PIN_LED      8                                // IC-Pin 5 ist der Anschluss fuer die LOGO-FLACKER-LED
#define LCDI2CADR    0x27                             // I2C-Adresse des LCD-Bausteins (typ. 0x27 oder 0x3F), I2C Scanner benutzen
#define WARTEZEIT    10                               // Wartezeit zwischen Abschaltung Heizung und Temperaturmessung in ms
#define REGELZEIT    100                              // Zeitintervall zwischen Regelprozessen in ms; muss >= 30 sein
#define A_ADC        -0.000155                        // Koeffizient des quadratrischen Terms im Zusammenhang zwischen ADC-Wert und IST-Temperatur -0.000151
#define B_ADC        0.533                            // Koeffizient des linearen Terms im Zusammenhang zwischen ADC-Wert und IST-Temperatur       0.533
#define C_ADC        37.40                            // Offset des quadratischen Zusammenhangs zwischen ADC-Wert und IST-Temperatur               37.40
#define W_AKT        0.7                              // Gewicht des aktuellen Temperaturwertes bei der laufenden Mittelwertbildung                0.7
#define P_KOEFF_P    7                                // Koeffizient des P-Reglers; VORSICHT: Darf maximal einen Wert von 30 annehmen, sonst Int-Problem im Regler
#define P_KOEFF_PID  9.0
#define I_KOEFF_PID  0.2
#define D_KOEFF_PID  19.5
#define T_MIN        50                               // Minimal erlaubte Temperatureinstellung in Grad Celsius
#define T_MAX        400                              // Maximal erlaubte Temperatureinstellung in Grad Celsius
#define PRELLUNG     10                               // Wartezeit fuer Entprellung des Tasters in Millisekunden
#define TONPEAKZEIT  70                               // Dauer einer Schwingung in Mikrosekunden
#define TONEINZEIT   1000                             // Dauer des Tons als Anzahl der Schwingungen
#define TONEINDIFF   5                                // Temperaturdifferenz SOLL/IST in Grad C bei der ein Ton ertoent
#define TONRESDIFF   10                               // Temperaturdifferenz SOLL/IST in Grad C fuer reset, um spaeter wieder zu ertoenen
#define NOTAUSDIFF   50                               // Abweichung vom letzten (sinnvollen) Wert, ab welcher ein Wert als ungueltig betrachtet wird
#define NOTAUSANZ    10                               // Anzahl der direkt aufeinanderfolgenden, ungueltigen Werte, bevor NOTAUS erfolgt

LiquidCrystal_I2C lcd(LCDI2CADR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Instanz "lcd" erzeugen und LCD-Anzeige initialisieren

byte          curpos  =  1;                           // Position des Cursors; Wertemenge = {1,2,3,4,5}
byte          unguelt =  0;                           // Anzahl der aufeinanderfolgenden, ungueltigen Temperaturwerte
int           a_tist  =  0; int   l_tist  = 0;        // Aktuelle (a_tist)  bzw. letzte (l_tist)  IST  - Temperatur in Grad Celsius
int           a_tsoll = 60; int   l_tsoll = 0;        // Aktuelle (a_tsoll) bzw. letzte (l_tsoll) SOLL - Temperatur in Grad Celsius
int           anz_tist;     int   lanz_tist;          // Geglaettet, angezeigte aktuelle (anz_tist)  bzw. letzte (lanz_tist)  IST  - Temperatur
int           lr_tist;                                // Letzte Temperatur fuer PID-Regler;
int           tstby   = 60;                           // Standby-Temperatur in Grad Celsius
int           tset1;                                  // SET 1 - Temperatur in Grad Celsius
int           tset2;                                  // SET 2 - Temperatur in Grad Celsius
int           tset3;                                  // SET 3 - Temperatur in Grad Celsius
int           pwm     =   0;                          // Aktueller Heizwert fuer PWM-Register; Wertebereich = [0,1023]
int           anzdreh =   0;                          // Anzahl Drehschritte nach rechts (+) oder links (-) seit letzter Abfrage
boolean       standby =   false;                      // Zwischenspeicher fuer den a_tsoll Wert, solange im Standby-Modus
boolean       ton     =   true;                       // Zeigt an, ob Tonsignal ein (=true) oder ausgeschalten ist
boolean       set_ton =   false;                      // War schon einmal ein Ton, bevor er zurueckgesetzt wurde?
boolean       lnotaus =   false;                      // War beim letzten Reglerdurchgang auch schon ein Problem?
unsigned long lzp_regler = 0;                         // Letzter Zeitpunkt zu dem der Regler nachgestellt wurde
volatile byte dstatus= 0;                             // Letzter Zustand des Drehschalters
const byte    dtabelle[7][4] = {                      // Dies ist die Zustandstabelle fuer den Drehschalter
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},       // Kredit geht dafuer an Ben Buxton 2011 mit seiner "Rotary" library
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
  {0x6, 0x5, 0x4,  0x0}};
byte backchar[8]={                                    // Beschreibung des selbstdefinierten Symbols "Zurueck"
  0x04,0x0E,0x1F,0x04,0x04,0x1C,0x00,0x00};
byte stbychar[8]={                                    // Beschreibung des selbstdefinierten Symbols "Standby"
  0x00,0x0E,0x15,0x17,0x11,0x0E,0x00,0x00};
byte settchar[8]={                                    // Beschreibung des selbstdefinierten Symbols "Settemp"
  0x00,0x04,0x06,0x1F,0x1F,0x06,0x04,0x00};
byte tonschar[8]={                                    // Beschreibung des selbstdefinierten Symbols "Ton"
  0x01,0x03,0x0f,0x0f,0x0f,0x03,0x01,0x00};
byte wellchar[8]={                                    // Beschreibung des selbstdefinierten Symbols "Wellen"
  0x04,0x08,0x10,0x1c,0x10,0x08,0x04,0x00};


void setup(){
  OSCCAL = 0x83;                                      // Kalibrierung des internen Oszillators, z.B. TinyTuner benutzen
  pinMode(PIN_ROT_A,    INPUT_PULLUP);                // IC-Pin 13 als Eingang mit Pullup-Widerstand definieren (Drehschalter A)
  pinMode(PIN_ROT_B,    INPUT_PULLUP);                // IC-Pin 12 als Eingang mit Pullup-Widerstand definieren (Drehschalter B)
  pinMode(PIN_ROT_PUSH, INPUT_PULLUP);                // IC-Pin 11 als Eingang mit Pullup-Widerstand definieren (Drehschalter Taster)
  pinMode(PIN_STBY,     INPUT_PULLUP);                // IC-Pin 2 als Eingang mit Pullup-Widerstand definieren (optionaler Standby-Taster)
  pinMode(PIN_HEAT,     OUTPUT);                      // IC-Pin 8 als Ausgang zur Schaltung der Heizung (PWM)
  pinMode(PIN_PIEZO,    OUTPUT);                      // IC-Pin 3 als Ausgang zur Ansteuerung des Piezos
  pinMode(PIN_LED,      OUTPUT);                      // IC-Pin 5 ist der Anschluss fuer die LOGO-FLACKER-LED
  eepromRead();                                       // Holt die SET-Temperaturen aus dem EEPROM
  if(tset1 == -1){                                    // Beim erstmaligen Aufruf des Programms ist das EEPROM noch leer,
    tset1 = 220;                                      // daher werden die SET-Temperaturen und Ton auf Standardwerte gesetzt
    tset2 = 260;                                      // hier sind die Standardtemperaturen fuer Sn62Pb36Ag2 mit Ts = 179 Grad C optimiert
    tset3 = 300;
    ton   = true;}
  TCCR1A = 0b00100011;                                // 10-bit PWM => 1023 als Maximalwert (=100% Heizung), toggle OC1B
  TCCR1B = 0b00000001;                                // Timer 1 ein, kein Prescaler => 1 MHz -> 496 Hz
  OCR1B  = 0;                                         // Heizung aus (= Duty Cycle 0%)
  GIMSK  |= 0b00010000;                               // Setzt PCIE0 bit (Pin Change Interrupt Enable 0) fuer Drehschalter Interrupt
  PCMSK0 |= 0b00000011;                               // Erlaubt PCINT0 (=Pin 13) und PCINT1 (=Pin 12) einen Pin-Change Interrupt
  sei();                                              // Erlaubt Interrupts
  lcd.begin(16,2);                                    // Starte LCD-Anzeige mit 16 Zeichen in 2 Zeilen
  lcd.createChar(1, backchar);                        // Erzeuge das Symbol "Zurueck" und weise ihm den Code 1 zu
  lcd.createChar(2, stbychar);                        // Erzeuge das Symbol "Standby" und weise ihm den Code 2 zu
  lcd.createChar(3, settchar);                        // Erzeuge das Symbol "Settemp" und weise ihm den Code 3 zu
  lcd.createChar(4, tonschar);                        // Erzeuge das Symbol "Ton"     und weise ihm den Code 4 zu
  lcd.createChar(5, wellchar);                        // Erzeuge das Symbol "Wellen"  und weise ihm den Code 5 zu
  lcd.clear();                                        // Loesche LCD-Anzeige
  lr_tist = abfrageTemperatur();                      // Letzten Temperaturwert auf aktuelle Temperatur setzen
  anz_tist = lr_tist;                                 // Startwert fuer die laufende Mittelwertbildung
  if(!digitalRead(PIN_ROT_PUSH)){schirmSetup();}      // Bei gedruecktem Taster zur Einstellung der Standardwerte
  else{
    lcd.print("SolderUnit V1.4");                     // Begruessungstext 1. Zeile
    lcd.setCursor(0,1);                               // Setze Cursor zum Beginn der zweiten Zeile
    lcd.print("by OE1CGS/OE9SAU");                    // Begruessungstext 2. Zeile
    delay(1500);}                                     // Begruessungstext 1,5 Sekunden lang anzeigen
  eepromUpdate();                                     // Schreibt ggf. geanderte SET-Temperaturen ins EEPROM
  schirmStdinit();                                    // Maske fuer den Standardschirm
}


void loop(){
  reglerTemperatur();                                 // Regelkreis fuer die Loetspitze
  anzeigeTemperatur();                                // Aktualisiert die Temperaturwerte auf der Anzeige
  aendereTemperatur();                                // Aendert ggf. Solltemperatur
  bewegeCursor();                                     // Aendert die Position des Cursors bei Betaetigung des Drehschalters
  waehleSettemperatur();                              // Aendert ggf. Solltemperatur auf eine SET-Temperatur
  standbyAbfrage();                                   // Prueft, ob Standby-Modus aktiviert ist
  logoflackerled();                                   // LOGO-FLACKER-LED
}

void logoflackerled(){
  if(pwm > 10){ 
  analogWrite(8, random(100)+235);
  delay(random(400));
  }
}

void schirmStdinit(){                                 // Maske fuer den Standardbildschirm
  lcd.clear();                                        // Sie wird nur einmalig beim Hochstarten erzeugt; in weiterer Folge
  lcdPrint3d(anz_tist);lanz_tist = anz_tist;          // werden nur die aktuelle Temperatur sowie gegebenenfalls die Solltemperatur
  lcd.print("\337C     ");                            // und der Cursor aktualisiert
  lcd.print(char(3));                                 // Zeige das selbsterstellte Symbol fuer "Settemp" an
  lcdPrint3d(a_tsoll);l_tsoll = a_tsoll;
  lcd.print("\337C");                                 // Der Code \337 erzeugt das "Grad" Symbol
  lcd.setCursor(0,1);
  lcdPrint3d(tset1);
  lcd.print("\337  ");
  lcdPrint3d(tset2);
  lcd.print("\337 ");
  lcdPrint3d(tset3);
  lcd.print("\337C");
  curpos = 1;                                         // setze den Cursor fuer den Hauptschirm auf die Settemperatur
  lcd.setCursor(13,0);
  lcd.cursor();                                       // Zeigt den Cursor an
}


void anzeigeTemperatur(){                             // Standardschirm: In dieser Funktion werden nur die aktuelle Temperaturanzeige
  anz_tist = W_AKT * a_tist + (1-W_AKT) * anz_tist;   // Anzeigewert ist der laufende, gewichtete Mittelwert
  if(anz_tist != lanz_tist){                          //   sowie gegebenenfalls die Solltemperatur und die Cursorposition/blinken
    lcd.setCursor(0,0);                               //   aktualisiert.
    lcdPrint3d(anz_tist);lanz_tist = anz_tist;        // Eine Aktualisierung erfolgt nur einmalig, wenn sich der Wert veraendert,
    setCurpos();                                      // Setzt den Cursor wieder an die richtige Stelle
    if(abs(a_tsoll-anz_tist) <= TONEINDIFF && !set_ton){// Wenn sich die Temperatur geaendert hat, koennte es auch sein, dass der Pizeo laeuten soll
      if(ton){ringPiezo(TONPEAKZEIT, TONEINZEIT);}    // ...aber nur, wenn Tonsignal eingeschalten ist
      set_ton = true;                                 // Jetzt soll es aber vorerst nicht mehr laeuten
    }
    if(abs(a_tsoll-anz_tist) >= TONRESDIFF){          // Erst wenn die Differenz wieder gross wurde (z.B. bei einem Standby-Betrieb) soll es wieder laeuten
      set_ton = false;
    }
  }
  if(a_tsoll != l_tsoll){
    lcd.setCursor(11,0);
    lcdPrint3d(a_tsoll);l_tsoll = a_tsoll;
    setCurpos();                                      // Setzt den Cursor wieder an die richtige Stelle
  }
}


void aendereTemperatur(){                             // Funktion zur Aenderung der aktuellen Solltemperatur bei Druck auf Drehschalter und folgender Drehung
  if(digitalRead(PIN_ROT_PUSH) || curpos > 1){return;}// Wenn der Taster nicht gedrueckt wird, oder auf SET-Wert steht sind wir hier schon fertig
  warten(PRELLUNG);                                   // Entprellung des Tasters
  while(!digitalRead(PIN_ROT_PUSH)){
    reglerTemperatur();                               //   ...aber die wichtigen Prozesse der Temperaturregelung und
    anzeigeTemperatur();                              //   ...der Anzeige der korrekten Temperaturen und
    standbyAbfrage();                                 //   ...der Check, ob Standby aktiv ist muessen fortgesetzt werden
    if(anzdreh != 0){
      a_tsoll = a_tsoll + anzdreh;                    // Anpassung von a_tsoll um die Drehbewegung des Drehschalters (1 Raster = 1 Grad Celsius)
      anzdreh = 0;                                    // Drehungswert zurueck setzen
      a_tsoll = min(max(a_tsoll,T_MIN),T_MAX);        // Schraenkt den Wertebereich der Solltemperatur auf T_MIN bis T_MAX ein
    }
  }
  warten(PRELLUNG);                                   // Nachdem der Taster losgelassen wurde, muss noch die Entprellung abgewartet werden
}


void bewegeCursor(){                                  // Aendert die Cursorposition, wenn der Drehschalter gedreht wird
  if(anzdreh != 0){                                   // Code nur ausfuehren, wenn der Drehschalter betaetigt wurde
  curpos = curpos + anzdreh;                          // Cursorposition um Bewegung des Drehschalters veraendern
    anzdreh = 0;                                      // Drehungswert zurueck stellen
    curpos = min(max(curpos,1),4);                    // Schraenkt den Wertebereich von curpos auf 1 bis 4 ein
    setCurpos();                                      // Setzt den Cursor wieder an die richtige Stelle
  }
}


void waehleSettemperatur(){                           // Macht die gewaehlte SET-Temperatur zur Solltemperatur
  if(digitalRead(PIN_ROT_PUSH) || curpos == 1){return;}       // Code wird nur ausgefuehrt, wenn Drehtaster gedrueckt wird waehrend Cursor auf einer Set-Temperatur
    switch(curpos){                                   // Ermittelt die Settemperatur auf der Anzeige abhaengig vom Wert von curpos
      case 1: break;                                  //    und weist diese der Solltemperatur zu
      case 2: a_tsoll = tset1; break;
      case 3: a_tsoll = tset2; break;
      case 4: a_tsoll = tset3; break;}
  warten(PRELLUNG);                                   // Warten auf Entprellung des Tasters
  while(!digitalRead(PIN_ROT_PUSH)){                  // Taster muss losgelassen werden bevor weitere Aktionen erfolgen koennen
    reglerTemperatur();                               //   ...aber die wichtigen Prozesse der Temperaturregelung und
    anzeigeTemperatur();                              //   ...der Anzeige der korrekten Temperatur und
    standbyAbfrage();}                                //   ...der Check, ob Standby aktiv ist muessen fortgesetzt werden
  warten(PRELLUNG);                                   // Warten auf Entprellung des Tasters
  curpos = 1;                                         // Nach Abschluss wird der Cursor wieder auf die Solltemperatur gestellt
  lcd.setCursor(13,0);                                //    und dort angezeigt
}


void schirmSetup(){                                   // Bildschirm zur Einstellung der Standardwerte; wird nur bei gedrueckte Taste aufgerufen
  lcd.print("Standard:  ");
  lcd.print(char(4));                                 // Stellt das Symbol "Ton" dar
  if(ton){lcd.print(char(5));}                        // Wenn Ton ein, stelle auch das Symbol "Wellen" dar
  else{lcd.print(" ");}                               //   sonst stelle nur ein Leerzeichen dar
  lcd.print("  ");
  lcd.print(char(1));                                 // Stellt das Symbol "Zurueck" dar
  lcd.setCursor(0,1);
  lcdPrint3d(tset1);
  lcd.print("\337  ");
  lcdPrint3d(tset2);
  lcd.print("\337 ");
  lcdPrint3d(tset3);
  lcd.print("\337C");
  lcd.cursor();                                       // Stellt den Cursor dar
  warten(PRELLUNG);                                   // Warten auf Entprellung des Tasters
  while(!digitalRead(PIN_ROT_PUSH))                   // Taster muss losgelassen werden bevor weitere Aktionen erfolgen koennen
  warten(PRELLUNG);                                   // Warten auf Entprellung des Tasters
  while (digitalRead(PIN_ROT_PUSH) || curpos != 2){   // Erst wenn Cursor auf "Ende" steht und Taster gedrueckt wird zurueck
    curpos = curpos + anzdreh;                        // Cursorposition um Bewegung des Drehschalters veraendern
    anzdreh = 0;                                      // Drehungswert zurueck stellen
    curpos = min(max(curpos,1),5);                    // Schraenkt den Wertebereich von curpos auf 1 bis 4 ein
    switch(curpos){                                   // Ermittelt die Cursorposition auf der Anzeige abhaengig vom Wert von curpos
      case 1:  lcd.setCursor(12,0); break;                             
      case 2:  lcd.setCursor(15,0); break;
      case 3:  lcd.setCursor( 2,1); break;
      case 4:  lcd.setCursor( 8,1); break;
      case 5:  lcd.setCursor(13,1); break;}
    while(!digitalRead(PIN_ROT_PUSH) && curpos != 2){
      warten(PRELLUNG);  
      if(anzdreh != 0){
        switch(curpos){
          case 1: ton = !ton;   lcd.setCursor(12,0);
                  if(ton){lcd.print(char(5));}
                  else{lcd.print(" ");}
                  lcd.setCursor(12,0); break;
          case 3: lcd.setCursor( 0,1);                 // Aendert den angezeigten Wert der soeben adjustierten SET-Temperatur
            tset1 = min(max(tset1+anzdreh,T_MIN),T_MAX);
            lcdPrint3d(tset1);   lcd.setCursor( 2,1);
            break;
          case 4: lcd.setCursor( 6,1);
            tset2 = min(max(tset2+anzdreh,T_MIN),T_MAX);
            lcdPrint3d(tset2);   lcd.setCursor( 8,1);
            break;
          case 5: lcd.setCursor(11,1);
            tset3 = min(max(tset3+anzdreh,T_MIN),T_MAX);
            lcdPrint3d(tset3);   lcd.setCursor(13,1);
            break;}
        anzdreh = 0;                                   // Drehungswert zurueck setzen
        }
     }
     warten(PRELLUNG);
   }
}


ISR(PCINT0_vect) {                                     // Interrupt Service Routine fuer Change-Interrupt an Pin 12 oder Pin 13 (Drehschalter)
  byte drehung = drehCheck();                          // Aufruf der Zustandsmaschine fuer die Abfrage des Drehschalters
  if(drehung == 0x10){anzdreh++;}                      // Wenn Linksdrehung:  Variable "Drehung" verringern
  if(drehung == 0x20){anzdreh--;}                      // Wenn Rechtsdrehung: Variable "Drehung" erhoehen
}


byte drehCheck() {                                     // Funktion zur Abfrage des Drehschalters auf Drehung
  byte pstatus = (digitalRead(PIN_ROT_B) << 1) | digitalRead(PIN_ROT_A);     // Gibt 0 zurueck, wenn keine Drehung erfolgt ist
  dstatus = dtabelle[dstatus & 0xf][pstatus];                                // Gibt 0x10 bei Linksdrehung zurueck
  return (dstatus & 0x30);                                                   // Gibt 0x20 bei Rechtsdrehung zurueck
}


int abfrageTemperatur(){                               // Liest die aktuelle Temperatur aus dem ADC, wandelt in Grad Celsius um und gibt Wert zurueck
  OCR1B  = 0;                                          // Heizung aus (= Duty Cycle 0%); Dies ist erforderlich, da sonst falsche Spannung an ADC liegt 
  delay(WARTEZEIT);                                    // Warten, um Tiefpassfilter Zeit zu geben
  int tempval = analogRead(PIN_TEMP);                  // Abfrage des ADC-Wertes des Temperaturfuehlers
  if(tempval > 1000){notAus();}                        // Bei extrem hoher Temperatur oder ohne Sensor -> NOT-AUS
  if(millis() > 4000 && tempval == 0){notAus();}       // Wenn bei laufendem Betrieb immer noch keine Temperatur gemeldet wird -> NOT-AUS
  OCR1B  = pwm;                                        // Heizung wieder auf den aktuellen Wert einschalten
  return A_ADC*tempval*tempval + B_ADC*tempval + C_ADC + 0.5;      // Umwandlung des ADC-Wertes in Temperaturwert [Grad Celsius]
}


void reglerTemperatur(){
  if(millis() < lzp_regler + REGELZEIT){return;}       // Wenn die REGELZEIT noch nicht abgelaufen ist, keine neue Regelung
  a_tist = abfrageTemperatur();                        // Holt die aktuelle Temperatur der Loetspitze in Grad Celsius
  if(abs(a_tist - lr_tist) > NOTAUSDIFF){              // Wenn wiederholt erratische Temperaturwerte empfangen werden -> NOT-AUS
    a_tist = lr_tist;
    unguelt = unguelt + 1;
    if(unguelt > NOTAUSANZ){notAus();}
  }
  else{unguelt = 0;}
  if(standby){                                         // Wenn im Standby-Modus,
    pwm = P_KOEFF_P * (tstby - a_tist);}               // berechne den neuen PWM-Wert mit P-Regler gegenueber Standby-Temperatur,
  else{                                                // sonst,
  pwm = P_KOEFF_PID * (a_tsoll - a_tist) + I_KOEFF_PID * pwm;
  pwm = pwm - D_KOEFF_PID * (a_tist - lr_tist);}       // PID-Regler
  pwm = int(min(max(pwm,0),1023));                     // Schraenkt den Wertebereich von pwm auf 0 bis 1023 ein
  OCR1B  = pwm;                                        // Stellt die Heizleistung (Wert/1023) ein
  lr_tist = a_tist;                                    // Letzte Temperatur ist gleich aktuelle Temperatur
  lzp_regler = millis();                               // Referenzzeit fuer naechste Regelung speichern
}


void standbyAbfrage(){                                 // Prueft, ob der Loetkolben in der Halterung steck. Wenn erstmalig ja, wird der Standby-Modus aktiviert
  if(!digitalRead(PIN_STBY) && !standby){              // Wenn Schalter erstmalig betaetigt...
    standby = true;                                    //   Standby-Modus aktiviert
    lcd.noCursor();
    lcd.setCursor(7,0);
    lcd.print(char(2));
    setCurpos();                                       // Setzt den Cursor wieder an die richtige Stelle
    lcd.cursor();}                                     // Anzeige des "Standby"-Symbols
  else if(digitalRead(PIN_STBY) && standby){           // Wenn Schalter erstmalig nicht mehr betaetigt...
    standby = false;                                   // Standby-Modus deaktiviert
    lcd.noCursor();
    lcd.setCursor(7,0);
    lcd.print(" ");                                    // Loeschen des "Standby"-Symbols
    setCurpos();                                       // Setzt den Cursor wieder an die richtige Stelle
    lcd.cursor();}                                     //   Anzeige des "Standby"-Symbols
}


void lcdPrint3d(int wert){                             // Stellt 3-stellige Integer-Zahlen immer rechtsbuendig dar
  if(wert <  10){lcd.print(" ");}                      // Fuehrende Nullen werden als Leerzeichen dargestellt
  if(wert < 100){lcd.print(" ");}                      //   und damit eventuell vorhandener Alttext geloescht
  lcd.print(wert);
}


void eepromUpdate(){                                   // Schreibt die SET-Temperaturen in EEPROM (aber nur, wenn sie geandert wurden)
  byte tlow; byte thigh;                               // Dazu muessen die Integer-Werte in jeweils zwei Byte zerlegt werden
  tlow  = tset1; thigh = (tset1 >> 8);
  EEPROM.update(0,tlow); EEPROM.update(1,thigh);       // Diese 3*2 = 6 Bytes werden in die EEPROM Speicher 0 bis 5 abgelegt      
  tlow  = tset2; thigh = (tset2 >> 8);
  EEPROM.update(2,tlow); EEPROM.update(3,thigh);
  tlow  = tset3; thigh = (tset3 >> 8);
  EEPROM.update(4,tlow); EEPROM.update(5,thigh);
  EEPROM.update(6,ton);
}   


void eepromRead(){                                     // Liest die SET-Temperaturen aus dem EEPROM
  byte tlow; byte thigh;                               // Dazu werden die Integer-Werte aus jeweils zwei Bytes zusammengesetzt
  tlow   = EEPROM.read(0); thigh = EEPROM.read(1);     // vrgl. eeprom_update()
  tset1  = ((thigh << 8) | tlow);
  tlow   = EEPROM.read(2); thigh = EEPROM.read(3);
  tset2  = ((thigh << 8) | tlow);
  tlow   = EEPROM.read(4); thigh = EEPROM.read(5);
  tset3  = ((thigh << 8) | tlow);
  ton    = EEPROM.read(6);
}


void setCurpos(){
    switch(curpos){                                    // Ermittelt die Cursorposition auf der Anzeige abhaengig vom Wert von curpos
      case 1:  lcd.setCursor(13,0); break;             //    und setzt den Cursor an die richtige Position
      case 2:  lcd.setCursor( 2,1); break;
      case 3:  lcd.setCursor( 8,1); break;
      case 4:  lcd.setCursor(13,1); break;}
}


void ringPiezo(int peakzeit, int einzeit){             // Laesst den Piezo-Buzzer kurz mit 4200 Hz ertoenen
  for(int i = 1; i < einzeit; i++){
    digitalWrite(PIN_PIEZO, HIGH);
    delayMicroseconds(peakzeit);
    digitalWrite(PIN_PIEZO, LOW);
    delayMicroseconds(peakzeit - 30);                  // Der Wert von 30 kompensiert die Laufzeit fuer die Schleife, damit t(LOW) = t(HIGH)
  }
}


void warten(int msekunden){                            // Anstelle eines einfachen "delay" verwendet, um weiterhin den Regler aufzurufen
  unsigned long startzeit = millis();
  while(millis() < startzeit + msekunden){
      reglerTemperatur();                              // Das Einzige, was waehrend der Wartezeit passiert: Regelkreis fuer die Loetspitze aufrufen
  }
}

void notAus(){                                         // Not-Aus Routine, wird bei ungew�hnlichem Verhalten aufgerufen und schaltet Loetspitze aus
  OCR1B  = 0;                                          // Schaltet die Heizung aus
  lcd.clear();                                         // LCD-Anzeige loeschen
  lcd.noCursor();                                      // Keinen Cursor auf der LCD-Anzeige darstellen
  lcd.print("    WARNUNG");                            // Notaustext 1. Zeile
  lcd.setCursor(0,1);                                  // Setze Cursor zum Beginn der zweiten Zeile
  lcd.print("   L\357tspitze");                        // Notaustext 2. Zeile
  for(int i = 1; i < 5; i++){                          // Warnton als Folge von 5 kurzen Toenen, unabhaengig von gewaehlter Toneinstellung (ein/aus)
    ringPiezo(TONPEAKZEIT, TONEINZEIT);
    delay(100);
  }
  while(true);                                         // Endlosschleife
}
