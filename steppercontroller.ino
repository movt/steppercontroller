// Dieser Sketch soll einmal erlauben, eine Fräse oder Drehmaschine via Arduino manuell über ein Bedienpult zu steuern.
// Die spätere implementation von GRBL ist noch offen, wenn möglich aber geplant. In erster Näherung wird die Z-Achse 
// einer Säulenfräse damit gesteuert. 
// Verbaute Komponenten:
// 1 x Arduino DUE
// 1 x Drehimpulsgeber
// 1 x LCD Display 20x4
// to be completed
// ...


// Benutze Bibliotheken
#include <stdlib.h>
#include <Wire.h>
#include <avr/dtostrf.h>


// Ausgangs-Pins
#define READY   13                            // Bereitschaft
#define STEP     9                            // Pin fuer Step-Impuls
#define DEBUG    1                            // 1: Debug ein, 0: Debug aus
#define DIRP    11                            // Pin fuer Richtung


// Eingangs-Pins
#define POTI    A1                            // Analogeingang fuer Poti
#define SWITCH  2                             // Geschwindigkeits-Wahlschalter
#define ENC_CLOCK 5                           // Encoder Takt
#define ENC_DATA  6                           // Encoder Signal
#define ENC_SW  4                             // Encoder Druck-Taster

// Poti-Paramter
#define Puse    90                            // nutzbarer Drehwinkel des Potis in % (max. 100)
#define Ruhe    10                            // Ruhezone in % (min. 1)
#define MITTE    0                            // Encoder Mittenstellung

#define Fmin     4                            // Minimale Schrittfrequenz - nicht kleiner als 1
#define Fmax  4000                            // Maximale Schrittfrequenz - nicht groesser als 4200

#define RFAKTOR  8                            // Reduktionsfaktor Slow-Mode

#define RICHTUNG 0                            // Richtungsumkehr: auf 0 oder 1 setzen je nach Wunsch                   

#define RELEASE  1                            // Schrittmotor "loslassen" in Ruhestellung? 0/1

// Achsparameter
float mmPerStepZ = 0.0014;                     // Wieviele Schritte macht der Motor für einen mm
int mmMaxTravelZ = 460;                        // Maximaler Verfahrweg der Z-Achse

// ab hier nichts mehr ändern
#define Pmin -20                               // Minwert vom Encoder
#define Pmax 20                                // Maxwert vom Encoder

char cBuffer[10]; // Variablen für LCD Ausgabe
char cLogValue[100];
char cDisplayValue[20];

int p, s, s_p;
long t = Pmin;
long d;
bool rightRotate;                             // Rechts rum

volatile int iEncVal = 0;                     // Encoderwert
volatile int iOldEncVal = 0;                  // Encoderwert

volatile long lCounter = 0;                   // Counter für das Timer-Callback (DEBUGGING)
volatile bool hase;                           // HIGH = Hasengang, LOW = Schneckengang
volatile int  z_pos_count;                    // Counter der Steps
volatile float pos;


void callback()
{
 
  lCounter++;
  
  if (rightRotate)
  {
    z_pos_count++;
  }
  else
  {
    z_pos_count--;
  }
}

void ISRHase()
{
  hase = digitalRead(SWITCH);
  if (hase)
    Serial.print ( "Hase gesetzt\n" );
  else
    Serial.print ( "Schnecke gesetzt\n" );
}

ISR_Hase()
{
  hase = !hase;
  if (hase)
    Serial.print ( "Hase gesetzt\n" );
  else
    Serial.print ( "Schnecke gesetzt\n" );
}

int getEncoderTurn(void)
{
  static int oldA = HIGH;
  static int oldB = HIGH;
  int result = 0;
  int newA = digitalRead(ENC_CLOCK);          //Encoder Clock nach newA
  int newB = digitalRead(ENC_DATA);           //Encoder Daten nach  newB
  if (newA != oldA || newB != oldB)
  {
    // Es wurde was geändert
    if (oldA == HIGH && newA == LOW)
    {
      result = (oldB * 2 - 1);
    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}

void setup() {
  Serial.begin ( 115200 );
  debugLog ( "Starte Programm\n" );
  interrupts ();

  
  pinMode(READY, OUTPUT);                     // Bereitschaftsanzeige
  pinMode(STEP, OUTPUT);                      // Step
  pinMode(DIRP, OUTPUT);                      // Richtungs-Pin
  pinMode(POTI, INPUT);                       // Poti Pin als INPUT
  pinMode(SWITCH, INPUT);                     // Geschwindigkeits-Wahlschalter
  pinMode(ENC_DATA, INPUT);                   // Encoder Daten
  pinMode(ENC_CLOCK, INPUT);                  // Encoder Clock
  pinMode(ENC_SW, INPUT);                     // Encoder Taster
  
  //Vorbelegungen
  digitalWrite(SWITCH, HIGH);                 // Pullup
  digitalWrite(ENC_CLOCK, HIGH);              // Pullup
  digitalWrite(ENC_DATA, HIGH);               // Pullup
  digitalWrite(ENC_SW, HIGH);                 // Pullup
  digitalWrite(READY, LOW);                   // Erstmal nicht READY

  //Interrupts
  attachInterrupt(digitalPinToInterrupt(SWITCH),ISRHase,CHANGE);          // Interrupt für den Eilgang
  attachInterrupt(digitalPinToInterrupt(ENC_SW),ISR_Hase,FALLING);          // Interrupt für den Eilgang
  
  digitalWrite(READY, HIGH);                  // Bereit
  debugLog( "Bereit!\n" );
}

/*Daten schreiben*/
void writeData() {

  digitalWrite(DIRP, rightRotate);
}

void debugLog (String sValue){
  if (DEBUG==1)
    Serial.print ( sValue );
}

/******************************** Loop ************************************/
void loop() {
  
                                 
  int change = getEncoderTurn();
  iEncVal = iEncVal + change;
  
  if(digitalRead(ENC_SW) == LOW){
      iEncVal = 0;
  }

#if RICHTUNG > 0
  rightRotate = (iEncVal < 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#else
  rightRotate = (iEncVal > 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#endif

  // Daten berechnen und Timer steuern
  if (iEncVal!=0){
    if (iEncVal != iOldEncVal){                 // Jetzt hat sich was getan
      iOldEncVal = iEncVal;                     // Wert merken
      int iAbsEncVal = abs(iEncVal);            // Wert für die Timerberechnung ohne Vorzeichen (das machen wir ja mit DIRP
      
      debugLog( dtostrf( iEncVal, 4,0, cBuffer ) );
      debugLog( "\n" );
    }
  }else{
    //Timer stoppen
  }

  // Position berechnen
  pos = 0.0014 * z_pos_count;

  // Daten schreiben
  writeData();

}
/*************************** Loop Ende ************************************/

