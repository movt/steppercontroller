

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
#include <avr/dtostrf.h>                      // Formatierungs Bibliothek (Standard)
#include <DueTimer.h>                         // DueTimer von Ivan Seidel

#define DEBUG    1                            // 1: Debug ein, 0: Debug aus


// Ausgangs-Pins
#define READY   13                            // Bereitschaft
#define ZSTEP    9                            // Pin fuer Step-Impuls
#define ZDIRP   11                            // Pin fuer Richtung
#define SLED     8                            // LED für den Eilgang

// Eingangs-Pins
#define POTI     A1                           // Analogeingang fuer Poti
#define SWITCH    2                           // Geschwindigkeits-Wahlschalter
#define ENC_CLOCK 5                           // Encoder Takt
#define ENC_DATA  6                           // Encoder Signal
#define ENC_SW    4                           // Encoder Druck-Taster

// Poti-Paramter
#define MITTE     0                           // Encoder Mittenstellung
#define ESTMIN    0                           // Minwert vom Encoder
#define ESTMAX  400                           // Maxwert vom Encoder je höher desto feinfühliger ist die Steuerung

// Allegemeine Parameter
#define Fmin      4                           // Minimale Schrittfrequenz - nicht kleiner als 1
#define Fmax   4000                           // Maximale Schrittfrequenz - nicht groesser als 4200

#define RFAKTOR   8                           // Reduktionsfaktor Slow-Mode

#define RICHTUNG  0                           // Richtungsumkehr: auf 0 oder 1 setzen je nach Wunsch                   

#define RELEASE   1                           // Schrittmotor "loslassen" in Ruhestellung? 0/1

// Achsparameter
float mmPerStepZ  = 0.0014;                   // Wieviele Schritte macht der Motor für einen mm
int mmMaxTravelZ  = 460;                      // Maximaler Verfahrweg der Z-Achse

// Rampen
#define SLOWZONE_Z  10                        // Abbrems-Start zum Endpunkt in mm
#define ACC_ZONE_Z  10                        // Beschleunigungsstrecke bis Max in mm       

// ab hier nichts mehr ändern

char cBuffer[10], cLogValue[100], cDisplayValue[20]; // Variablen für  Ausgabe

long t = ESTMIN;                              // Anfänglicher Takt
bool rightRotate;                             // Rechts rum

volatile int iEncVal = 0;                     // Encoderwert
volatile int iOldEncVal = 0;                  // Encoderwert

volatile long lCounter = 0;                   // Counter für das Timer-Callback (DEBUGGING)
volatile bool hase;                           // HIGH = Hasengang, LOW = Schneckengang
volatile int  z_pos_count;                    // Counter der Steps
volatile float pos;                           // Positionszähler



int getEncoderTurn( void )
{
  static int oldA = HIGH;
  static int oldB = HIGH;
  int result = 0;
  int newA = digitalRead( ENC_CLOCK );          //Encoder Clock nach newA
  int newB = digitalRead( ENC_DATA );           //Encoder Daten nach newB
  if (newA != oldA || newB != oldB){          // Es wurde was geändert
    
    if ( oldA == HIGH && newA == LOW ){
      result = ( oldB * 2 - 1 );
    }
  }
  oldA = newA;
  oldB = newB;
  return result;
}
void setup( void ) {
  Serial.begin ( 115200 );
  debugLog ( "Starte Programm\n" );
  noInterrupts();
  interrupts ();

  
  pinMode( READY, OUTPUT );                     // Bereitschaftsanzeige
  pinMode( ZSTEP, OUTPUT );                     // Z Achsen Step
  pinMode( ZDIRP, OUTPUT );                     // Z Achsen Richtungs-Pin
  pinMode( POTI, INPUT );                       // Poti Pin als INPUT
  pinMode( SWITCH, INPUT );                     // Geschwindigkeits-Wahlschalter
  pinMode( ENC_DATA, INPUT_PULLUP );            // Encoder Daten
  pinMode( ENC_CLOCK, INPUT_PULLUP );           // Encoder Clock
  pinMode( ENC_SW, INPUT );                     // Encoder Taster
  pinMode( SLED, OUTPUT );                      // Eilgang LED
  //Vorbelegungen
  digitalWrite( SWITCH, HIGH );                 // Pullup
  digitalWrite( ENC_CLOCK, HIGH );              // Pullup
  digitalWrite( ENC_DATA, HIGH );               // Pullup
  digitalWrite( ENC_SW, HIGH );                 // Pullup
  digitalWrite( READY, LOW );                   // Erstmal nicht READY
  
  Timer0.attachInterrupt( zTimer );             // Timer0 Interrupt für die Z-Achse
  
  
  digitalWrite( READY, HIGH );                  // Bereit

  hase = LOW;
  
  debugLog( "Bereit!\n" );
}

/*Z-Achsen Timer*/
void zTimer( void ){
  int step=!digitalRead( ZSTEP );
  digitalWrite( ZSTEP,step );
  
  if ( rightRotate ){
    z_pos_count++;
  }else{
    z_pos_count--;
  }
}

/*Daten schreiben*/
void writeData( void ) {
  
  digitalWrite( ZDIRP, rightRotate );

//  debugLog( "Z-Pos: " );
//  debugLog( dtostrf( pos, 8, 3, cBuffer ));
//  debugLog( "\n" );
  
}

void debugLog ( String sValue ){
  if ( DEBUG==1 )
    Serial.print ( sValue );
}

/******************************** Loop ************************************/
void loop( void ) {
  
                                 
  int change = getEncoderTurn();                // Encoderwert holen
  iEncVal = iEncVal + change;                   // Verstellwert mit Istwert abgleichen
  int iAbsEncVal = abs( iEncVal );              // Wert für die Timerberechnung ohne Vorzeichen (das machen wir ja mit ZDIRP
  
  if(digitalRead( ENC_SW ) == LOW ){            // Wenn der Taster gedrückt wird
    if( iAbsEncVal != 0 ){                      // un der aktuelle Wert ist ungleich 0
        iEncVal = 0;                            // wird der Speed genullt und der Stepper sollte sofort stoppen
        iAbsEncVal = 0;
    }else{
      // ToDo Eilgang toggeln
      // 1 Sekunde halen Menü aufrufen
    }
  }

#if RICHTUNG > 0
  rightRotate = (iEncVal < 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#else
  rightRotate = (iEncVal > 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#endif
  // Daten berechnen und Timer steuern
  if ( iEncVal!=0 && iAbsEncVal < ESTMAX ){
    if ( iEncVal != iOldEncVal ){               // Jetzt hat sich was getan
      iOldEncVal = iEncVal;                     // Wert merken
      
      if ( hase ) {
        t = map( iAbsEncVal, ESTMIN, ESTMAX, Fmin, Fmax )*2;  // und Poti -> Frequenz mappen und mal 2 da wir ja pro TimerRoutine nur eine Flanke setzen HI>LO oder LO>HI
      } else {
        t = map( iAbsEncVal, ESTMIN, ESTMAX, Fmin, Fmax / RFAKTOR )*2;
      }
      debugLog( "real:" );
      debugLog( dtostrf( iEncVal, 4,0, cBuffer ) );
      debugLog( " gemappt:" );
      debugLog( ltoa( t,cBuffer, 10 ) );
      debugLog( "\n" );
      Timer0.setFrequency(t);
      Timer.start();
    }
  }else{
    if ( iAbsEncVal>0 ){                         // Wenn das Poti auf Anschlagswert ist nichts machen
     iEncVal = iOldEncVal;
    }else{                                       // Poti ist auf 0
      Timer0.stop();                             // Timer0 stoppen
      digitalWrite( ZSTEP,LOW );                 // Zur Sicherheit noch Step auf LOW setzen um sicher zu sein das gepullt wird
    }
  }

  // Position berechnen
  pos = mmPerStepZ * z_pos_count;

  // Daten schreiben
  writeData();

}
/*************************** Loop Ende ************************************/

