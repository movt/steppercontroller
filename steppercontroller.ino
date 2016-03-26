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
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define DEBUG    1                            // 1: Debug ein, 0: Debug aus


// Ausgangs-Pins
#define READY   13                            // Bereitschaft
#define ZSTEP    9                            // Pin fuer Step-Impuls
#define ZDIRP   11                            // Pin fuer Richtung
#define SLED     8                            // LED für den Eilgang

#define BACKLIGHT_PIN 3                       //LCD Pins...
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7                              //... LCD Pins

// Eingangs-Pins
#define ENC_CLOCK 49                          // Encoder Takt
#define ENC_DATA  47                          // Encoder Signal
#define ENC_SW    51                          // Encoder Druck-Taster
#define MENU      53                          // Menu-Taster

// Poti-Paramter
#define MITTE     0                           // Encoder Mittenstellung
#define ESTMIN    0                           // Minwert vom Encoder
#define ESTMAX   40                           // Maxwert vom Encoder je höher desto feinfühliger ist die Steuerung

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
#define I2C_ADR_Display 0x27                  // Adresse Display

char cBuffer[10], cLogValue[100], cDisplayValue[20]; // Variablen für  Ausgabe

long rFreq = 0;                               // Ausgelesene Timer Frequenz
long t = ESTMIN;                              // Anfänglicher Takt
bool rightRotate;                             // Rechts rum
bool bTimerRunning  = LOW ;

volatile int iEncVal = 0;                     // Encoderwert
volatile int iAbsEncVal = 0;
volatile int iOldEncVal = 0;                  // Encoderwert
volatile int change = 0;
volatile long lCounter = 0;                   // Counter für das Timer-Callback (DEBUGGING)
volatile bool hase;                           // HIGH = Hasengang, LOW = Schneckengang
volatile int  z_pos_count;                    // Counter der Steps
volatile float pos;                           // Positionszähler

//Displayinstanz lcd erstellen
LiquidCrystal_I2C lcd(I2C_ADR_Display, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin, BACKLIGHT_PIN, POSITIVE);



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
  Wire.begin();
  lcdSetup(); 

  
  pinMode( READY, OUTPUT );                     // Bereitschaftsanzeige
  pinMode( ZSTEP, OUTPUT );                     // Z Achsen Step
  pinMode( ZDIRP, OUTPUT );                     // Z Achsen Richtungs-Pin
  pinMode( ENC_DATA, INPUT );            // Encoder Daten
  pinMode( ENC_CLOCK, INPUT );           // Encoder Clock
  pinMode( ENC_SW, INPUT );                     // Encoder Taster
  pinMode( SLED, OUTPUT );                      // Eilgang LED
  pinMode( MENU, INPUT );                       // Menu-Taster
  
  //Vorbelegungen
  digitalWrite( ENC_SW, HIGH );                 // Pullup
  //digitalWrite( MENU, HIGH );
  digitalWrite( READY, LOW );                   // Erstmal nicht READY
  
  Timer0.attachInterrupt( zTimer );             // Timer0 Interrupt für die Z-Achse
    
  attachInterrupt(digitalPinToInterrupt(ENC_CLOCK),isr_enc,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DATA),isr_enc,FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_SW),isr_sw,FALLING);
  debugLog ( "Starte 1\n" );
  attachInterrupt(digitalPinToInterrupt(MENU), isr_menu, FALLING);  
  debugLog ( "Starte 2\n" );
  
  digitalWrite( READY, HIGH );                  // Bereit

  hase = HIGH;
  
  debugLog( "Bereit!\n" );
  delay( 500 );                                 // Zeit um eventuelle Fehler zu lesen
  lcdInit();
}

/* Encoder Interrupt-Routine */
void isr_enc( void ){
  
  change = getEncoderTurn();                // Encoderwert holen
  iEncVal = iEncVal + change;                   // Verstellwert mit Istwert abgleichen
  iAbsEncVal = abs( iEncVal );              // Wert für die Timerberechnung ohne Vorzeichen (das machen wir ja mit ZDIRP

}

void isr_sw(void){
  
    if( iAbsEncVal != 0 ){                      // un der aktuelle Wert ist ungleich 0
        iEncVal = 0;                            // wird der Speed genullt und der Stepper sollte sofort stoppen
        iAbsEncVal = 0;
        debugLog( "Stop ausgef.\n" );
    }else{
      debugLog( "Hase ausgef.\n" );
      // ToDo Eilgang toggeln
      hase = !hase;
      // 1 Sekunde halen Menü aufrufen
    }
}

void isr_menu(void){
  int i = 0;
  lcd.clear();                                  // Erstmal alles löschen
  lcdWrite( 0, 0, "Menu:" );
  while(i<10){
      lcdWrite(1,0,itoa(i,cBuffer,10));
      i++;
  }
}

/* write Timer um die Ausgabe am Display zu verlangsamen (Ghosting) */
void wTimer( void ){
  writeData();
}
/* Z-Achsen Timer Routine */
void zTimer( void ){
  
  Timer0.setFrequency(t);
  Timer0.start();
  int step=!digitalRead( ZSTEP );
  digitalWrite( ZSTEP,step );
  
  if ( rightRotate ){
    z_pos_count++;
  }else{
    z_pos_count--;
  }
}

void lcdSetup( void ){
  
  lcd.setBacklightPin( 3, POSITIVE );           //LCD init und Ausgabe starten
  lcd.setBacklight( 1 );
  lcd.begin( 20, 4 );                           // 20 Zeichen mit 4 Zeilen initialisieren
  lcd.setCursor( 0, 0 );
  lcd.clear();
  lcd.print( "Initialisiere..." );
}

void lcdInit( void ) {

  lcd.clear();                                  // Erstmal alles löschen
  lcdWrite( 0, 0, "mm/s :" );
  lcdWrite( 1, 0, "Z Pos:" );
  lcdWrite( 1, 12, "        " );

}

void lcdWrite ( int iRow, int iCol, String sValue ) {
  lcd.setCursor( iCol, iRow );
  lcd.print( sValue );
}

/*Daten schreiben*/
void writeData( void ) {
  float fWegSek;                
  lcdWrite( 0, 0, "mm/s :" );                     // Zeile 1 des Displays schreiben
  if ( bTimerRunning ){
    if ( hase ){
      fWegSek = rFreq*mmPerStepZ/0.8;             // Verfahrgeschwindigkeit / REFAKOR
    }else{
      fWegSek = rFreq*mmPerStepZ;
    }
    lcdWrite( 0, 12, dtostrf(fWegSek, 4, 3, cBuffer) );    
  }else{
     lcdWrite( 0, 12, "0.000" );       
  }
  
  if ( hase ) {
    lcdWrite(0, 19, "H");
  } else {
    lcdWrite(0, 19, "L");
  }
  if ( rightRotate ){
      lcdWrite( 1, 19, ">" );                     // Richtung für Z-Achse rauf
  }else{
      lcdWrite( 1, 19, "<" );                     // Richtung für Z-Achse runter
  }
  if ( !bTimerRunning ){
     lcdWrite( 1, 19, "X" );                      // Richtung für Z-Achse Stop
  }
  
  lcdWrite(1, 0, "Z Pos:");                       // Zeile 3 des Displays schreiben
  lcdWrite(1, 10, dtostrf(pos, 7, 3, cBuffer));

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
  
                                  
/*  if(digitalRead( ENC_SW ) == LOW ){          // Wenn der Taster gedrückt wird
    if( iAbsEncVal != 0 ){                      // un der aktuelle Wert ist ungleich 0
        iEncVal = 0;                            // wird der Speed genullt und der Stepper sollte sofort stoppen
        iAbsEncVal = 0;
    }else{
      // ToDo Eilgang toggeln
      // 1 Sekunde halen Menü aufrufen
    }
  }*/

#if RICHTUNG > 0
  rightRotate = (iEncVal < 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#else
  rightRotate = (iEncVal > 0) ? HIGH : LOW ;    // Richtungs-Pin fuer Stepper
#endif

  digitalWrite( ZDIRP, rightRotate );           // Richtung direkt an die Endstufe weitergeben

  // Daten berechnen und Timer steuern
  if ( iEncVal!=0 && iAbsEncVal < ESTMAX ){
    if ( iEncVal != iOldEncVal ){               // Jetzt hat sich was getan
      iOldEncVal = iEncVal;                     // Wert merken
      
      if ( hase ) {
        t = map( iAbsEncVal, ESTMIN, ESTMAX, Fmin, Fmax )*2;  // und Poti -> Frequenz mappen und mal 2 da wir ja pro TimerRoutine nur eine Flanke setzen HI>LO oder LO>HI
      } else {
        t = map( iAbsEncVal, ESTMIN, ESTMAX, Fmin, Fmax / RFAKTOR )*2;
      }
      
      if (!bTimerRunning){
        bTimerRunning = HIGH;
        Timer0.setFrequency(t);
        Timer0.start();
      }
      debugLog( "real:" );
      debugLog( dtostrf( iEncVal, 4,0, cBuffer ) );
      debugLog( " gemappt:" );
      debugLog( ltoa( t,cBuffer, 10 ) );
      debugLog(  " Realer Takt" );
      rFreq = Timer0.getFrequency();
      debugLog( ltoa(rFreq,cBuffer, 10 ) );

      debugLog( "\n" );

    }
  }else{
    if ( iAbsEncVal>0 ){                         // Wenn das Poti auf Anschlagswert ist nichts machen
     iEncVal = iOldEncVal;
    }else{                                       // Poti ist auf 0
      bTimerRunning = LOW;
      Timer0.stop();                             // Timer0 stoppen
      digitalWrite( ZSTEP,LOW );                 // Zur Sicherheit noch Step auf LOW setzen um sicher zu sein das gepullt wird
    }
  }

  // Position berechnen
  pos = mmPerStepZ * z_pos_count;

  writeData();
}
/*************************** Loop Ende ************************************/

