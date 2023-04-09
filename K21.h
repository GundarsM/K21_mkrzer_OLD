#include <Arduino.h>
#include <Keyboard.h>
/*
          A
      BA  |  AD
     B -  P -  D
      CB  |  DC
          C
*/
#define SOUNDEFFECT // VIBRATIONEFFECT

/**
   Pad sectors
*/
#define A 0
#define AD 1
#define D 2
#define DC 3
#define C 4
#define CB 5
#define B 6
#define BA 7
#define P 8
#define UNPRESSED 9
#define OTHER 10
#define Z 11

/**
   Sector states
*/
#define IDLE    0 // Used by both sector and button state
#define OVER    1
#define DRAGGED 2

/**
   Button states
*/
#define PRESSED   1
#define RELEASED  2
#define HELD      3
#define DPRESSED  4

/**
   Define INPUT pins hidden buttons
*/
#define BTN1 6
#define BTN3 7

#define MIC A1

/**
   OUTPUTS
*/
#define RGB_R 4   // PWM
#define RGB_G 5   // PWM
#define RGB_B A2  // No PWM

#define redON     digitalWrite(RGB_R, LOW); digitalWrite(RGB_G, HIGH); digitalWrite(RGB_B, HIGH);
#define greenON   digitalWrite(RGB_G, LOW); digitalWrite(RGB_R, HIGH); digitalWrite(RGB_B, HIGH);
#define blueON    digitalWrite(RGB_B, LOW); digitalWrite(RGB_R, HIGH); digitalWrite(RGB_G, HIGH);
#define blueToggle digitalWrite(RGB_B, !digitalRead(RGB_B)); digitalWrite(RGB_R, HIGH); digitalWrite(RGB_G, HIGH);
#define whiteON   digitalWrite(RGB_R, LOW); digitalWrite(RGB_G, LOW);  digitalWrite(RGB_B, LOW);
#define ledsOFF   digitalWrite(4, HIGH);    digitalWrite(5, HIGH);     digitalWrite(A2, HIGH);

/**
   Bluetooth
*/
#define BT_STATUS 9
#define BT_CONNECTED A5
#define BT_PIO4 8 // Reset pin

/**
   Audio enable pins
*/
#define SDAUX 10
#define SDSPEAKER A4
#define AUXBUTTON A3
#define AUXon digitalWrite(SDAUX,HIGH); digitalWrite(SDSPEAKER,LOW);
#define SPEAKERon digitalWrite(SDAUX,LOW); digitalWrite(SDSPEAKER,HIGH);

/**
   Vibration motor pin
*/
#define VIBRATE 1
#define SOUND 0

/**
   Vibration constants
*/
#define longVIBRATION 100
#define shortVIBRATION 10

#define SOUNDON1 tone(SOUND,5000);
#define SOUNDON2 tone(SOUND,2000);
#define SOUNDOFF noTone(SOUND);

#define WINDOWSIZE 5

#define vibrationON digitalWrite(VIBRATE,HIGH);
#define vibrationOFF digitalWrite(VIBRATE,LOW);

/**
   READ actions from buttons
*/
#define BR1 (byte)~digitalRead(BTN1)&0x01
#define BR3 (byte)~digitalRead(BTN3)&0x01

/**
   Language definitions
*/
#define ENGLISH 1
#define LATVIAN 2
#define RUSSIAN 3

typedef struct KeyInfo {
  uint8_t mode;
  uint8_t button;
} KeyInfo;

/**
   Definition of functions for K21 operation
*/
class K21
{
  public:
    K21(uint8_t language, uint8_t numerics);

    /* Functions */
    char checkState(uint8_t buttonState1, uint8_t buttonState2, uint8_t buttonState3, uint8_t buttonState4, uint8_t buttonState5);
    void outputStateSerial(uint8_t stateOut);
    void powerOnVibration();
    uint8_t writeLetter(uint16_t value, uint8_t states, uint8_t pluged);
    KeyInfo buttonOperations(uint8_t states, uint16_t touchh, uint8_t br2, uint8_t br4);
    String getLanguage();
    String getSymMode();
    String getSector();
    String getNavMode();
    KeyInfo keycodeFormatting(uint8_t sectorStatus, uint8_t buttonStatus);
    KeyInfo volumeFormatting(String source, String volAdjustment, int16_t vol);
    KeyInfo functionFormatting(String functionMessage);
    char list[16];
    int16_t volAUX, volSPEAKER, buttonPressCounter;
    uint8_t volumeAdjust;

    /* Variables */
    uint8_t buttonState1;
    uint8_t buttonState2;
    uint8_t buttonState3;
    uint8_t buttonState4;
    uint8_t buttonState5;
    uint8_t symbols[8][8];  // English alphabet
    uint8_t symbolsR[8][8]; // Russian alphabet
    uint8_t symbolsN[8][8]; // Numerics & symbols
    uint16_t padList[WINDOWSIZE];
    uint8_t state;
    uint8_t lastStateList[200];
    uint8_t debTime;
    uint8_t i;
    uint8_t letter;
    uint8_t unpresState;
    uint16_t capsLockState;
    uint8_t lockCapsLock;
    uint8_t lockButtons;
    uint8_t filter;
    uint8_t lCount;
    uint8_t language;
    uint8_t languageState;
    uint8_t numerics;
    uint8_t numericsState;
    uint8_t modeState;
    uint8_t backspaceCount;
    uint8_t buttonDelay;
    String buttonMessage;
    KeyInfo pressedKeyInfo;
};
