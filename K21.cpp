#include <Arduino.h>
#include "K21.h"

uint8_t previousButton = 100, pressedButton = 100;
uint8_t previousButton2 = 100, pressedButton2 = 100;
uint8_t vibrationState = 0;


K21::K21(uint8_t language, uint8_t numerics)
{
  lCount = 0;

  volAUX = 6;
  volSPEAKER = 0;
  pinMode(VIBRATE, OUTPUT);
  pinMode(SOUND, OUTPUT);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);

  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);

  digitalWrite(RGB_R, HIGH);
  digitalWrite(RGB_G, HIGH);
  digitalWrite(RGB_B, HIGH);

  pinMode(BT_STATUS, INPUT);
  pinMode(BT_CONNECTED, INPUT);
  pinMode(BT_PIO4, OUTPUT);

  pinMode(MIC, INPUT);

  pinMode(SDAUX, OUTPUT);
  pinMode(SDSPEAKER, OUTPUT);
  pinMode(AUXBUTTON, INPUT);

  digitalWrite(SDAUX, LOW); // HIGH - Left channel enabled LOW - shutdown
  digitalWrite(SDSPEAKER, LOW);
  digitalWrite(AUXBUTTON, HIGH);

  digitalWrite(BT_PIO4, HIGH);

  for (int sector = 0; sector < WINDOWSIZE; sector++) {
    padList[sector] = UNPRESSED;
  }

  pressedKeyInfo.mode = 0;
  capsLockState = 0;
  unpresState = 0;
  filter = 2;
  buttonMessage = "";
  volumeAdjust = 0;
  language = 0;
  languageState = 0;
  numerics = 0;
  numericsState = 0;
  buttonPressCounter = 0;
  modeState = 0;
  backspaceCount = 0;
  buttonDelay = 0;

  /* Swipo 1.0.2 keymapping */
  symbols[0][0] = 'i';  symbols[0][1] = 'j';  symbols[0][2] = 'k';  symbols[0][3] = 'l';  symbols[0][4] = 'm';  symbols[0][5] = 'n';  symbols[0][6] = 0;    symbols[0][7] = 0;
  symbols[1][0] = 0;    symbols[1][1] = 'o';  symbols[1][2] = 'p';  symbols[1][3] = 'r';  symbols[1][4] = 's';  symbols[1][5] = 0;    symbols[1][6] = 0;    symbols[1][7] = 0;
  symbols[2][0] = 0;    symbols[2][1] = 0;    symbols[2][2] = 't';  symbols[2][3] = 'u';  symbols[2][4] = 'v';  symbols[2][5] = 'z';  symbols[2][6] = 0;    symbols[2][7] = 0;
  symbols[3][0] = 0;    symbols[3][1] = 0;    symbols[3][2] = 0;    symbols[3][3] = 'w';  symbols[3][4] = 'y';  symbols[3][5] = 'q';  symbols[3][6] = 'x';  symbols[3][7] = 0;
  symbols[4][0] = '?';  symbols[4][1] = 39;   symbols[4][2] = '"';  symbols[4][3] = '@';  symbols[4][4] = 32;   symbols[4][5] = '.';  symbols[4][6] = ',';  symbols[4][7] = '!';
  symbols[5][0] = 0;    symbols[5][1] = 0;    symbols[5][2] = 0;    symbols[5][3] = 0;    symbols[5][4] = 0;    symbols[5][5] = 0;    symbols[5][6] = 0;    symbols[5][7] = 0;
  symbols[6][0] = 'c';  symbols[6][1] = 'd';  symbols[6][2] = 0;    symbols[6][3] = 0;    symbols[6][4] = 0;    symbols[6][5] = 0;    symbols[6][6] = 'a';  symbols[6][7] = 'b';
  symbols[7][0] = 'f';  symbols[7][1] = 'g';  symbols[7][2] = 'h';  symbols[7][3] = 0;    symbols[7][4] = 0;    symbols[7][5] = 0;    symbols[7][6] = 0;    symbols[7][7] = 'e';

  symbolsR[0][0] = 'b';  symbolsR[0][1] = 'q';  symbolsR[0][2] = 'r';  symbolsR[0][3] = 'k';  symbolsR[0][4] = 'v';  symbolsR[0][5] = 'y';  symbolsR[0][6] = 0;    symbolsR[0][7] = 0;
  symbolsR[1][0] = 0;    symbolsR[1][1] = 'j';  symbolsR[1][2] = 'g';  symbolsR[1][3] = 'h';  symbolsR[1][4] = 'c';  symbolsR[1][5] = 'n';  symbolsR[1][6] = 'e';  symbolsR[1][7] = 0;
  symbolsR[2][0] = 0;    symbolsR[2][1] = 0;    symbolsR[2][2] = 'a';  symbolsR[2][3] = '[';  symbolsR[2][4] = 'w';  symbolsR[2][5] = 'x';  symbolsR[2][6] = 'i';  symbolsR[2][7] = 'o';
  symbolsR[3][0] = 'z';  symbolsR[3][1] = 0;    symbolsR[3][2] = 0;    symbolsR[3][3] = ']';  symbolsR[3][4] = 's';  symbolsR[3][5] = 'm';  symbolsR[3][6] = 39;  symbolsR[3][7] = '.';
  symbolsR[4][0] = '&';  symbolsR[4][1] = 39;   symbolsR[4][2] = '@';  symbolsR[4][3] = 64;   symbolsR[4][4] = 32;   symbolsR[4][5] = '/';  symbolsR[4][6] = '?';  symbolsR[4][7] = '!';
  symbolsR[5][0] = 0;    symbolsR[5][1] = 0;    symbolsR[5][2] = 0;    symbolsR[5][3] = 0;    symbolsR[5][4] = 0;    symbolsR[5][5] = 0;    symbolsR[5][6] = 0;    symbolsR[5][7] = 0;
  symbolsR[6][0] = 'd';  symbolsR[6][1] = 'u';  symbolsR[6][2] = 'l';  symbolsR[6][3] = 0;    symbolsR[6][4] = 0;    symbolsR[6][5] = 0;    symbolsR[6][6] = 'f';  symbolsR[6][7] = ',';
  symbolsR[7][0] = '`';  symbolsR[7][1] = ';';  symbolsR[7][2] = 'p';  symbolsR[7][3] = 0;    symbolsR[7][4] = 0;    symbolsR[7][5] = 0;    symbolsR[7][6] = 0;    symbolsR[7][7] = 't';

  symbolsN[0][0] = '-';  symbolsN[0][1] = '+';  symbolsN[0][2] = '#';  symbolsN[0][3] = '_';  symbolsN[0][4] = '*';  symbolsN[0][5] = '/';  symbolsN[0][6] = '='; symbolsN[0][7] = '%';
  symbolsN[1][0] = '^';  symbolsN[1][1] = ':';  symbolsN[1][2] = ';';  symbolsN[1][3] = '(';  symbolsN[1][4] = ')';  symbolsN[1][5] = '\\'; symbolsN[1][6] = '"'; symbolsN[1][7] = '~';
  symbolsN[2][0] = 0;    symbolsN[2][1] = 0;    symbolsN[2][2] = 38;   symbolsN[2][3] = 64;   symbolsN[2][4] = 36;   symbolsN[2][5] = 156;  symbolsN[2][6] = 190; symbolsN[2][7] = 128;
  symbolsN[3][0] = '}';  symbolsN[3][1] = '[';  symbolsN[3][2] = ']';  symbolsN[3][3] = '>';  symbolsN[3][4] = '<';  symbolsN[3][5] = 96;   symbolsN[3][6] = 124; symbolsN[3][7] = '{';
  symbolsN[4][0] = '?';  symbolsN[4][1] = 39;   symbolsN[4][2] = '"';  symbolsN[4][3] = '@';  symbolsN[4][4] = 32;   symbolsN[4][5] = '.';  symbolsN[4][6] = ','; symbolsN[4][7] = '!';
  symbolsN[5][0] = 0;    symbolsN[5][1] = 0;    symbolsN[5][2] = 0;    symbolsN[5][3] = 0;    symbolsN[5][4] = 0;    symbolsN[5][5] = 0;    symbolsN[5][6] = 0;   symbolsN[5][7] = 0;
  symbolsN[6][0] = '3';  symbolsN[6][1] = '4';  symbolsN[6][2] = '5';  symbolsN[6][3] = 0;    symbolsN[6][4] = 0;    symbolsN[6][5] = 0;    symbolsN[6][6] = '1'; symbolsN[6][7] = '2';
  symbolsN[7][0] = '7';  symbolsN[7][1] = '8';  symbolsN[7][2] = '9';  symbolsN[7][3] = '0';  symbolsN[7][4] = 0;    symbolsN[7][5] = 0;    symbolsN[7][6] = 0;   symbolsN[7][7] = '6';
}

char K21::checkState(uint8_t buttonState1, uint8_t buttonState2, uint8_t buttonState3, uint8_t buttonState4, uint8_t buttonState5)
{
  if ( (buttonState1 == 0) && (buttonState2 == 0) && (buttonState3 == 0) ) {
    state = BA;
  } else if ( (buttonState1 == 0) && (buttonState3 == 0) && (buttonState4 == 0) ) {
    state = CB;
  } else if ( (buttonState1 == 0) && (buttonState4 == 0) && (buttonState5 == 0) ) {
    state = DC;
  } else if ( (buttonState1 == 0) && (buttonState5 == 0) && (buttonState2 == 0) ) {
    state = AD;
  } else if ( (buttonState1 == 0) && (buttonState2 == 0) ) {
    state = A;
  } else if ( (buttonState1 == 0) && (buttonState3 == 0) ) {
    state = B;
  } else if ( (buttonState1 == 0) && (buttonState4 == 0) ) {
    state = C;
  } else if ( (buttonState1 == 0) && (buttonState5 == 0) ) {
    state = D;
  } else if ( (buttonState1 == 0) ) {
    state = P;
  } else {
    state = UNPRESSED;
  }

  return state;
}

void K21::outputStateSerial(uint8_t stateOut)
{
  switch (stateOut) {
    case P:
      Serial.println("P");
      break;
    case A:
      Serial.println("A");
      break;
    case B:
      Serial.println("B");
      break;
    case C:
      Serial.println("C");
      break;
    case D:
      Serial.println("D");
      break;
    case AD:
      Serial.println("AD");
      break;
    case DC:
      Serial.println("DC");
      break;
    case CB:
      Serial.println("CB");
      break;
    case BA:
      Serial.println("BA");
      break;
    case UNPRESSED:
      Serial.println("UNPRESSED");
      break;
  };
}

void K21::powerOnVibration()
{
  vibrationON;
  delay(100);
  vibrationOFF;
}

long long t = millis();
long long tt = millis();
uint8_t lockLongPress = 1;

/**
    Getter functions
*/
String K21::getLanguage() {
  String currentLanguage;

  if (language) {
    currentLanguage = "ru_";
  } else {
    currentLanguage = "en_";
  }

  return currentLanguage;
}

String K21::getSymMode() {
  String currentSymMode;

  if (numerics) {
    currentSymMode = "ls_";
  } else {
    currentSymMode = "la_";
  }

  return currentSymMode;
}

String K21::getNavMode() {
  String currentNavMode;

  if (pressedKeyInfo.mode) {
    currentNavMode = "na_";
  } else {
    currentNavMode = "nd_";
  }

  return currentNavMode;
}

String K21::getSector() {
  String sector = "s";
  String lastSector;
  for (char j = 0; j < i; j++) {
    lastSector = lastStateList[j];
  }

  sector += lastStateList[0];
  if (lastSector) {
    sector += lastSector;
  } else {
    sector += lastStateList[0];
  }
  sector += "_";

  return sector;
}


/**
    Method for keycode formatting
*/
KeyInfo K21::keycodeFormatting(uint8_t sectorStatus, uint8_t buttonStatus) {
  String keycode = "";

  keycode += getLanguage();
  keycode += getSymMode();
  keycode += getNavMode();
  keycode += getSector();

  if (sectorStatus == IDLE) {
    keycode += "i_";
  } else if (sectorStatus == OVER) {
    keycode += "o_";
  } else if (sectorStatus == DRAGGED) {
    keycode += "d_";
  }

  if (buttonStatus == IDLE) {
    keycode += "i";
  } else if (buttonStatus == PRESSED) {
    keycode += "p";
  } else if (buttonStatus == RELEASED) {
    keycode += "r";
  } else if (buttonStatus == HELD) {
    keycode += "h";
  } else if (buttonStatus == DPRESSED) {
    keycode += "d";
  }

  Serial.println(keycode);
}

KeyInfo K21::volumeFormatting(String source, String volAdjustment, int16_t vol) {
  String volcode = "";

  volcode += getLanguage();
  volcode += getSymMode();
  volcode += getNavMode();
  volcode += source;
  volcode += volAdjustment;
  volcode += vol;

  Serial.println(volcode);
}

KeyInfo K21::functionFormatting(String functionMessage) {
  String functioncode = "";

  functioncode += getLanguage();
  functioncode += getSymMode();
  functioncode += getNavMode();
  functioncode += functionMessage;

  Serial.println(functioncode);
}


/**
   Method that controls navigation button presses
*/
KeyInfo K21::buttonOperations(uint8_t states, uint16_t touchh, uint8_t br2, uint8_t br4)
{
  /**
     Mode 1
  */
  /* Change device active language */
  if ( ((touchh == 32) && ((states == 1) || (states == 2))) &&
    (lockButtons == 0) && (languageState == 0) ) {
    languageState++;

    functionFormatting("ch_lang");
    language = language ^ 1;
    keycodeFormatting(IDLE, PRESSED);

    buttonMessage = "Language change";

#ifdef SOUNDEFFECT
    if (buttonPressCounter == 0) {
      SOUNDON1;
      delay(4);
      SOUNDOFF;
    }
#endif
  }

  /* Change to numerics & symbols layer */
  if ( ((touchh == 512) && ((states == 1) || (states == 8) || (states == 9))) &&
    (lockButtons == 0) && (numericsState == 0) ) {
    numericsState++;

    functionFormatting("ch_sym");
    numerics = numerics ^ 1;
    keycodeFormatting(IDLE, PRESSED);

    buttonMessage = "Symbol change";

#ifdef SOUNDEFFECT
    if (buttonPressCounter == 0) {
      SOUNDON1;
      delay(4);
      SOUNDOFF;
    }
#endif
  }

  /* Change navigation mode */
  if ( ( (touchh == 4) && ((states == 4) || (states == 2) || (states == 6)) ) &&
    (lockButtons == 0) && (modeState == 0) ) {
    modeState++;

    functionFormatting("ch_mode");
    if ( pressedKeyInfo.mode == 0 ) {
      pressedKeyInfo.mode = 1;
    } else pressedKeyInfo.mode = 0;
    keycodeFormatting(IDLE, PRESSED);

    buttonMessage = "Mode change";

#ifdef SOUNDEFFECT
    if (buttonPressCounter == 0) {
      SOUNDON1;
      delay(4);
      SOUNDOFF;
    }
#endif
  }

  /* Enter key */
  if ( ( (touchh == 128) && ((states == 4) || (states == 8) || (states == 12)) ) &&
       (lockButtons == 0) ) {
    Keyboard.write(KEY_RETURN);
    Serial1.print((char)13);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_enter");

    backspaceCount++;
    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 128;
    buttonMessage = "Enter";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* Toggle Caps Lock */
  if ( BR1 && pressedKeyInfo.mode == 0 && touchh == 64 && lockCapsLock == 0 ) {
    Keyboard.write(KEY_CAPS_LOCK);
    Serial1.print((char)28);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_caps");

    pressedKeyInfo.button = 1;
    buttonMessage = "Caps lock";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* TODO: Microphone */
  else if ( br2 && pressedKeyInfo.mode == 0 && touchh == 2 ) {
    //    Keyboard.write();
    //    Serial1.print();
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_mic");

    backspaceCount++;
    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 2;
    buttonMessage = "Microphone";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* Backspace */
  else if ( BR3 && pressedKeyInfo.mode == 0 && touchh == 8 ) {
    Keyboard.write(KEY_BACKSPACE);
    Serial1.print((char)8);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_bcksp");

    backspaceCount++;
    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 4;
    buttonMessage = "Backspace";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* TODO: Assistant key */
  else if ( br4 && pressedKeyInfo.mode == 0 && touchh == 256 ) {
    //    Keyboard.write();
    //    Serial1.print();
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_ai");

    backspaceCount++;
    if (backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 8;
    buttonMessage = "AI";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /**
     Mode 2
  */
  /* Up arrow */
  if ( BR1 && pressedKeyInfo.mode == 1 && touchh == 64 ) {
    Keyboard.write(KEY_UP_ARROW);
    Serial1.print((char)14);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_up");

    backspaceCount++;
    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    buttonMessage = "Up arrow";
    pressedKeyInfo.button = 1;

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* Right arrow */
  else if ( br2 && pressedKeyInfo.mode == 1 && touchh == 2 ) {
    Keyboard.write(KEY_RIGHT_ARROW);
    Serial1.print((char)7);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_right");

    backspaceCount++;
    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    buttonMessage = "Right arrow";
    pressedKeyInfo.button = 2;

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* Down arrow */
  else if ( BR3 && pressedKeyInfo.mode == 1 && touchh == 8 ) {
    Keyboard.write(KEY_DOWN_ARROW);
    Serial1.print((char)12);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_down");

    backspaceCount++;

    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 4;
    buttonMessage = "Down arrow";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  /* Left arrow */
  else if ( br4 && pressedKeyInfo.mode == 1 && touchh == 256 ) {
    Keyboard.write(KEY_LEFT_ARROW);
    Serial1.print((char)11);
    keycodeFormatting(IDLE, PRESSED);
    functionFormatting("mod_left");

    backspaceCount++;

    if ( backspaceCount < 2 ) {
      buttonDelay = 1;
    } else buttonDelay = 0;

    pressedKeyInfo.button = 8;
    buttonMessage = "Left arrow";

#ifdef SOUNDEFFECT
    SOUNDON1;
    delay(4);
    SOUNDOFF;
#endif
  }

  return pressedKeyInfo;
}

uint8_t K21::writeLetter(uint16_t value, uint8_t states, uint8_t pluged)
{
  uint8_t tmp1 = previousButton2;
  previousButton2 = pressedButton2;
  volumeAdjust = 0;

  if ( value == 4 ) {
    pressedButton2 = DC;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 128 ) {
    pressedButton2 = CB;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 16 ) {
    pressedButton2 = P;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 32 ) {
    pressedButton2 = AD;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 8 ) {
    pressedButton2 = C;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 2 ) {
    pressedButton2 = D;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 512 ) {
    pressedButton2 = BA;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 64 ) {
    pressedButton2 = A;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 1 ) {
    pressedButton2 = Z;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 256 ) {
    pressedButton2 = B;
    lCount++;
    unpresState = 0;
  }

  else if ( value == 0 ) {
    lCount++;
    pressedButton2 = UNPRESSED;
    unpresState++;
  }

  else {
    pressedButton2 = OTHER;
    lCount++;
    unpresState = 0;
  }

  for (int sector = 1; sector < WINDOWSIZE; sector++) {
    padList[sector - 1] = padList[sector];
  }
  padList[WINDOWSIZE - 1] = pressedButton2;

  /**
     TODO: Useful functionality - double taps, long press

    // Double tap reset
    if (millis()-t > 500) {
    t = millis();
    pCount=0;
    }

    // Double tap detection
    if (pCount >= 2) {
    pCount = 0;
    letter = 8; //BACKSPACE
    Serial.println("DOUBLE TAP");
    lockLongPress = 0;
    return 1;
    }

    // Long press
    if ( (lCount >= 7) && (!lockLongPress) ) {
      letter = 8; //BACKSPACE
      Serial.println("LONG PRESS");
      return 1;
    }

    // Unlock long press
    if( (previousButton2 == UNPRESSED) && (pressedButton2 == P) ) {
    lockLongPress = 0;
    Serial.println("Unlocked");
    }
    TODOEND
  */

  /* Button state changed */
  //atfiltree trokšņu pieskaarienus un nodzeesh nepareizos ierakstus
  /*if ((pressedButton2 != previousButton2) && (pressedButton2 != UNPRESSED)&& (pressedButton2 != P)&&(pressedButton2 != OTHER)&&(states==0) && (lCount<=filter) && (i>=0)) //notesteet bez i
    {
    Serial.print("presssedbutton1: ");
    Serial.println(pressedButton2);

    if(i==0)
    {
      lastStateList[0] = pressedButton2;
      i++;
      Serial.print("original list2: ");
      outputStateList();
      lCount=0;
      return 0;
    }
    else{
      Serial.print("original list: ");
      outputStateList();
      Serial.print("i: ");
      Serial.println(i);

      lastStateList[i] = NULL;
      i--;
      lCount=0;
      if(i==0)
        {
        pressedButton2 = previousButton2; return 0;
        }
      else
        {
          lastStateList[i] = NULL;


          Serial.print("fixed list: ");
          outputStateList();

          pressedButton2 = previousButton2;
          return 0;
        }
    }
    }*/

  /* Fix so there's no overflow */
  if (lCount > 300) {
    lCount = filter + 10;
  }

  if ( (pressedButton2 != previousButton2) && (pressedButton2 != UNPRESSED) &&
       (pressedButton2 != P) && (pressedButton2 != OTHER) && (states == 0) &&
       (lCount <= filter) && (i != 0) ) {
    if (i == 1) {
      lastStateList[0] = pressedButton2;
    } else lastStateList[i - 1] = pressedButton2;
    lCount = 0;

    return 0;
  }
  // i - number of time there's been sector change

  /* Button state changed */
  if ( (pressedButton2 != previousButton2) && (pressedButton2 != UNPRESSED) &&
       (pressedButton2 != P) && (pressedButton2 != OTHER) && (states == 0) &&
       (lCount > filter) ) {

    vibrationON;
    delay(30);
    vibrationOFF;

    lastStateList[i] = pressedButton2;
    i++;

    lCount = 0; // Counts how long sector been held for

    keycodeFormatting(OVER, IDLE);

    //lock longpress
    if ( (previousButton2 != UNPRESSED) && (pressedButton2 != P) ) {
//      lockLongPress = 1;
//      Serial.println("LONG PRESS");
    }

#ifdef VIBRATIONEFFECT
    vibrationON;
    delay(50);
    vibrationOFF;
#endif

    // TODO: Volume control?
    //t = millis();
    //vibrationState = 1;
    if (i > 8) {
      volumeAdjust = 1;
      int8_t tmp1 = (lastStateList[i - 2] - lastStateList[i - 1]);

      if ( (tmp1 < 0 && tmp1 != -7) || tmp1 == 7 ) {
        if (!pluged) {
          volSPEAKER += 10;
          if (volSPEAKER > 320) volSPEAKER = 320;

          volumeFormatting("speaker_", "up_", volSPEAKER);
        } else if (pluged) {
          volAUX += 1;
          if (volAUX > 30) volAUX = 30;

          volumeFormatting("aux_", "up_", volAUX);
        }
      }
      else if ( (tmp1 > 0 && tmp1 != 7) || tmp1 == -7 ) {
        if (!pluged) {
          volSPEAKER -= 10;

          if (volSPEAKER <= 0) volSPEAKER = 0;

          volumeFormatting("speaker_", "dn_", volSPEAKER);
        }
        else if (pluged) {
          volAUX -= 1;

          if (volAUX <= 0) volAUX = 0;
          volumeFormatting("aux_", "dn_", volAUX);
        }
      }
    }// end of if (i>8)
  } else if ( (pressedButton2 != previousButton2) && (pressedButton2 == UNPRESSED) &&
       (pressedButton2 != P) && (pressedButton2 != OTHER) && (states == 0) &&
       (lCount > filter) && (i > 0) ) {
    keycodeFormatting(IDLE, IDLE);
  }
  /* END of button state changed */


  /* Letter confirmation */
  if ( (i > 0) && (previousButton2 != P) && (pressedButton2 == P) &&
       (states == 0) && (lCount > filter) ) {
    lastStateList[i] = 10; // ieraksta beigu marķieri
    lCount = 0;

    if (numerics == 0) {
      if (language == 0) {
        letter = symbols[lastStateList[0]][lastStateList[i - 1]];
      } else {
        letter = symbolsR[lastStateList[0]][lastStateList[i - 1]];
      }
    } else {
      letter = symbolsN[lastStateList[0]][lastStateList[i - 1]];
    }

    keycodeFormatting(DRAGGED, IDLE);
    return 1;
  }

  /* Gesture stand-by */
  if (pressedButton2 == UNPRESSED) {
    i = 0;
    lastStateList[0] = 10;
    return 0;
  } else return 0;
}
