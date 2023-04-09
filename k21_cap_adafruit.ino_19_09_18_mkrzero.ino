/*
  For capacitive sensor to work better I changed in AdafruitMPR121.cpp:
  1) Added fixed baseline for each pad - ~256, works fine
  2) changed MPR_121_ECR register walue from 0x8F to 0x4F - wont update baselines

   MAX08357:
     GND connected GND
     VIN connected 5V
     LRC connected to pin 0 (Zero) or pin 3 (MKR1000, MKRZero)
     BCLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKRZero)
     DIN connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero)


  Bluetooth uses 'Serial1'
*/


#include <Wire.h>
#include <SPI.h>
#include "K21.h"
#include "Adafruit_MPR121.h"
#include <stdio.h>
#include <stdlib.h>
#include <SD.h>
#include <ArduinoSound.h>
#include "audiNames.h"
#include "bluetoothFunc.h"


// Variable representing WAV files
SDWaveFile waveFileHELLO, waveFileA, waveFileA1, waveFileA2, waveFileA3, waveFileA4, waveFileB, waveFileC, waveFileD, waveFileE,
           waveFileF, waveFileG, waveFileH, waveFileI, waveFileJ, waveFileO, waveFileK, waveFileL, waveFileM, waveFileN, waveFileP, waveFileQ,
           waveFileR, waveFileS, waveFileT, waveFileU, waveFileV, waveFileW, waveFileX, waveFileY, waveFileZ, waveFile0, waveFile1, waveFile2,
           waveFile3, waveFile4, waveFile5, waveFile6, waveFile7, waveFile8, waveFile9, waveFileAND, waveFileAT, waveFileATX, waveFileCLN, waveFileCMA,
           waveFileCPA, waveFileDLR, waveFileDOT, waveFileDSH, waveFileENT, waveFileEQL, waveFileEXM, waveFileGRT, waveFileHSH, waveFileLET, waveFileOPA,
           waveFilePER, waveFilePLS, waveFileQTM, waveFileQUO, waveFileSEC, waveFileSLH, waveFileSPC, waveFileSQT, waveFileTLD, waveFileUNS,
           waveFileAMO, waveFileBCO, waveFileBDO, waveFileBER, waveFileBSP, waveFileCBA, waveFileCER, waveFileDWN, waveFileEMO, waveFileLFT,
           waveFileMOC, waveFileRHT, waveFileSER, waveFileUCO, waveFileUDO, waveFileUER, waveFileUP, waveFileAXC, waveFileAXD,
           waveFileCLO, waveFileCOF, waveFileCLK, waveFileECO, waveFileENG, waveFileRUS, waveFileBEP, waveFileBP2, waveFileBP3, waveFileSLP,
           waveFileBTF, waveFileBTO, waveFileWKU, waveFileCBF,

           waveFileRA, waveFileRB, waveFileRV, waveFileRII, waveFileRG, waveFileRD, waveFileRSH, waveFileRJO, waveFileRE, waveFileRZH, waveFileRIJ,
           waveFileRJA, waveFileRCH, waveFileRC, waveFileRY, waveFileRF, waveFileRJU, waveFileRNO, waveFileRIII, waveFileREE, waveFileRN, waveFileRM,
           waveFileRI, waveFileRK, waveFileRL, waveFileRZ, waveFileRJJ, waveFileRT, waveFileRS, waveFileRO, waveFileRP, waveFileRR, waveFileR0,
           waveFileR1, waveFileR2, waveFileR3, waveFileR4, waveFileR5, waveFileR6, waveFileR7, waveFileR8, waveFileR9, waveFileRSPC, waveFileRH,
           waveFileRTCH, waveFileRUSC, waveFileRCLN, waveFileRCOM, waveFileRCPA, waveFileRDSH, waveFileRENT, waveFileREXM, waveFileROPA, waveFileRQUM, waveFileRSLN, waveFileRSLH,
           waveFileRCOL, waveFileRBSP, waveFileRCHB, waveFileRCHM, waveFileRDWN, waveFileRLFT, waveFileRRGH, waveFileRRUS, waveFileRSLP, waveFileRUP, waveFileRWKU, waveFileRCHA;

File myFile;


#define SOUNDEFFECT // VIBRATIONEFFECT

#define BR2 ((~(cap.gpioData())&0x0080)>>7)
#define BR4 ((~(cap.gpioData())&0x0040)>>6)

K21 k21(ENGLISH, 0);

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
uint8_t acceptSoundState = 0;
uint8_t correction = 0, debounce = 0, AUXconnected = 0;
uint16_t baseList[12];
uint16_t touchedPad = 0, inactiveCounter = 0;
uint8_t previousLanguage = 0, sleepMode = 0, buttonReleased = 0, discoveryState = 0;
int16_t vollActive = 0, firstSleepCycle = 0, unConnectedCountBT = 0;


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  /*************** bluetooth setup - done once ****************/
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");

  Serial.print("Response: ");
  Serial.println(Serial1.read());

  delay(100);  // Short delay, wait for the Mate to send back CMD
  //Serial1.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // delay(10);
  //Serial1.println("SN,SWIPo");
  delay(10);
  //Serial1.println("SM,6");
  //Serial1.println("SH,0200");
  delay(10);
  Serial1.println("SM,3"); //iesleedz auto reconnect peec powerdown
  delay(100);
  Serial.println("Bluetooth setup done");

  resetBT();
  delay(100);
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");
  delay(30);
  //  Serial.print("Response: ");
  //  Serial.println(Serial1.read());
  delay(100);  // Short delay, wait for the Mate to send back CMD
  delay(10);
  Serial1.println("SM,3");
  delay(150);
  Serial1.println("SQ,8"); // Quiet, Turn off Discovery and Connectability
  delay(50);
  Serial1.println ("R,1"); // Restart BT
  delay(50);
  /*************** END bluetooth setup - done once ****************/

  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  Serial1.begin(115200);  // Start bluetooth serial at 9600

  //For audio starts SD card
  Serial.println("Initializing SD card...");

  if (!SD.begin(SS1)) {
    Serial.println("SD initialization failed!");
  }
  Serial.println("SD initialization done.");

  SPEAKERon;
  AUXon;


  /**
     Create a SDWaveFile (English)
  */
  waveFileA = SDWaveFile(filenameA);
  waveFileB = SDWaveFile(filenameB);
  waveFileC = SDWaveFile(filenameC);
  waveFileD = SDWaveFile(filenameD);
  waveFileE = SDWaveFile(filenameE);
  waveFileF = SDWaveFile(filenameF);
  waveFileG = SDWaveFile(filenameG);
  waveFileH = SDWaveFile(filenameH);
  waveFileI = SDWaveFile(filenameI);
  waveFileJ = SDWaveFile(filenameJ);
  waveFileK = SDWaveFile(filenameK);
  waveFileL = SDWaveFile(filenameL);
  waveFileM = SDWaveFile(filenameM);
  waveFileN = SDWaveFile(filenameN);
  waveFileO = SDWaveFile(filenameO);
  waveFileP = SDWaveFile(filenameP);
  waveFileQ = SDWaveFile(filenameQ);
  waveFileR = SDWaveFile(filenameR);
  waveFileS = SDWaveFile(filenameS);
  waveFileT = SDWaveFile(filenameT);
  waveFileU = SDWaveFile(filenameU);
  waveFileV = SDWaveFile(filenameV);
  waveFileZ = SDWaveFile(filenameZ);
  waveFileW = SDWaveFile(filenameW);
  waveFileX = SDWaveFile(filenameX);
  waveFileY = SDWaveFile(filenameY);

  waveFile0 = SDWaveFile(filename0);
  waveFile1 = SDWaveFile(filename1);
  waveFile2 = SDWaveFile(filename2);
  waveFile3 = SDWaveFile(filename3);
  waveFile4 = SDWaveFile(filename4);
  waveFile5 = SDWaveFile(filename5);
  waveFile6 = SDWaveFile(filename6);
  waveFile7 = SDWaveFile(filename7);
  waveFile8 = SDWaveFile(filename8);
  waveFile9 = SDWaveFile(filename9);

  waveFileAND = SDWaveFile(filenameAND);
  waveFileAT  = SDWaveFile(filenameAT);
  waveFileATX = SDWaveFile(filenameATX);
  waveFileCLN = SDWaveFile(filenameCLN);
  waveFileCMA = SDWaveFile(filenameCMA);
  waveFileCPA = SDWaveFile(filenameCPA);
  waveFileDLR = SDWaveFile(filenameDLR);
  waveFileDOT = SDWaveFile(filenameDOT);
  waveFileDSH = SDWaveFile(filenameDSH);
  waveFileENT = SDWaveFile(filenameENT);
  waveFileEQL = SDWaveFile(filenameEQL);
  waveFileEXM = SDWaveFile(filenameEXM);
  waveFileGRT = SDWaveFile(filenameGRT);
  waveFileHSH = SDWaveFile(filenameHSH);
  waveFileLET = SDWaveFile(filenameLET);
  waveFileOPA = SDWaveFile(filenameOPA);
  waveFilePER = SDWaveFile(filenamePER);
  waveFilePLS = SDWaveFile(filenamePLS);
  waveFileQTM = SDWaveFile(filenameQTM);
  waveFileQUO = SDWaveFile(filenameQUO);
  waveFileSEC = SDWaveFile(filenameSEC);
  waveFileSLH = SDWaveFile(filenameSLH);
  waveFileSPC = SDWaveFile(filenameSPC);
  waveFileSQT = SDWaveFile(filenameSQT);
  waveFileTLD = SDWaveFile(filenameTLD);
  waveFileUNS = SDWaveFile(filenameUNS);

  waveFileAMO = SDWaveFile(filenameAMO);
  waveFileBCO = SDWaveFile(filenameBCO);
  waveFileBDO = SDWaveFile(filenameBDO);
  waveFileBER = SDWaveFile(filenameBER);
  waveFileBSP = SDWaveFile(filenameBSP);
  waveFileCBA = SDWaveFile(filenameCBA);
  waveFileCER = SDWaveFile(filenameCER);
  waveFileDWN = SDWaveFile(filenameDWN);
  waveFileEMO = SDWaveFile(filenameEMO);
  waveFileLFT = SDWaveFile(filenameLFT);
  waveFileMOC = SDWaveFile(filenameMOC);
  waveFileRHT = SDWaveFile(filenameRHT);
  waveFileSER = SDWaveFile(filenameSER);
  waveFileUCO = SDWaveFile(filenameUCO);
  waveFileUDO = SDWaveFile(filenameUDO);
  waveFileUER = SDWaveFile(filenameUER);
  waveFileUP =  SDWaveFile(filenameUP);

  waveFileAXC = SDWaveFile(filenameAXC);
  waveFileAXD = SDWaveFile(filenameAXD);
  waveFileCOF = SDWaveFile(filenameCOF);
  waveFileCLK = SDWaveFile(filenameCLK);
  waveFileECO = SDWaveFile(filenameECO);
  waveFileENG = SDWaveFile(filenameENG);
  waveFileRUS = SDWaveFile(filenameRUS);
  waveFileBEP = SDWaveFile(filenameBEP);
  waveFileBP2 = SDWaveFile(filenameBP2);
  waveFileBP3 = SDWaveFile(filenameBP3);
  waveFileSLP = SDWaveFile(filenameSLP);
  waveFileBTF = SDWaveFile(filenameBTF);
  waveFileBTO = SDWaveFile(filenameBTO);
  waveFileWKU = SDWaveFile(filenameWKU);
  waveFileCBF = SDWaveFile(filenameCBF);

  /**
     Create a SDWaveFile (Russian)
  */
  waveFileRA = SDWaveFile(filenameRA);
  waveFileRB = SDWaveFile(filenameRB);
  waveFileRV = SDWaveFile(filenameRV);
  waveFileRII = SDWaveFile(filenameRII);
  waveFileRG = SDWaveFile(filenameRG);
  waveFileRD = SDWaveFile(filenameRD);
  waveFileRSH = SDWaveFile(filenameRSH);
  waveFileRJO = SDWaveFile(filenameRJO);
  waveFileRE = SDWaveFile(filenameRE);
  waveFileRZH = SDWaveFile(filenameRZH);
  waveFileRIJ = SDWaveFile(filenameRIJ);
  waveFileRJA = SDWaveFile(filenameRJA);
  waveFileRCH = SDWaveFile(filenameRCH);
  waveFileRC = SDWaveFile(filenameRC);
  waveFileRY = SDWaveFile(filenameRY);
  waveFileRF = SDWaveFile(filenameRF);
  waveFileRJU = SDWaveFile(filenameRJU);
  waveFileRNO = SDWaveFile(filenameRNO);
  waveFileRIII = SDWaveFile(filenameRIII);
  waveFileREE = SDWaveFile(filenameREE);
  waveFileRN = SDWaveFile(filenameRN);
  waveFileRM = SDWaveFile(filenameRM);
  waveFileRI = SDWaveFile(filenameRI);
  waveFileRK = SDWaveFile(filenameRK);
  waveFileRL = SDWaveFile(filenameRL);
  waveFileRZ = SDWaveFile(filenameRZ);
  waveFileRJJ = SDWaveFile(filenameRJJ);
  waveFileRT = SDWaveFile(filenameRT);
  waveFileRS = SDWaveFile(filenameRS);
  waveFileRO = SDWaveFile(filenameRO);
  waveFileRP = SDWaveFile(filenameRP);
  waveFileRR = SDWaveFile(filenameRR);
  waveFileRH = SDWaveFile(filenameRH);

  waveFileR1 = SDWaveFile(filenameR1);
  waveFileR2 = SDWaveFile(filenameR2);
  waveFileR3 = SDWaveFile(filenameR3);
  waveFileR4 = SDWaveFile(filenameR4);
  waveFileR5 = SDWaveFile(filenameR5);
  waveFileR6 = SDWaveFile(filenameR6);
  waveFileR7 = SDWaveFile(filenameR7);
  waveFileR8 = SDWaveFile(filenameR8);
  waveFileR9 = SDWaveFile(filenameR9);
  waveFileR0 = SDWaveFile(filenameR0);

  waveFileRSPC = SDWaveFile(filenameRSPC);
  waveFileRTCH = SDWaveFile(filenameRTCH);
  waveFileRUSC = SDWaveFile(filenameRUSC);
  waveFileRCLN = SDWaveFile(filenameRCLN);
  waveFileRCOM = SDWaveFile(filenameRCOM);
  waveFileRCPA = SDWaveFile(filenameRCPA);
  waveFileRDSH = SDWaveFile(filenameRDSH);
  waveFileRENT = SDWaveFile(filenameRENT);
  waveFileREXM = SDWaveFile(filenameREXM);
  waveFileROPA = SDWaveFile(filenameROPA);
  waveFileRQUM = SDWaveFile(filenameRQUM);
  waveFileRSLN = SDWaveFile(filenameRSLN);
  waveFileRSLH = SDWaveFile(filenameRSLH);
  waveFileRCOL = SDWaveFile(filenameRCOL);

  waveFileRBSP = SDWaveFile(filenameRBSP);
  waveFileRCHB = SDWaveFile(filenameRCHB);
  waveFileRCHM = SDWaveFile(filenameRCHM);
  waveFileRDWN = SDWaveFile(filenameRDWN);
  waveFileRLFT = SDWaveFile(filenameRLFT);
  waveFileRRGH = SDWaveFile(filenameRRGH);
  waveFileRRUS = SDWaveFile(filenameRRUS);
  waveFileRSLP = SDWaveFile(filenameRSLP);
  waveFileRUP  = SDWaveFile(filenameRUP);
  waveFileRWKU = SDWaveFile(filenameRWKU);
  waveFileRCHA = SDWaveFile(filenameRCHA);

  Serial.println("Config done");
  redON;
  delay(1000);
  greenON;
  delay(1000);
  blueON;
  delay(1000);
  ledsOFF;

  SOUNDON1;
  delay(100);
  SOUNDOFF;

  vibrationON;
  delay(200);
  vibrationOFF;

  /* For testing Bluetooth */
  //  while(1)
  //  {
  //      Serial.print("BT statuss: ");
  //      Serial.println(digitalRead(BT_STATUS));
  //      Serial.print("BT connected: ");
  //      Serial.println(digitalRead(BT_CONNECTED));
  //      Serial.print("BT PIO6: ");
  //      Serial.println(digitalRead(BT_PIO4));
  //         if(digitalRead(BT_CONNECTED)) {blueON;}
  //         else {greenON;}
  //      delay(200);
  //      //Serial1.print('f');
  //    }

  delay(1);
  Serial.println("Setup CAP done.");

  /* Check if MPR121 is found */
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
  cap.setThresholds(3, 3);

  /* For setting baseline - read actual capacity values  */
  for (uint8_t i = 0; i < 12; i++) {
    baseList[i] = cap.filteredData(i) ;
  }
  for (uint8_t i = 0; i < 12; i++) {
    baseList[i] = (baseList[i] + cap.filteredData(i)) / 2 ;
  }
  for (uint8_t i = 0; i < 12; i++) {
    Serial.print(baseList[i]); Serial.print("\t");
  }
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ADC_BATTERY, INPUT);

  // open a new file and immediately close it - create file:
  // Serial.println("Creating example.txt...");
  // myFile = SD.open("k21_log.txt", FILE_WRITE);
  // myFile.close();

  // Check to see if the file exists
  if ( SD.exists("k21_log.txt") ) {
    Serial.println("k21_log.txt exists.");
  } else {
    Serial.println("k21_log.txt doesn't exist.");
  }
} // END SETUP

unsigned long time = 0;
unsigned long timeVibration = 0;
unsigned long chargeCounter = 0;
unsigned long currentMillis = millis(), buttonMillis = millis(), BTtimeout = millis();
uint8_t caps = 0; //32
uint8_t buttons = 0, toggl = 0, prevButtons = 0, prevBTState = 0; // 0 - disconnected
uint8_t keyMode = 0, previousMode = 0, testSound = 0, sleepActive = 0, batteryState = 1; // 1 - good battery
unsigned long lockButtonsTime = 0;
unsigned long auxTimeout = millis(), sdLog = millis(), batTime = millis();
uint16_t auxDetection =  1;
String dataString = "";
uint16_t discoveryBT = 0;
uint16_t blueLedCounter = 0, prevBatValue = 1020;
int sensorValue1 = 1020;

void loop() {
  /**
     BATTERY
  */
  /* Checks sensor once every 30s */
  if (millis() - batTime > 30000) {
    batTime = millis();

    prevBatValue = sensorValue1;
    sensorValue1 = analogRead(ADC_BATTERY);

    /* Trying to cut out false readings */
    if ( (abs)(prevBatValue - sensorValue1) > 100 ) {
      sensorValue1 = prevBatValue;
    }

    float voltage = sensorValue1 * (4.3 / 1023.0);

    /* Check if battery needs charging */
    if (voltage < 3.75 && batteryState == 1) {
      redON;

      /* Notifies user in the active language */
      if (k21.language == 0) {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileCBA);
      } else {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileRCHB);
      }
      batteryState = 0;
    } else if ( voltage > 4.28 && (batteryState == 0 || batteryState == 2) ) {
      chargeCounter++;
      batteryState = 2; // Charging

      /* After 1hr of charging */
      if (chargeCounter > 120) {
        ledsOFF;
        batteryState = 1;
        chargeCounter = 0;

        if (k21.language == 0 && batteryState == 1) {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileCBF);
        } else {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileRCHA);
        }
      }
    }
  }



  /**
     BLUETOOTH reconnecting attempt
  */
  if (millis() - BTtimeout >= 1000) {
    if ( !digitalRead(BT_CONNECTED) ) {
      unConnectedCountBT++;
    } else {
      unConnectedCountBT = 0;
    }

    /* Checks for BT state change */
    if (digitalRead(BT_CONNECTED) != prevBTState && !sleepMode) {
      prevBTState = digitalRead(BT_CONNECTED);

      /* When connected */
      if (digitalRead(BT_CONNECTED) == 1) {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileBCO);
        discoveryState = 0;
        /* When disconnected leave discoverable */
      } else if (digitalRead(BT_CONNECTED) == 0) {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileBDO);
        discoveryState = 1;
      }
    }

    /* Discoverability stays on for ~60s, afterwards BT discovery turns off  */
    if (unConnectedCountBT > 65) {
      unConnectedCountBT = 65;
    }
    if (unConnectedCountBT > 60 && discoveryState) {
      discoveryBT = 1;
      discoveryState = 0;
      if (AudioOutI2S.isPlaying()) {
        AudioOutI2S.stop();
      }
      AudioOutI2S.play(waveFileBTF);
      BT_discoveryOFF();
    }

    testSound = 0;
    BTtimeout = millis();

    /* Safety against variable overflow */
    if (inactiveCounter > 5000) {
      inactiveCounter = 500;
    }

    /* Needs to be 60 */
    if (inactiveCounter > 60) {
      if (firstSleepCycle) {
        if (k21.language == 0) {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileSLP);
        } else {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileRSLP);
        }

        k21.functionFormatting("dev_slp");
        BT_disconnect();
      }

      sleepMode = 1;
      prevBTState = 0;
      if (!sleepActive) {
        ledsOFF;
        sleepActive = 1;
        vibrationON;
        delay(100);
        vibrationOFF;
        delay(100);
        vibrationON;
        delay(100);
        vibrationOFF;
      }
      firstSleepCycle = 0;
    } // Enter sleep mode
  }



  /**
     Lock buttons
  */
  if (millis() - lockButtonsTime >= 500 && k21.lockButtons > 0) {
    k21.lockButtons = 0;
    k21.lockCapsLock = 0;
  }



  /**
     AUX detection
  */
  if (millis() - auxTimeout >= 50 && !sleepMode) {
    pinMode(AUXBUTTON, INPUT_PULLUP);
    auxTimeout = millis();
    uint16_t data = analogRead(AUXBUTTON);

    /* was 700, #10 needs 600 */
    if ( ( data < 700) || (data > 900) ) {
      auxDetection++;
      if (auxDetection > 21) auxDetection = 21;
    }
    else {
      auxDetection--;
      if (auxDetection <= 1) auxDetection = 1;
    }

    /* PROCESS LEDS */
    if (!sleepMode) {
      if (blueLedCounter > 2000) blueLedCounter = 0;

      if (batteryState == 0) {
        redON;
      } else if (batteryState == 2) {
        ledsOFF;
        analogWrite(RGB_R, 100);
      } else if (digitalRead(BT_CONNECTED) == 1) {
        blueON;
      } else if (discoveryState) { // blueLedCounter
        blueLedCounter++;
        if (blueLedCounter % 8 == 0) blueToggle;
      } else if (!AUXconnected) {
        greenON;
      } else whiteON;
    }
  }

  /**
     BUTTONS

     Process pressed buttons
  */
  if (millis() - buttonMillis >= 80) {
    char data = Serial.read();

    if (Serial.available()) {
      Serial.print((char) data);
      if (data == 'e') {
        BT_discoveryOFF();
      } else if (data == 'b') {
        BT_reconnect();
      } else if (data == 'd') {
        BT_disconnect();
      } else if (data == 'a') {
        BT_discoveryON();
      } else if (data == 'c') {
        resetBT();
      }
    }
    buttonMillis = millis();



    /**
       AUX

       TODO: What is this?!
    */
    if (auxDetection >= 20) { // 6 for normal sound
      AUXon;
      AudioOutI2S.volume(k21.volAUX);
      AUXconnected = 1;
    } else if (auxDetection < 5) { // 10 for normal sound
      SPEAKERon;
      AudioOutI2S.volume(k21.volSPEAKER);
      AUXconnected = 0;
    }
    /* END of AUX */

    prevButtons = buttons;
    buttons = 0;
    buttonReleased = 0;
    k21.pressedKeyInfo.button = 0;
    buttons = ((BR4 & 0x01) << 3 | (BR3 & 0x01) << 2 | (BR2 & 0x01) << 1 | BR1 & 0x01);

    if (prevButtons != 0 && !buttons) {
      k21.keycodeFormatting(IDLE, RELEASED);
      buttonReleased = 1;
      k21.backspaceCount = 0;
      k21.volumeAdjust = 0;
      k21.i = 0; // For removing false volume adjustments
    }

    if (!sleepMode) {
      KeyInfo tmp = k21.buttonOperations(buttons, currtouched, BR2, BR4);
      keyMode = tmp.mode;

      // Writes pressed button to SD card
      if (buttons != 0 && toggl == 0) { // toggl - IF pressed the first time
        uint8_t paused = 0;

        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.pause();
          paused = 1;
        }

        /* Write to log file */
        dataString = String(millis() - sdLog) + ";" + (String)(k21.buttonMessage);
        sdLog = millis();
        File dataFile = SD.open("k21_log.txt", FILE_WRITE);
        if (dataFile) {
          dataFile.println(dataString);
          dataFile.close();
          //          Serial.println(k21.buttonMessage); // Print to the serial port too
          k21.buttonMessage = "";
        }

        if (paused) AudioOutI2S.resume();
      }



      /**
         MODE CHANGE ACTIONS
      */
      if ( ((currtouched == 4) && ((buttons == 4) || (buttons == 2) || (buttons == 6))) ||
           ((currtouched == 128) && ((buttons == 4) || (buttons == 8) || (buttons == 12))) ||
           ((currtouched == 512) && ((buttons == 1) || (buttons == 8) || (buttons == 9))) ||
           ((currtouched == 32) && ((buttons == 1) || (buttons == 2) || (buttons == 3))) ) {

        k21.buttonPressCounter++;
      }

      /* Button hold to enable Bluetooth */
      if (k21.buttonPressCounter == 20 && !discoveryState && !digitalRead(BT_CONNECTED)) {
        k21.buttonPressCounter++;
        blueON;

        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileBTO);
        SOUNDON1;
        delay(50);
        SOUNDOFF;
        unConnectedCountBT = 0;
        BT_discoveryON();
        BT_reconnect();
        discoveryState = 1;
      }

      /* Mode change */
      if ( (previousMode != keyMode && currtouched == 4 && k21.lockButtons == 0) &&
           !sleepMode && buttonReleased && k21.buttonPressCounter <= 3 ) {

        previousMode = keyMode;
        lockButtonsTime = millis();
        k21.lockButtons++;

        whiteON;
        delay(200);

        if (k21.language == 0) {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileMOC);
        } else {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileRCHM);
        }
      }

      /* Caps Lock pressed */
      if ( (keyMode == 0) && (currtouched == 64) && (buttons == 1) && (k21.lockButtons == 0) && !sleepMode) {
        lockButtonsTime = millis();
        k21.lockButtons++;
        k21.lockCapsLock++;
      }

      /* Checks which mode is active and which button is pressed */
      if (!sleepMode) {
        if (k21.language == 0) {
          if (tmp.mode == 0 && tmp.button == 1 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileCLK);
          }
          else if (tmp.mode == 0 && tmp.button == 2 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRHT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 0 && tmp.button == 4 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileBSP);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 0 && tmp.button == 8 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileLFT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 1 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileUP);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 2 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRHT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 4 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileDWN);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 8 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileLFT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
        } else {
          if (tmp.mode == 0 && tmp.button == 1 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileCLK);
          }
          else if (tmp.mode == 0 && tmp.button == 2 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRRGH);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 0 && tmp.button == 4 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRBSP);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 0 && tmp.button == 8 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRLFT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 1 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRUP);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 2 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRRGH);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 4 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRDWN);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
          else if (tmp.mode == 1 && tmp.button == 8 && toggl == 0) {
            if (AudioOutI2S.isPlaying()) AudioOutI2S.stop();
            AudioOutI2S.play(waveFileRLFT);
            if (k21.buttonDelay) {
              delay(200);
            }
          }
        }
      }

      if (buttons) {
        toggl = 1;
      } else {
        toggl = 0;
      }

      /**
         Language change
      */
      if (k21.language != previousLanguage) {
        if (k21.language == 0) {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileENG);
        } else {
          if (AudioOutI2S.isPlaying()) {
            AudioOutI2S.stop();
          }
          AudioOutI2S.play(waveFileRRUS);
        }

        previousLanguage = k21.language;
        lockButtonsTime = millis();
        k21.lockButtons++;
      }
    }   // end of if(!sleepMode)

    /**
       TODO: SOMETHING NEEDS TO BE CLEANED HERE!
    */
    // izejam no miega režīma - wake up
    if ( ((currtouched == 64) && (buttons == 1) && (sleepMode == 1)) ||
         ((currtouched == 2) && (buttons == 2) && (sleepMode == 1)) ||
         ((currtouched == 8) && (buttons == 4) && (sleepMode == 1)) ||
         ((currtouched == 256) && (buttons == 8) && (sleepMode == 1)) ||
         ((currtouched == 4) && ((buttons == 4) || (buttons == 2) || (buttons == 6)) && (sleepMode == 1)) ||
         ((currtouched == 128) && ((buttons == 4) || (buttons == 8) || (buttons == 12)) && (sleepMode == 1)) ||
         ((currtouched == 512) && ((buttons == 1) || (buttons == 8) || (buttons == 9)) && (sleepMode == 1)) ||
         ((currtouched == 32) && ((buttons == 1) || (buttons == 2) || (buttons == 3)) && (sleepMode == 1)) ) {

      greenON;

      if (k21.language == 0) {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileWKU);
      } else {
        if (AudioOutI2S.isPlaying()) {
          AudioOutI2S.stop();
        }
        AudioOutI2S.play(waveFileRWKU);
      }
      k21.functionFormatting("dev_wku");

      firstSleepCycle = 1;
      unConnectedCountBT = 0;
      BT_reconnect();
      sleepMode = 0;
      sleepActive = 0;
      inactiveCounter = 0;
      discoveryState = 0;
      prevBTState = 0;

      vibrationON;
      delay(100);
      vibrationOFF;
      delay(200);
    }

    /* TODO: Something for long presses */
    if (!sleepMode && buttons) {
      //k21.buttonPressCounter++;
      //Serial.print("\tLong press: ");
      //Serial.println(k21.buttonPressCounter);
    }

    if (!sleepMode && !buttons && k21.buttonPressCounter > 0)
    {
      k21.languageState = 0;
      k21.numericsState = 0;
      k21.modeState = 0;
      k21.buttonPressCounter = 0;
    }
  } // END OF if (millis() - buttonMillis >= 80)




  /**
     PADS

     Process capacitive pads
  */
  if (millis() - currentMillis >= 45 ) {
    currentMillis = millis();

    // Get the currently touched pads
    currtouched = cap.touched2();

    /************* lai nebuutu false atlaishanu ieviesham "delay" atlaishanai ***************/
    if (!sleepMode) {
      if (currtouched == 0) {
        debounce++;
        k21.lCount++; // turpinam pieskaitiit peedeejam aktiivajam laukuma
        if (debounce > 10) debounce = 0;
      } else {
        debounce = 0;
        inactiveCounter = 0;
      }

      // atmetam ja nu kāda nulle gadaas
      if (debounce == 0) {
        debounce = 0;

        if (!discoveryState) {
          inactiveCounter++;
        }

        /******************* UPDATING capacity BASELINE readings ***/
        if (k21.unpresState > 20) {
          k21.unpresState = 0;
          for (uint8_t i = 0; i < 12; i++) {
            baseList[i] = 0;
          }
          for (uint8_t i = 0; i < 12; i++) {
            baseList[i] = cap.filteredData(i) ;
          }
          for (uint8_t i = 0; i < 12; i++) {
            baseList[i] = (baseList[i] + cap.filteredData(i)) / 2 ;
          }
        }
        /********************* END of UPDATING capacity BASELINE readings ***/


        /************* MAIN processing loop for pads ***************/
        if (k21.writeLetter(currtouched, buttons, AUXconnected) == 1) {
          inactiveCounter = 0;

#ifdef SOUNDEFFECT
          SOUNDON1;
          delay(4);
          SOUNDOFF;
#endif

#ifdef VIBRATIONEFFECT
          vibrationON;
          delay(50);
          vibrationOFF;
#endif

          timeVibration = millis();
          k21.i = 0;

          /****** Outputs letter via USB and/or Bluetooth *******/
          Keyboard.write(k21.letter); // USB keyboard
          Serial1.print((char)k21.letter); // Bluetooth
          delay(10);

          /****** AUDIO RESET *******/
          if (AudioOutI2S.isPlaying()) AudioOutI2S.stop(); //Šeit jāliek jo konfliktee ar SD kartes procesiem


          /****** ieraksta SD kartē info par nospiesto burtu *******/
          File dataFile = SD.open("k21_log.txt", FILE_WRITE);

          dataString = String(millis() - sdLog) + ";" + (String)((char)k21.letter);
          sdLog = millis();
          if (dataFile) {
            dataFile.println(dataString);
            dataFile.close();
            dataString = "";
          }
          /****** END ieraksta SD kartē info par nospiesto burtu *******/



          /**
             Audio playback
          */
          if (k21.language == 0) {
            if (k21.letter == 'a') {
              AudioOutI2S.play(waveFileA);
            }
            else if (k21.letter == 'b') {
              AudioOutI2S.play(waveFileB);
            }
            else if (k21.letter == 'c') {
              AudioOutI2S.play(waveFileC);
            }
            else if (k21.letter == 'd') {
              AudioOutI2S.play(waveFileD);
            }
            else if (k21.letter == 'e') {
              AudioOutI2S.play(waveFileE);
            }
            else if (k21.letter == 'f') {
              AudioOutI2S.play(waveFileF);
            }
            else if (k21.letter == 'g') {
              AudioOutI2S.play(waveFileG);
            }
            else if (k21.letter == 'h') {
              AudioOutI2S.play(waveFileH);
            }
            else if (k21.letter == 'i') {
              AudioOutI2S.play(waveFileI);
            }
            else if (k21.letter == 'j') {
              AudioOutI2S.play(waveFileJ);
            }
            else if (k21.letter == 'k') {
              AudioOutI2S.play(waveFileK);
            }
            else if (k21.letter == 'l') {
              AudioOutI2S.play(waveFileL);
            }
            else if (k21.letter == 'm') {
              AudioOutI2S.play(waveFileM);
            }
            else if (k21.letter == 'n') {
              AudioOutI2S.play(waveFileN);
            }
            else if (k21.letter == 'o') {
              AudioOutI2S.play(waveFileO);
            }
            else if (k21.letter == 'p') {
              AudioOutI2S.play(waveFileP);
            }
            else if (k21.letter == 'q') {
              AudioOutI2S.play(waveFileQ);
            }
            else if (k21.letter == 'r') {
              AudioOutI2S.play(waveFileR);
            }
            else if (k21.letter == 's') {
              AudioOutI2S.play(waveFileS);
            }
            else if (k21.letter == 't') {
              AudioOutI2S.play(waveFileT);
            }
            else if (k21.letter == 'u') {
              AudioOutI2S.play(waveFileU);
            }
            else if (k21.letter == 'v') {
              AudioOutI2S.play(waveFileV);
            }
            else if (k21.letter == 'z') {
              AudioOutI2S.play(waveFileZ);
            }
            else if (k21.letter == 'x') {
              AudioOutI2S.play(waveFileX);
            }
            else if (k21.letter == 'y') {
              AudioOutI2S.play(waveFileY);
            }
            else if (k21.letter == 'w') {
              AudioOutI2S.play(waveFileW);
            }

            else if (k21.letter == '0') {
              AudioOutI2S.play(waveFile0);
            }
            else if (k21.letter == '1') {
              AudioOutI2S.play(waveFile1);
            }
            else if (k21.letter == '2') {
              AudioOutI2S.play(waveFile2);
            }
            else if (k21.letter == '3') {
              AudioOutI2S.play(waveFile3);
            }
            else if (k21.letter == '4') {
              AudioOutI2S.play(waveFile4);
            }
            else if (k21.letter == '5') {
              AudioOutI2S.play(waveFile5);
            }
            else if (k21.letter == '6') {
              AudioOutI2S.play(waveFile6);
            }
            else if (k21.letter == '7') {
              AudioOutI2S.play(waveFile7);
            }
            else if (k21.letter == '8') {
              AudioOutI2S.play(waveFile8);
            }
            else if (k21.letter == '9') {
              AudioOutI2S.play(waveFile9);
            }

            else if (k21.letter == '&') {
              AudioOutI2S.play(waveFileAND);
            }
            else if (k21.letter == '@') {
              AudioOutI2S.play(waveFileAT);
            }
            else if (k21.letter == '*') {
              AudioOutI2S.play(waveFileATX);
            }
            else if (k21.letter == ':') {
              AudioOutI2S.play(waveFileCLN);
            }
            else if (k21.letter == ',') {
              AudioOutI2S.play(waveFileCMA);
            }
            else if (k21.letter == ')') {
              AudioOutI2S.play(waveFileCPA);
            }
            else if (k21.letter == '$') {
              AudioOutI2S.play(waveFileDLR);
            }
            else if (k21.letter == '.') {
              AudioOutI2S.play(waveFileDOT);
            }
            else if (k21.letter == '-') {
              AudioOutI2S.play(waveFileDSH);
            }
            else if (k21.letter == '\n') {
              AudioOutI2S.play(waveFileENT);
            }
            else if (k21.letter == '=') {
              AudioOutI2S.play(waveFileEQL);
            }
            else if (k21.letter == '!') {
              AudioOutI2S.play(waveFileEXM);
            }
            else if (k21.letter == '>') {
              AudioOutI2S.play(waveFileGRT);
            }
            else if (k21.letter == '#') {
              AudioOutI2S.play(waveFileHSH);
            }
            else if (k21.letter == '<') {
              AudioOutI2S.play(waveFileLET);
            }
            else if (k21.letter == '(') {
              AudioOutI2S.play(waveFileOPA);
            }
            else if (k21.letter == '%') {
              AudioOutI2S.play(waveFilePER);
            }
            else if (k21.letter == '+') {
              AudioOutI2S.play(waveFilePLS);
            }
            else if (k21.letter == '?') {
              AudioOutI2S.play(waveFileQTM);
            }
            else if (k21.letter == '"') {
              AudioOutI2S.play(waveFileQUO);
            }
            else if (k21.letter == ';') {
              AudioOutI2S.play(waveFileSEC);
            }
            else if (k21.letter == '/') {
              AudioOutI2S.play(waveFileSLH);
            }
            else if (k21.letter == 32 ) {
              AudioOutI2S.play(waveFileSPC);
            }
            else if (k21.letter == 39) {
              AudioOutI2S.play(waveFileSQT);
            }
            else if (k21.letter == '~') {
              AudioOutI2S.play(waveFileTLD);
            }
            else if (k21.letter == '_') {
              AudioOutI2S.play(waveFileUNS);
            }
          } else {
            if (k21.letter == 'f') {
              AudioOutI2S.play(waveFileRA);
            }
            else if (k21.letter == ',') {
              AudioOutI2S.play(waveFileRB);
            }
            else if (k21.letter == 'o') {
              AudioOutI2S.play(waveFileRSH);
            }
            else if (k21.letter == 'c') {
              AudioOutI2S.play(waveFileRS);
            }
            else if (k21.letter == 'q') {
              AudioOutI2S.play(waveFileRIII); // i kratkoje
            }
            else if (k21.letter == '\'') {
              AudioOutI2S.play(waveFileREE);
            }
            else if (k21.letter == 'w') {
              AudioOutI2S.play(waveFileRC);
            }
            else if (k21.letter == 'l') {
              AudioOutI2S.play(waveFileRD);
            }
            else if (k21.letter == 't') {
              AudioOutI2S.play(waveFileRE);
            }
            else if (k21.letter == 'a') {
              AudioOutI2S.play(waveFileRF);
            }
            else if (k21.letter == 'u') {
              AudioOutI2S.play(waveFileRG);
            }
            else if (k21.letter == '[') {
              AudioOutI2S.play(waveFileRH);
            }
            else if (k21.letter == 'b') {
              AudioOutI2S.play(waveFileRI);
            }
            else if (k21.letter == 's') {
              AudioOutI2S.play(waveFileRIJ);
            }
            else if (k21.letter == 'r') {
              AudioOutI2S.play(waveFileRK);
            }
            else if (k21.letter == 'k') {
              AudioOutI2S.play(waveFileRL);
            }
            else if (k21.letter == 'v') {
              AudioOutI2S.play(waveFileRM);
            }
            else if (k21.letter == 'y') {
              AudioOutI2S.play(waveFileRN);
            }
            else if (k21.letter == 'j') {
              AudioOutI2S.play(waveFileRO);
            }
            else if (k21.letter == 'g') {
              AudioOutI2S.play(waveFileRP);
            }
            else if (k21.letter == 'm') {
              AudioOutI2S.play(waveFileRII); // mjakij znak
            }
            else if (k21.letter == 'h') {
              AudioOutI2S.play(waveFileRR);
            }
            else if (k21.letter == 'c') {
              AudioOutI2S.play(waveFileRS);
            }
            else if (k21.letter == 'n') {
              AudioOutI2S.play(waveFileRT);
            }
            else if (k21.letter == 'e') {
              AudioOutI2S.play(waveFileRY);
            }
            else if (k21.letter == 'd') {
              AudioOutI2S.play(waveFileRV);
            }
            else if (k21.letter == 'p') {
              AudioOutI2S.play(waveFileRZ);
            }
            else if (k21.letter == '`') {
              AudioOutI2S.play(waveFileRJO);
            }
            else if (k21.letter == 'i') {
              AudioOutI2S.play(waveFileRSH);
            }
            else if (k21.letter == ';') {
              AudioOutI2S.play(waveFileRZH);
            }
            else if (k21.letter == 'z') {
              AudioOutI2S.play(waveFileRJA);
            }
            else if (k21.letter == 'x') {
              AudioOutI2S.play(waveFileRCH);
            }
            else if (k21.letter == '.') {
              AudioOutI2S.play(waveFileRJU);
            }
            else if (k21.letter == '#') {
              AudioOutI2S.play(waveFileRNO);
            }
            else if (k21.letter == 'q') {
              AudioOutI2S.play(waveFileRIJ);
            }
            else if (k21.letter == 't') {
              AudioOutI2S.play(waveFileRE);
            }
            else if (k21.letter == '}') {
              AudioOutI2S.play(waveFileRJJ);
            }

            else if (k21.letter == '0') {
              AudioOutI2S.play(waveFileR0);
            }
            else if (k21.letter == '1') {
              AudioOutI2S.play(waveFileR1);
            }
            else if (k21.letter == '2') {
              AudioOutI2S.play(waveFileR2);
            }
            else if (k21.letter == '3') {
              AudioOutI2S.play(waveFileR3);
            }
            else if (k21.letter == '4') {
              AudioOutI2S.play(waveFileR4);
            }
            else if (k21.letter == '5') {
              AudioOutI2S.play(waveFileR5);
            }
            else if (k21.letter == '6') {
              AudioOutI2S.play(waveFileR6);
            }
            else if (k21.letter == '7') {
              AudioOutI2S.play(waveFileR7);
            }
            else if (k21.letter == '8') {
              AudioOutI2S.play(waveFileR8);
            }
            else if (k21.letter == '9') {
              AudioOutI2S.play(waveFileR9);
            }

            else if (k21.letter ==  32) {
              AudioOutI2S.play(waveFileRSPC);
            }
            else if (k21.letter == '?') {
              AudioOutI2S.play(waveFileRCOM);
            }
            else if (k21.letter == '&') {
              AudioOutI2S.play(waveFileRQUM);
            }
            else if (k21.letter == '!') {
              AudioOutI2S.play(waveFileREXM);
            }
            else if (k21.letter == '_') {
              AudioOutI2S.play(waveFileRUSC);
            }
            else if (k21.letter == '-') {
              AudioOutI2S.play(waveFileRDSH);
            }
            else if (k21.letter == '/') {
              AudioOutI2S.play(waveFileRTCH);
            }
            else if (k21.letter == '^') {
              AudioOutI2S.play(waveFileRCLN);
            }
            else if (k21.letter == '(') {
              AudioOutI2S.play(waveFileROPA);
            }
            else if (k21.letter == ')') {
              AudioOutI2S.play(waveFileRCPA);
            }
            else if (k21.letter == '\n') {
              AudioOutI2S.play(waveFileRENT);
            }
            else if (k21.letter == '$') {
              AudioOutI2S.play(waveFileRSLN);
            }
            else if (k21.letter == '\\') {
              AudioOutI2S.play(waveFileRSLH);
            }
            else if (k21.letter == '@') {
              AudioOutI2S.play(waveFileRCOL);
            }
          }

          acceptSoundState = 1;
        } //end writeLetter --->  if(k21.writeLetter(currtouched,buttons)==1)
      }//end debounce if

      if (millis() - timeVibration > shortVIBRATION && acceptSoundState == 1) {
        acceptSoundState = 0;
        vibrationOFF;
      }


      /******* when a button is pressed on circle *********/
      if (k21.volumeAdjust == 1) {
        AudioOutI2S.play(waveFileBEP);
      }

      // Reset pad state
      lasttouched = currtouched;

    } // end of if(!sleepMode)
  }
}
