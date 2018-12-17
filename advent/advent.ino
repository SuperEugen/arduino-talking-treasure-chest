/*
 * advent calendar
 * 
 * Controls a wooden box that acts as a treasure chest used during christmas time. 
 * Every day the box can be opened once by someone with the right RFID token. The box
 * can be opened by another person (santa) with another RFID token to refill the box.
 * After refilled the box indicates that it contains another surprise with a blinking LED.
 * The LED is only blinking from midnight till box is opened and the surprise is taken.
 * The box knows the date and time of day due to its real time clock.
 * A built-in speaker plays severall audio files to greet the person and announce
 * the date.
 * The box has a servo controled lock and the lid has a built-in switch to sense that the
 * box can be locked again.
 * 
 * created by Ingo Hoffmann,    14. November 2018,  initial tests
 * updated by Ingo Hoffmann,    30. November 2018,  finite state machine
 * updated by Ingo Hoffmann,     1. December 2018,  finishing
 * 
 * Components:
 * LED:               connects to a digital pin and uses additional current limiter resistor
 * button:            connects to a digital pin and uses built-in pull-up resistor
 * servo:             connects to a PWM pin
 * Adafruit AudioFX:  connects via UART
 * Adafruit Amp:      connects to AudioFX board
 * Sparkfun RTC:      connects via I2C
 * RFID reader:       connects via SPI
 * 
 * Libraries:
 * Wire.h - Arduino I2C library
 * SoftwareSerial.h - Arduino serial library
 * SPI.h - Arduino SPI library
 * PWMServo.h - servo library using PWM instead of timer
 * SparkFunDS1307RTC.h - Sparkfun RTC library for DS1307
 * Adafruit_Soundboard.h - Adafruit library for Audio FX Soundboard
 * MFRC522.h - GitHub community RFID library
 * 
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <PWMServo.h>
#include <SparkFunDS1307RTC.h>
#include <Adafruit_Soundboard.h>
#include <MFRC522.h>

// Pin connections
#define INFO_LED    7
#define LID_BUTTON  2       // normally open (NO)
//      LID_C       GND     // common (C)
#define LOCK_SERVO  9
#define RFID_SDA    10
#define RFID_SCK    13
#define RFID_MOSI   11
#define RFID_MISO   12
#define RFID_RST    8
#define SFX_TX      5
#define SFX_RX      6
#define SFX_RST     4
//      SFX_UG      GND     // to boot into UART mode
//      RTC_SDA     A4      // I2C
//      RTC_SCL     A5      // I2C
#define RTC_SQW     3
//      AMP_IN+     SFX_GND
//      AMP_IN-     SFX_R

// servo angle
const int LockOpenPos = 120;
const int LockClosedPos = 60;
// debounce of switch
const unsigned long debounceMillis = 20;
// RFID keys
const byte RFIDMasterKey[] = {0x04, 0xFE, 0xB3, 0x7A, 0xAF, 0x48, 0x80};
const byte RFIDStandardKey[] = {0x04, 0x72, 0xD9, 0x7A, 0xAF, 0x48, 0x80};
const byte UserMaster = 1;
const byte UserStandard = 2;
const byte UserUnknown = 3;
// track numbers of AudioFX tracks
const byte AnnouncementBoxFilled = 0;                                     // box closed after opened by master
const byte AnnouncementBoxOpenedForCheck = 1;                             // box opened by master after it was already filled
const byte AnnouncementBoxClosedAfterCheck = 2;                           // box closed after opened by master for check
const byte AnnouncementBoxOpenedForCheckAlreadyPreparedForFirstUse = 3;   // box opened by master but ready for first use
const byte AnnouncementBoxClosedAfterCheckAlreadyPreparedForFirstUse = 4; // box closed after opened by master and ready for first use
const byte AnnouncementBoxOpenedForCheckAlreadyPrepared = 5;              // box opened by master but is already prepared for use
const byte AnnouncementBoxClosedAfterCheckAlreadyPrepared = 6;            // box closed after opened by master and already prepared for use
const byte AnnouncementFirsUseExplanation = 7;                            // box opened by user for the first time
const byte AnnouncementBoxClosed = 8;                                     // box closed by user
const byte AnnouncementLastDay = 9;                                       // box emptied by user on the 24th December
const byte AnnouncementBoxOpenedForFilling = 10;                          // box opened by master
const byte AnnouncementBoxOpenedForFirstFilling = 11;                     // box opened by master after new start
const byte AnnouncementUnknownKey = 12;                                   // unknown RFID key recognized (used in 4 different states)
const byte AnnouncementUserTriedEarly = 13;                               // user tried to open box after it was filled but before next day
const byte AnnouncementUserTriedAgain = 14;                               // user tried to open box after it was emptied and before it was filled again
const byte AnnouncementUserTriedOffSeason = 15;                           // user tried before 1st December or after last opening on 24th or later
const byte DaylyEarlyMorningGreeting = 16;                                // greeting after user opened box between 5:00am and 7:00am
const byte DaylyMorningGreeting = 17;                                     // greeting after user opened box between 7:0am and 11:00am
const byte DaylyGreeting = 18;                                            // greeting after user opened box all other times
const byte DaylyAnnouncement[] = {19, 20, 21, 22, 23, 24, 25, 26,         // list of track numbers for the dayly announcements
                                  27, 28, 29, 30, 31, 32, 33, 34,         // subtract 1 from the current because the array is zero based
                                  35, 36, 37, 38, 39, 40, 41, 42};
// states of the finite state machine
const byte StateOpenedForFilling = 1;
const byte StateFilled = 2;
const byte StateReadyForFirstUse = 3;
const byte StateOpenedForChecking = 4;
const byte StateReadyForUse = 5;
const byte StateOpenedForEmptiing = 6;
const byte StateEmpty = 7;
const byte StateOffSeason = 8;

// global variables
int8_t lastMinute = -1;
int8_t lastDay = -1;
int8_t lastState = -1;
volatile unsigned long lastMillis = 0;
byte userKey = UserUnknown;
byte boxState = StateOffSeason;
bool blinking = false;
bool firstUse = true;

// global object variables
PWMServo lockServo;
SoftwareSerial softSerial = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&softSerial, NULL, SFX_RST);
MFRC522 rfid(RFID_SDA, RFID_RST);

void setup() {
  Serial.begin(115200);
  softSerial.begin(9600);
  SPI.begin();

  // info LED
  pinMode(INFO_LED, OUTPUT);

  // lid button
  pinMode(LID_BUTTON, INPUT_PULLUP);

  // real time clock
  pinMode(RTC_SQW, INPUT_PULLUP);
  rtc.begin();
  rtc.writeSQW(SQW_SQUARE_1);
  // set time manually, seconds, minutes, hours, day of week (1=sunday), day, month, year (2 digits)
//  rtc.setTime(10, 55, 11, 1, 18, 11, 18); // comment out once the RTC is set!!!

  // sound board
  sfx.reset();

  // RFID reader
  rfid.PCD_Init();

  // servo
  lockServo.attach(LOCK_SERVO);
  lockServo.write(LockClosedPos);
  delay(15);
  lockServo.detach();

  // state machine start
  boxState = StateOffSeason;
}

void loop() {
  // print date and time every minute
  rtc.update();
  if(rtc.minute() != lastMinute) {  // if the minute has changed
    printTime();                    // print the new time
    lastMinute = rtc.minute();      // update lastMinute value
  }
  
  // blink info LED
  if(blinking) {
    digitalWrite(INFO_LED, digitalRead(RTC_SQW));
  } else {
    digitalWrite(INFO_LED, LOW);
  }

  // finite state machine
  switch(boxState) {
    case StateOpenedForFilling:
      if(lastState != boxState) {
        Serial.println("State: Openend for filling");
        lastState = boxState;
      }
      // lid closed?
      if(digitalRead(LID_BUTTON) == LOW) {
        playTrack(AnnouncementBoxFilled);
        lockLid();
        boxState = StateFilled;
      }
      break;
    case StateFilled:
      if(lastState != boxState) {
        Serial.println("State: Filled");
        lastState = boxState;
      }
      // december and new day and first use?
      if((rtc.month() == 12) && (rtc.date() != lastDay) && firstUse) {
        lastDay = rtc.date();
        blinking = true;
        boxState = StateReadyForFirstUse;
        break;
      }
      // december and new day?
      if((rtc.month() == 12) && (rtc.date() != lastDay)) {
        blinking = true;
        boxState = StateReadyForUse;
        break;
      }
      if(rfid.PICC_IsNewCardPresent()) {
        if(rfid.PICC_ReadCardSerial()) {
          userKey = readRFIDKey();
          switch(userKey) {
            case UserMaster:
              // RFID master?
              playTrack(AnnouncementBoxOpenedForCheck);
              unlockLid();
              delay(5000);
              boxState = StateOpenedForChecking;
              break;
            case UserStandard:
              // RFID user?
              playTrack(AnnouncementUserTriedEarly);
              boxState = StateFilled;
              break;
           case UserUnknown:
              // RFID unknown?
              playTrack(AnnouncementUnknownKey);
              boxState = StateFilled;
              break;  
          }
        }
      }
      break;
    case StateReadyForFirstUse:
      if(lastState != boxState) {
        Serial.println("State: Ready for first use");
        lastState = boxState;
      }
      if(rfid.PICC_IsNewCardPresent()) {
        if(rfid.PICC_ReadCardSerial()) {
          userKey = readRFIDKey();
          switch(userKey) {
            case UserMaster:
              // RFID master?
              playTrack(AnnouncementBoxOpenedForCheckAlreadyPreparedForFirstUse);
              unlockLid();
              delay(5000);
              boxState = StateOpenedForChecking;
              break;
            case UserStandard:
              // RFID user?
              playTrack(AnnouncementFirsUseExplanation);
              delay(30000);
              firstUse = false;
              lastDay = rtc.date();
              unlockLid();
              delay(5000);
              boxState = StateOpenedForEmptiing;
              break;
           case UserUnknown:
              // RFID unknown?
              playTrack(AnnouncementUnknownKey);
              boxState = StateReadyForFirstUse;
              break;  
          }
        }
      }
      break;
    case StateOpenedForChecking:
      if(lastState != boxState) {
        Serial.println("State: Openend for checking");
        lastState = boxState;
      }
      // lid closed and first use?
      if((digitalRead(LID_BUTTON) == LOW) && firstUse) {
        playTrack(AnnouncementBoxClosedAfterCheckAlreadyPreparedForFirstUse);
        lockLid();
        boxState = StateReadyForFirstUse;
        break;
      }
      // lid closed and new day?
      if((digitalRead(LID_BUTTON) == LOW) && (rtc.date() != lastDay)) {
        playTrack(AnnouncementBoxClosedAfterCheckAlreadyPrepared);
        lockLid();
        boxState = StateReadyForUse;
        break;
      }
      // lid closed?
      if(digitalRead(LID_BUTTON) == LOW) {
        playTrack(AnnouncementBoxClosedAfterCheck);
        lockLid();
        boxState = StateFilled;
        break;
      }
    case StateReadyForUse:
      if(lastState != boxState) {
        Serial.println("State: Ready for use");
        lastState = boxState;
      }
      if(rfid.PICC_IsNewCardPresent()) {
        if(rfid.PICC_ReadCardSerial()) {
          userKey = readRFIDKey();
          switch(userKey) {
            case UserMaster:
              // RFID master?
              playTrack(AnnouncementBoxOpenedForCheckAlreadyPrepared);
              unlockLid();
              delay(5000);
              boxState = StateOpenedForChecking;
              break;
            case UserStandard:
              // RFID user?
              doDailyAnnouncement();
              lastDay = rtc.date();
              unlockLid();
              delay(5000);
              boxState = StateOpenedForEmptiing;
              break;
           case UserUnknown:
              // RFID unknown?
              playTrack(AnnouncementUnknownKey);
              boxState = StateReadyForUse;
              break;  
          }
        }
      }
      break;
    case StateOpenedForEmptiing:
      if(lastState != boxState) {
        Serial.println("State: Openend for emptiing");
        lastState = boxState;
      }
      // lid closed?
      if(digitalRead(LID_BUTTON) == LOW) {
        playTrack(AnnouncementBoxClosed);
        lockLid();
        blinking = false;
        boxState = StateEmpty;
      }
      break;
    case StateEmpty:
      if(lastState != boxState) {
        Serial.println("State: Empty");
        lastState = boxState;
      }
      // last day?
      if(rtc.date() == 24) {
        playTrack(AnnouncementLastDay);
        boxState = StateOffSeason;
        break;        
      }
      if(rfid.PICC_IsNewCardPresent()) {
        if(rfid.PICC_ReadCardSerial()) {
          userKey = readRFIDKey();
          switch(userKey) {
            case UserMaster:
              // RFID master?
              playTrack(AnnouncementBoxOpenedForFilling);
              unlockLid();
              delay(5000);
              boxState = StateOpenedForFilling;
              break;
            case UserStandard:
              // RFID user?
              playTrack(AnnouncementUserTriedAgain);
              boxState = StateEmpty;
              break;
           case UserUnknown:
              // RFID unknown?
              playTrack(AnnouncementUnknownKey);
              boxState = StateEmpty;
              break;  
          }
        }
      }
      break;
    case StateOffSeason:
      if(lastState != boxState) {
        Serial.println("State: Off season");
        lastState = boxState;
      }
      if(rfid.PICC_IsNewCardPresent()) {
        if(rfid.PICC_ReadCardSerial()) {
          userKey = readRFIDKey();
          switch(userKey) {
            case UserMaster:
              // RFID master?
              playTrack(AnnouncementBoxOpenedForFirstFilling);
              unlockLid();
              delay(6000);
              if((rtc.month() == 12) && (rtc.date() == 1)) {
                firstUse = true;
                Serial.println("First use");
              } else {
                firstUse = false;
                Serial.println("Skip first use");
              }
              boxState = StateOpenedForFilling;
              break;
            case UserStandard:
              // RFID user?
              playTrack(AnnouncementUserTriedOffSeason);
              boxState = StateOffSeason;
              break;
           case UserUnknown:
              // RFID unknown?
              playTrack(AnnouncementUnknownKey);
              boxState = StateOffSeason;
              break;  
          }
        }
      } 
      break;
  }
}

void doDailyAnnouncement() {
  uint32_t current;
  uint32_t total;
  
  if((rtc.hour() > 5) && (rtc.hour() < 7)) {
    playTrack(DaylyEarlyMorningGreeting);
  } else if((rtc.hour() > 5) && (rtc.hour() < 11)) {
    playTrack(DaylyMorningGreeting);
  } else {
    playTrack(DaylyGreeting);
  }
  delay(5000);
  playTrack(DaylyAnnouncement[rtc.date() - 1]);
}

void playTrack(byte track) {
  if(! sfx.playTrack(track)) {
    Serial.print("Failed to play track ");
    Serial.println(track);
  }   
}

void lockLid() {
  lockServo.attach(LOCK_SERVO);
  lockServo.write(LockClosedPos);
  delay(500);
  lockServo.detach();  
}

void unlockLid() {
  lockServo.attach(LOCK_SERVO);
  lockServo.write(LockOpenPos);
  delay(500);
  lockServo.detach();

}

byte readRFIDKey() {
  if(rfid.uid.size == sizeof(RFIDMasterKey)) {
    if(memcmp(rfid.uid.uidByte, RFIDMasterKey, sizeof(RFIDMasterKey)) == 0) {
      return UserMaster;
    }
  }
  if(rfid.uid.size == sizeof(RFIDStandardKey)) {
    if(memcmp(rfid.uid.uidByte, RFIDStandardKey, sizeof(RFIDStandardKey)) == 0) {
      return UserStandard;
    }
  }
  return UserUnknown;
}

void printTime() {
  switch(rtc.day()) {                           // print weekday
    case 1: Serial.print("Sonntag"); break;
    case 2: Serial.print("Montag"); break;
    case 3: Serial.print("Dienstag"); break;
    case 4: Serial.print("Mittwoch"); break;
    case 5: Serial.print("Donnerstag"); break;
    case 6: Serial.print("Freitag"); break;
    case 7: Serial.print("Samstag"); break;
  }
  Serial.print(", ");
  Serial.print(String(rtc.date()) + ".");       // print day
  if (rtc.month() < 10) {
    Serial.print('0');                          // print leading '0' for month
  }
  Serial.print(String(rtc.month()) + ".");      // print month
  Serial.print("20" + String(rtc.year()));      // print year with leading "20"
  Serial.print(", ");
  Serial.print(String(rtc.hour()) + ":");       // print hour
  if (rtc.minute() < 10) {
    Serial.print('0');                          // print leading '0' for minute
  }
  Serial.print(String(rtc.minute()) + " Uhr");  // print minute
  Serial.println();
}
