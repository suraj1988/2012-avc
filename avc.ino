#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "Logger.h"
#include <Streaming.h>
#include "AvcPid.h"
#include "AvcGps.h"
#include "AvcImu.h"
#include <LiquidCrystal.h>
#include "AvcLcd.h"
#include "AvcNav.h"
#include "Avc.h"
#include <EEPROM.h>
#include "AvcMenu.h"


SoftwareSerial navSerial(RXPIN, TXPIN);
AvcNav *nav;
AvcImu *imu;
AvcLcd *lcd;
AvcMenu *menu;

volatile unsigned long rotations = 0;
volatile unsigned long previousOdometerMicros = 0;
volatile unsigned long odometerMicrosDelta = 0;
boolean refreshLcd = false;
unsigned long previousTime = 0;
unsigned long fiftyHertzTime = 0;

void setup()
{
  Serial.begin(57600);
  navSerial.begin(14400);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MENU_SELECT_PIN, INPUT);
  pinMode(MENU_SCROLL_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT);
  //enable internal pullup resistors
  digitalWrite(MENU_SELECT_PIN, HIGH);
  digitalWrite(MENU_SCROLL_PIN, HIGH);
  digitalWrite(HALL_SENSOR_PIN, HIGH);
  nav = new AvcNav();
  imu = new AvcImu();
  lcd = new AvcLcd();
  menu = new AvcMenu(lcd, nav);
  attachInterrupt(0, countRotations, CHANGE);
}

void loop() {
  while (navSerial.available()) {
    byte c = navSerial.read();
    imu->parse(c);
    if (imu->isComplete()) {
      if (imu->isValid()) {
        switch (imu->getMode()) {
          case AvcImu::COMPASS:
            nav->updateCompass(imu);
            break;
          case AvcImu::GPS:
            nav->updateGps(imu);
            if (nav->isSampling()) {
              nav->sample(lcd);
              refreshLcd = true;
            }
            break;
          case AvcImu::IMU:
            nav->update(imu);
            break;
        }
        if (!nav->isSampling()) {
          if (odometerMicrosDelta < 0) {
            odometerMicrosDelta = 4294967295 + odometerMicrosDelta;
          }
          nav->steer();
#if LOG_NAV
          nav->print();
#endif
        }
      }
      imu->reset();
      break;
    }
  }
  if (millis() - fiftyHertzTime > 20) {
    fiftyHertzTime = millis();
    if (!nav->isSampling()) {
      nav->updateSpeed(odometerMicrosDelta * .000001);
    }
  }
  if (millis() - previousTime > 1000 / LOOP_SPEED && nav->getOdometerSpeed() < 1) {
    previousTime = millis();
    lcd->display();
    if (!nav->isSampling()) {
      menu->checkButtons(refreshLcd);
      refreshLcd = false;
    }
  }  
}
