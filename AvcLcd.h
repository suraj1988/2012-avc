#ifndef AvcLcd_h
#define AvcLcd_h

#include "Arduino.h"
#include <Streaming.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "Avc.h"
#include "AvcEeprom.h"

class AvcLcd {
public:
  enum Mode {NONE, SAMPLING, WAYPOINTS};

  AvcLcd();
  void printHello();
  void setMode(Mode);
  void display();
  void resetMode();
  void printGps(long,long,float,boolean);
  void printStartSampling(byte waypoints);
  void askReset(byte waypoints);
  void askWaypointSlideshow(byte waypoints);
  inline Mode getMode () {return mode;}
  
private:
  LiquidCrystal lcd;
  Mode mode;
  unsigned long time;
  boolean init;
  boolean backlit;
  Mode previousMode;
  byte waypointIndex;

  void sampling();
  void waypointSlideshow ();
};
#endif
