#ifndef AvcLcd_h
#define AvcLcd_h

#include "Arduino.h"
#include <Streaming.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "Avc.h"

class AvcLcd {
public:
  enum Mode {NONE, SAMPLING};

  AvcLcd();
  void printHello();
  void setMode(Mode);
  void display();
  void resetMode();
  void printGps(long,long,float,boolean);
  void printStartSampling(byte waypoints);
  void askReset(byte waypoints);
  
private:
  LiquidCrystal lcd;
  Mode mode;
  unsigned long time;
  boolean init;
  boolean backlit;
  Mode previousMode;

  void sampling();
};
#endif
