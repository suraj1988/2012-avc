#ifndef Pid_h
#define Pid_h

#include "Arduino.h"
#include <PID_v1.h>
#include "Gps.h"

#define SAMPLE_RATE 200 //millis
#define PROPORTIONAL_COEFICIENT 1.0
#define INTEGRAL_COEFICIENT 0.0
#define DIFFERENTIAL_COEFICIENT 3.0
#define PID_MAX_OUTPUT 40.0
#define PID_MIN_OUTPUT -40.0

class Pid {
  double _setpoint, _input, _output, _lastInput;
  PID pc;
//  boolean _left = false;
  public:
//  static float getCrosstrackError2(Gps *start, Gps *dest, Gps *car) { 
//    float base = sqrt(pow(dest->getLatitude() - start->getLatitude(), 2.0) +
//        pow(dest->getLongitude() - start->getLongitude(), 2.0));
//    float leg1 = sqrt(pow(car->getLatitude() - start->getLatitude(), 2.0) +
//        pow(car->getLongitude() - start->getLongitude(), 2.0));
//    float leg2 = sqrt(pow(dest->getLatitude() - car->getLatitude(), 2.0) +
//        pow(dest->getLongitude() - car->getLongitude(), 2.0));
//    float s = (base + leg1 + leg2)/2;
//    // Heron's formula
//    float height = (2.0/base) * sqrt(s * (s - leg1) * (s - leg2) * (s - base));
//    return height;
//  }

  static float getCrosstrackError(Gps *start, Gps *dest, Gps *car) {
    //x1,y1 = start
    //x2,y2 = end
    //x0,y0 = car
    //((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
    float minutes = ((dest->getLatitude() - start->getLatitude()) *
        (start->getLongitude() - car->getLongitude()) - (start->getLatitude() - car->getLatitude()) *
        (dest->getLongitude() - start->getLongitude())) /
        sqrt(pow(dest->getLatitude() - start->getLatitude(), 2) +
        pow(dest->getLongitude() - start->getLongitude(), 2));
    return minutes;
  }
  
  Pid(float minOutput, float maxOutput);
  float update (Gps *start, Gps *dest, Gps *car);
  void start();
  inline void stop() {pc.SetMode(MANUAL);}

  inline float getAdjKp() {return pc.getAdjKp();}
  inline float getAdjKi() {return pc.getAdjKi();}
  inline float getAdjKd() {return pc.getAdjKd();}
  inline float getError() {return pc.getError();}
  inline float getIntegral() {return pc.getIntegral();}
  inline float getDifferential() {return pc.getDifferential();}
  inline float isUpdated() {return pc.isUpdated();}
};

#endif
