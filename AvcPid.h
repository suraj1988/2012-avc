#ifndef AvcPid_h
#define AvcPid_h

#include "Arduino.h"
#include <math.h>

class AvcPid {
  // pi = 180 degrees
  static const float alpha = PI/4.0;
  static const float kp = 1.0;
  
  float previousError;
  
public:
  // time is in seconds, speed in meters / second
  int compute (float error, float timeDelta, float speed);
};

#endif
