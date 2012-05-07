#ifndef AvcPid_h
#define AvcPid_h

#include "Arduino.h"
#include <math.h>

// meters per latitude minute
// phoenix is at 33° 26' 54.2" N
// boulder is at 40° 0' 54" N
// at latitude 33 = 1848.41/min
// at latitude 34 = 1848.71/min
// at latitude 40 = 1850.58/min

// meters per longitude minute
// at latitude 33 = 1557.55/min
// at latitude 34 = 1539.75/min
// at latitude 40 = 1423.23/min


class AvcPid {
  // pi = 180 degrees
  static const float alpha = PI/4.0;
  static const float kp = 1.0;
  
  float previousError;
  
public:
  // time is in seconds, speed in meters / second
  float compute (float error, float timeDelta, float speed);
};

#endif
