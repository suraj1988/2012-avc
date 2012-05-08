#ifndef AvcGps_h
#define AvcGps_h

#include "Arduino.h"
#include <Streaming.h>
#include "Avc.h"

class AvcGps {
  long latitude;
  long longitude;
  float hdop;
  byte samples;
public:
  AvcGps();
  boolean sample (long lat, long lon, float h);
  
  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
};
#endif
