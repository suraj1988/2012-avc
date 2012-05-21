#ifndef AvcGps_h
#define AvcGps_h

#include "Arduino.h"

class AvcGps {
  long latitude;
  long longitude;
  float hdop;
public:
  AvcGps();
  void sample (long lat, long lon, float h);
  
  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
};
#endif
