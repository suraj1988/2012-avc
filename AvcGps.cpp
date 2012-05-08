#include "AvcGps.h"

AvcGps::AvcGps() {
  latitude = 0;
  longitude = 0;
  hdop = 0;
  samples = 0;
}

boolean AvcGps::sample (long lat, long lon, float h) {
  if (samples >= MAX_SAMPLES) {
    return false;
  }
  samples++;
  if (hdop < .0001 || h < hdop) {
    latitude = lat;
    longitude = lon;
    hdop = h;
  }
  return true;
}

