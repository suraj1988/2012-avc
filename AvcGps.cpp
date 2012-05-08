#include "AvcGps.h"

AvcGps::AvcGps() {
  latitude = 0;
  longitude = 0;
  hdop = 0;
}

void AvcGps::sample (long lat, long lon, float h) {
  if (hdop < .0001 || h < hdop) {
    latitude = lat;
    longitude = lon;
    hdop = h;
  }
}

