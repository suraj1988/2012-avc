#ifndef Logger_h
#define Logger_h

#include <Streaming.h>
#include "Avc.h"

class Logger {
  public:
#if LOG_HEADING
  static void logHeadingData(long dLat, long dLon, long cLat, long cLon, int carHeading, int headingToDest, float hdop, byte steering) {
    Serial <<
    _FLOAT(dLat/1000000.0, 6) << "\t" <<
    _FLOAT(dLon/1000000.0, 6) << "\t" <<
    _FLOAT(cLat/1000000.0, 6) << "\t" <<
    _FLOAT(cLon/1000000.0, 6) << "\t" <<
    headingToDest << "\t" <<
    carHeading << "\t" <<
    steering << "\t" <<
    hdop <<
    endl;
  }
#endif
};

#endif
