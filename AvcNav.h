#ifndef AvcNav_h
#define AvcNav_h

#include "Arduino.h"
#include <Streaming.h>
#include "AvcGps.h"
#include <TinyGPS.h>
#include "AvcImu.h"
#include "Avc.h"

#define BUF_SIZE 256
#define NUM_ELEMENTS 8

class AvcNav {
  long latitude;
  long longitude;
  float hdop;
  float distanceTraveled;
  unsigned long fixTime;
  float speed;
  boolean waasLock;
  int heading;
  AvcGps *waypoints[WAYPOINT_COUNT];
  int waypointSamplingIndex;
  int numWaypointsSet;
  int nextWaypoint;
  int bestKnownHeading;
  boolean sampling;
  int samples;

public:
  AvcNav ();
  int pickWaypoint(AvcGps *waypoints[], int nextWaypoint, int totalWaypoints);
  void steer ();
  void update (AvcImu*);
  void sample ();
  void resetWaypoints();
  void startSampling();
  
  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
  inline float getDistanceTraveled() {return distanceTraveled;}
  inline unsigned long getFixTime() {return fixTime;}
  inline float getSpeed() {return speed;}
  inline boolean hasWaasLock() {return waasLock;}
  inline int getHeading() {return heading;}
  inline float toFloat (long fixed) {return fixed / 1000000.0;}
  inline int getHeadingTo (AvcGps *dest) {
    return (int) TinyGPS::course_to(toFloat(latitude), 0.0f, toFloat(dest->getLatitude()), toFloat(dest->getLongitude() - longitude));
  }
  inline boolean isSampling() {return sampling;}

  inline void print() {
    Serial << "NAV" << "\t" <<
        latitude << "\t" <<
        longitude << "\t" <<
        hdop << "\t" <<
        distanceTraveled << "\t" <<
        fixTime << "\t" <<
        speed << "\t" <<
        waasLock << "\t" <<
        heading <<
        endl;
  }
  
  inline void printWaypoints() {
    if (numWaypointsSet > 0) {
      for (int ii = 0; ii < numWaypointsSet; ii++) {
        Serial << 
            waypoints[ii]->getLatitude() << "\t" <<
            waypoints[ii]->getLongitude() << "\t";
      }
      Serial << endl;
    }
  }
};
#endif
