#ifndef AvcNav_h
#define AvcNav_h

#include "Arduino.h"
#include <Streaming.h>
#include <TinyGPS.h>
#include "AvcImu.h"
#include "Avc.h"
#include "AvcPid.h"
#include "AvcLcd.h"
#include "AvcEeprom.h"
#include "Logger.h"

#define BUF_SIZE 256
#define NUM_ELEMENTS 8

class AvcNav {
  long latitude;
  long longitude;
  float hdop;
  float distanceTraveled;
  unsigned long fixTime;
  unsigned long previousFixTime;
  float speed;
  boolean waasLock;
  int heading;
//  AvcGps *waypoints[WAYPOINT_COUNT];
  AvcGps *tempWaypoint;
  int waypointSamplingIndex;
  int numWaypointsSet;
  int nextWaypoint;
  int bestKnownHeading;
  boolean sampling;
  int samples;
  AvcPid pid;
  boolean goodHdop;
  boolean reorienting;
  long sLat, sLon, dLat, dLon;
  float odometerSpeed;
  unsigned long previousOdometerCount;
  unsigned long previousOdometerMillis;
  boolean gpsUpdated;
  byte previousPidOffset;
  byte pidOffset;
  byte previousSteering;

  inline float toFloat (long fixed) {return fixed / 1000000.0;}
  inline int getHeadingToWaypoint () {
    long lat,lon;
    AvcEeprom::readLatLon (nextWaypoint, &lat, &lon);
    return (int) TinyGPS::course_to(toFloat(latitude), 0.0f, toFloat(lat), toFloat(lon - longitude));
  }
  inline boolean checkHdop() {return hdop > .0001 && hdop < 2.0;}

  float crossTrackError ();
  void pickWaypoint();

public:
  AvcNav ();
  void steer ();
  void update (AvcImu*);
  void sample (AvcLcd*);
  void resetWaypoints();
  void startSampling(AvcLcd*);
  void updateSpeed(float);
  void updateCompass(AvcImu*);
  void updateGps(AvcImu*);
  
  inline long getLatitude() {return latitude;}
  inline long getLongitude() {return longitude;}
  inline float getHdop() {return hdop;}
  inline float getDistanceTraveled() {return distanceTraveled;}
  inline unsigned long getFixTime() {return fixTime;}
  inline float getSpeed() {return speed;}
  inline boolean hasWaasLock() {return waasLock;}
  inline int getHeading() {return heading;}
  inline boolean isSampling() {return sampling;}
  inline byte getNumWaypoints() {return numWaypointsSet;}
  inline float getOdometerSpeed() {return odometerSpeed;}

#if LOG_NAV
  inline void print() {
    Serial << "NAV" << "\t" <<
        latitude << "\t" <<
        longitude << "\t" <<
        hdop << "\t" <<
        distanceTraveled << "\t" <<
        fixTime << "\t" <<
        speed << "\t" <<
        odometerSpeed << "\t" <<
        waasLock << "\t" <<
        heading <<
        endl;
  }
#endif

  inline void printWaypoints() {
    if (numWaypointsSet > 0) {
      for (int ii = 0; ii < numWaypointsSet; ii++) {
        long lat, lon;
        AvcEeprom::readLatLon (ii, &lat, &lon);
        Serial << 
            lat << "\t" <<
            lon << "\t";
      }
      Serial << endl;
    }
  }
  
};
#endif
