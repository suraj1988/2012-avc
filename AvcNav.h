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

#if USE_SERVO_LIBRARY
#include <Servo.h>
#endif

#define BUF_SIZE 256

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
  int previousSteering;
#if USE_SERVO_LIBRARY
  Servo steeringServo;
  Servo speedServo;
#endif
  float previousCte;
  float maxSpeed;
  float previousSpeed;

  inline float toFloat (long fixed) {return fixed / 1000000.0;}
  inline int getHeadingToWaypoint () {
    long lat,lon;
    AvcEeprom::readLatLon (nextWaypoint, &lat, &lon);
    return (int) TinyGPS::course_to(toFloat(latitude), 0.0f, toFloat(lat), toFloat(lon - longitude));
  }
  inline boolean checkHdop() {return hdop > .0001 && hdop < 2.0;}
  inline float percentOfServo (float percent) {return (MAX_SERVO - SERVO_CENTER) * percent;}

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
  //pass value between 0 and 1
  void setSpeed(float);
  void setMaxSpeed();
  void drive();
  
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
  inline float getMaxSpeed() {return maxSpeed;}

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
  
#if LOG_HEADING
  void logHeadingData(int headingToDest, int steering, float cte, int steeringAdj) {
    Serial <<
    _FLOAT(dLat/1000000.0, 6) << "\t" <<
    _FLOAT(dLon/1000000.0, 6) << "\t" <<
    _FLOAT(latitude/1000000.0, 6) << "\t" <<
    _FLOAT(longitude/1000000.0, 6) << "\t" <<
    headingToDest << "\t" <<
    heading << "\t" <<
    cte << "\t" <<
    steering << "\t" <<
    steeringAdj << "\t" <<
    hdop <<
    nextWaypoint << "\t" <<
    endl;
  }
#endif
};
#endif
