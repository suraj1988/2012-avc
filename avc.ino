#include <Wire.h>
#include <TinyGPS.h>
#include <HMC5883L.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include "Gps.h"
#include "Logger.h"
#include "Pid.h"
#include <Streaming.h>
#include "AvcPid.h"
#include <math.h>
#include "AvcNav.h"
#include "AvcGps.h"
#include "Avc.h"
#include "AvcImu.h"


// booleans
#define PRINT_BUTTON_STATUS 0
#define TEST_WAYPOINT_SAMPLING 0
#define LOG_WAYPOINTS 0
// for testing pid in a straight line
#define DRIVE_TO_POINT 0
#define COMPASS_ONLY 1

#define GPS_LED 8

#define SAMPLING_BLINK_MILLIS 200
#define WAAS_BLINK_MILLIS 1000
#define SAMPLING_COUNT 50
#define REORIENT_THRESHOLD 20
#define MAX_PID_DEVIATION_FROM_COMPASS 120
#define MAX_STEERING_CHANGE 5

float MAG_Heading;
int currentHeading;
int normalizedHeading;
float gpsHeading;
boolean reorient = true;

//float samplingHdop = 0;
//int sampleCount = 0;

// LED vars
int isLedOn = 0;
boolean isGpsLock = false;

Pid pid(PID_MIN_OUTPUT, PID_MAX_OUTPUT);
float pidOutput = 0.0;
boolean pidActive = false;

#define RXPIN 4
#define TXPIN 5

SoftwareSerial navSerial(RXPIN, TXPIN);
//Gps location;
AvcNav *nav;
AvcImu *imu;

void setup()
{
  Serial.begin(57600);
  navSerial.begin(9600);
  pinMode(SERVO_PIN, OUTPUT);
//  Compass_Init();
  pinMode(SET_WAYPOINT_PIN, INPUT);
  pinMode(RESET_BUTTON_PIN, INPUT);
  //enable internal pullup resistors
  digitalWrite(SET_WAYPOINT_PIN, HIGH);
  digitalWrite(RESET_BUTTON_PIN, HIGH);
//  pinMode(GPS_LED,OUTPUT);
//  Gps::init(&mySerial);
//  pinMode(6,OUTPUT);
//  digitalWrite(6,HIGH);
//  pinMode(5,OUTPUT);
//  digitalWrite(5,HIGH);
//  delay(1000);
//  digitalWrite(6,LOW);
//  digitalWrite(5,LOW);
//  for (int ii = 0; ii < WAYPOINT_COUNT; ii++) {
//    waypoints[ii] = 0;
//  }
//  distances();
nav = new AvcNav();
imu = new AvcImu();
}

void loop()
{
  while (navSerial.available()) {
    byte c = navSerial.read();
    imu->parse(c);
    if (imu->isComplete()) {
      if (imu->isValid()) {
        nav->update(imu);
        if (nav->isSampling()) {
          nav->sample();
        }
      }
      imu->reset();
      break;
    }
  }
  if (!nav->isSampling()) {
    checkButtons(nav);
    nav->steer();
    nav->printWaypoints();
  }
//  if (isSamplingGps) {
//    Gps *wp = waypoints[waypointSamplingIndex];
//    if (!wp) {
//      wp = new Gps;
//    }
//    sampleCount += wp->sample(&mySerial);
//    waypoints[waypointSamplingIndex] = wp;
//    if (sampleCount >= SAMPLING_COUNT) {
//      isSamplingGps = false;
//    }
//  } else {
//    sampleCount = 0;
//  }
//  controlGpsLed(&location);
//  // don't bother with the rest if recording a waypoint
//  if (isSamplingGps) {
//    return;
//  }
//  location.checkGps(&mySerial);
//  isGpsLock = location.hasLock();
//  nextWaypoint = location.pickWaypoint(waypoints, nextWaypoint, numWaypointsSet);
//#if COMPASS_ONLY != 0
//  if (numWaypointsSet >= 2 || DRIVE_TO_POINT == 1) {
//    if (location.hasWaypointChanged()) {
//      pid.stop();
//      pid.start();
//    }
//    pidOutput = pid.update(waypoints[(nextWaypoint - 1 + numWaypointsSet) % numWaypointsSet], waypoints[nextWaypoint], &location);
//    pidActive = true;
//  } else {
//    reorient = true;
//  }
//#endif
//#if LOG_WAYPOINTS == 1
//  if (numWaypointsSet > 0 && !isSamplingGps) {
//    for (int ii = 0; ii < numWaypointsSet; ii++) {
//      Serial.print(waypoints[ii]->getLatitude());
//      Serial.print("\t");
//      Serial.print(waypoints[ii]->getLongitude());
//      Serial.print("\t");
//    }
//    Serial.println();
//  }
//#endif
//  gpsHeading = location.getHeadingTo(waypoints[nextWaypoint]);
//  int steering = 0;
//  int useCompass = 100;
//  int steeringDiff = 0;
//  Read_Compass();
//  Compass_Heading();
//  if (gpsHeading >= 0) {
//    currentHeading = (int) gpsHeading;
//  }
//  // +360%360 makes sure the number is positive
//  // +180 reverses the direction because the compass is backward
//  normalizedHeading = (int(MAG_Heading) + 360 + 180 - currentHeading) % 360;
//  steeringDiff = normalizedHeading;
//  if(normalizedHeading >= 180) {
//    steeringDiff = 360 - normalizedHeading;
//  }
//  if ((location.hasWaypointChanged() || reorient || abs(steeringDiff) > MAX_PID_DEVIATION_FROM_COMPASS) && DRIVE_TO_POINT != 1 || COMPASS_ONLY == 1) {
//    useCompass = 100;
//    if(normalizedHeading >= 180) {
//      steering = map(normalizedHeading, 180, 360, MAX_STEERING, STEERING_CENTER);
//    } else {
//      steering = map(normalizedHeading, 180, 0, MIN_STEERING, STEERING_CENTER);
//    }
//    if (abs(steeringDiff) < REORIENT_THRESHOLD && pidActive) {
//      reorient = false;
//      pid.stop();
//      pid.start();
//    } else {
//      reorient = true;
//    }
//  } else {
//#if DRIVE_TO_POINT == 1
//    reorient = false;
//    pid.stop();
//    pid.start();
//#endif
//    useCompass = 0;
//    int prevSteering = steering;
//    if(pidOutput >= 0) {
//      steering = map(pidOutput, PID_MAX_OUTPUT, 0, MAX_STEERING/* - (MAX_STEERING - STEERING_CENTER) / 2*/, STEERING_CENTER);
//      if ((steering - prevSteering) > MAX_STEERING_CHANGE) {
//        steering = prevSteering + MAX_STEERING_CHANGE;
//      }
//    } else {
//      steering = map(pidOutput, PID_MIN_OUTPUT, 0, MIN_STEERING/* + (STEERING_CENTER - MIN_STEERING) / 2 */, STEERING_CENTER);
//      if ((prevSteering - steering) > MAX_STEERING_CHANGE) {
//        steering = prevSteering - MAX_STEERING_CHANGE;
//      }
//    }
//  }
//  analogWrite(SERVO_PIN, steering);
//#if PRINT_DATA == 1
//#if PRINT_HEADINGS == 1
//  Logger::logHeadingColumnNames();
//#endif
//  Logger::logHeadingData(waypoints[nextWaypoint], &location, currentHeading, normalizedHeading);
//#endif
//#if LOG_CTE == 1
//  if (numWaypointsSet >= 2) {
//    Logger::logCrossTrackError(waypoints[(nextWaypoint - 1 + numWaypointsSet) % numWaypointsSet], waypoints[nextWaypoint], &location, MAG_Heading, pidOutput, steering-STEERING_CENTER, useCompass, steeringDiff, pidActive, nextWaypoint * 100);
//  }
//#endif
//#if LOG_PID == 1
//  if (numWaypointsSet >= 2 && pid.isUpdated()) {
//    Logger::logPid(pid.getError(), pid.getIntegral(), pid.getDifferential(), pid.getAdjKp(), pid.getAdjKi(), pid.getAdjKd(), waypoints[(nextWaypoint - 1 + numWaypointsSet) % numWaypointsSet], waypoints[nextWaypoint], &location, pidOutput, useCompass, steeringDiff, nextWaypoint * 100);
//  }
//#endif
//  delay(50);
}
