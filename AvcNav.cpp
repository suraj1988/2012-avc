#include "AvcNav.h"

AvcNav::AvcNav () {
  latitude = 0;
  longitude = 0;
  hdop = 0;
  distanceTraveled = 0;
  fixTime = 0;
  speed = 0;
  waasLock = false;
  heading = 0;
  waypointSamplingIndex = -1;
  numWaypointsSet = 0;
  nextWaypoint = 0;
  bestKnownHeading = 0;
}

int AvcNav::pickWaypoint(AvcGps *waypoints[], int nextWaypoint, int totalWaypoints) {
  if (totalWaypoints >= 2) {
    float distanceFromWaypoint = TinyGPS::distance_between(toFloat(latitude), 0.0f, 
        toFloat(waypoints[nextWaypoint]->getLatitude()), toFloat(waypoints[nextWaypoint]->getLongitude() - longitude));
    if (distanceFromWaypoint < WAYPOINT_RADIUS) {
      nextWaypoint = (nextWaypoint + 1) % totalWaypoints;
    }
  }
  return nextWaypoint;
}

void AvcNav::steer () {
  Serial << heading;
  if (numWaypointsSet > 0) {
    int h = getHeadingTo(waypoints[nextWaypoint]);
    if (h >= 0) {
      bestKnownHeading = h;
    }
  }
  int headingOffset = (heading - bestKnownHeading + 360) % 360;
  Serial << "\t" << headingOffset;
  int steering = STEERING_CENTER;
  if(headingOffset >= 180) {
    steering = map(headingOffset, 180, 360, MAX_STEERING, STEERING_CENTER);
  } else {
    steering = map(headingOffset, 180, 0, MIN_STEERING, STEERING_CENTER);
  }
  Serial << "\t" << steering << endl;
  analogWrite(SERVO_PIN, steering);
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
}

void AvcNav::update (AvcImu *imu) {
  latitude = imu->getLatitude();
  longitude = imu->getLongitude();
  hdop = imu->getHdop();
  distanceTraveled = imu->getDistanceTraveled();
  fixTime = imu->getFixTime();
  speed = imu->getSpeed();
  waasLock = imu->hasWaasLock();
  heading = imu->getHeading();
}

boolean AvcNav::sample () {
  AvcGps *wp = waypoints[waypointSamplingIndex];
  if (!wp) {
    wp = new AvcGps;
  }
  return !wp->sample(getLatitude(), getLongitude(), getHdop());
}

void AvcNav::resetWaypoints() {
  waypointSamplingIndex = -1;
  numWaypointsSet = 0;
  nextWaypoint = 0;
  for (int ii = 0; ii < WAYPOINT_COUNT; ii++) {
    delete waypoints[ii];
    waypoints[ii] = 0;
  }
}
