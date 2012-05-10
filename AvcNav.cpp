#include "AvcNav.h"

AvcNav::AvcNav (): pid() {
  latitude = 0;
  longitude = 0;
  hdop = 0;
  distanceTraveled = 0;
  fixTime = 0;
  speed = 0;
  waasLock = false;
  heading = 0;
  waypointSamplingIndex = -1;
  numWaypointsSet = max(AvcEeprom::getWayCount(), 0);
  nextWaypoint = 0;
  bestKnownHeading = 0;
  sampling = false;
  samples = 0;
  goodHdop = true;
  reorienting = false;
}

void AvcNav::pickWaypoint() {
  if (numWaypointsSet >= 2) {
    long lat,lon;
    AvcEeprom::readLatLon (nextWaypoint, &lat, &lon);
    float distanceFromWaypoint = TinyGPS::distance_between(toFloat(latitude), 0.0f, 
        toFloat(lat), toFloat(lon - longitude));
    if (distanceFromWaypoint < WAYPOINT_RADIUS) {
      nextWaypoint = (nextWaypoint + 1) % numWaypointsSet;
    }
  }
}

void AvcNav::steer () {
  pickWaypoint();
  if (numWaypointsSet > 0) {
    int h = getHeadingToWaypoint();
    if (h >= 0) {
      bestKnownHeading = h;
    }
  } else {
    bestKnownHeading = 0;
  }
  float cte = crossTrackError();
  byte pidOffset = pid.compute(cte, (fixTime - previousFixTime)/1000.0, speed);
  int steering = STEERING_CENTER;
  if (!goodHdop || reorienting || COMPASS_ONLY) {
    int headingOffset = (heading - bestKnownHeading + 360) % 360;
    if(headingOffset >= 180) {
      steering = map(headingOffset, 180, 360, MAX_STEERING, STEERING_CENTER);
    } else {
      steering = map(headingOffset, 180, 0, MIN_STEERING, STEERING_CENTER);
    }
    goodHdop = checkHdop();
  } else { // use pid
    if(pidOffset >= 0) {
      steering = map(pidOffset, -100, 0, MAX_STEERING, STEERING_CENTER);
    } else {
      steering = map(pidOffset, 100, 0, MIN_STEERING, STEERING_CENTER);
    }
    goodHdop = checkHdop();
  }

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
  previousFixTime = fixTime;
  fixTime = imu->getFixTime();
  speed = imu->getSpeed();
  waasLock = imu->hasWaasLock();
  heading = imu->getHeading();
}

void AvcNav::sample (AvcLcd *lcd) {
  if (fixTime != previousFixTime) {
    return;
  }
  if (samples >= MAX_SAMPLES) {
    long lat = tempWaypoint->getLatitude();
    long lon = tempWaypoint->getLongitude();
    AvcEeprom::writeLatLon (numWaypointsSet - 1, &lat, &lon);
    delete tempWaypoint;
    sampling = false;
    lcd->resetMode();
    return;
  }
  samples++;
  tempWaypoint->sample(getLatitude(), getLongitude(), getHdop());
}

void AvcNav::resetWaypoints() {
  AvcEeprom::setWayCount(0);
  delete tempWaypoint;
  waypointSamplingIndex = -1;
  numWaypointsSet = 0;
  nextWaypoint = 0;
  sampling = false;
}

void AvcNav::startSampling(AvcLcd *lcd) {
  if (sampling) {
    return;
  }
  waypointSamplingIndex++;
  numWaypointsSet++;
  AvcEeprom::setWayCount(numWaypointsSet);
  sampling = true;
  samples = 0;
  tempWaypoint = new AvcGps();
  lcd->setMode(lcd->SAMPLING);
}

float AvcNav::crossTrackError () {
  long sLat, sLon, dLat, dLon;
  AvcEeprom::readLatLon (nextWaypoint, &dLat, &dLon);
  byte count = numWaypointsSet;
  AvcEeprom::readLatLon ((nextWaypoint - 1 + count) % count, &sLat, &sLon);
  int multiplier = 1;
  // this will tell me if the car is on the right or left of the line
  float plusminus = ((dLat - sLat) * (sLon - getLongitude()) - (sLat - getLatitude()) * (dLon - sLon));
  if (plusminus < 0) {
    multiplier = -1;
  }
  float a = TinyGPS::distance_between(toFloat(sLat), 0.0f, toFloat(dLat), toFloat(sLon - dLon));
  float b = TinyGPS::distance_between(toFloat(getLatitude()), 0.0f, toFloat(dLat), toFloat(getLongitude() - dLon));
  float c = TinyGPS::distance_between(toFloat(getLatitude()), 0.0f, toFloat(sLat), toFloat(getLongitude() - sLon));
  
  float cosc = (sq(a) + sq(b) - sq(c)) / (2 * a * b);
  float C = acos(cosc);
  float d = sin(C) * b;
  return float(multiplier) * d;
}

