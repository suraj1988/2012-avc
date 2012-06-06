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
  bestKnownHeading = 0;
  sampling = false;
  samples = 0;
  goodHdop = true;
  reorienting = false;
  odometerSpeed = 0;
  previousOdometerCount = 0;
  previousOdometerMillis = 0;
  gpsUpdated = false;
  previousPidOffset = 0;
  previousSteering = SERVO_CENTER;
  runLocation = AvcEeprom::getRunLocation();
  if (runLocation == 255) runLocation = 1;
  updateWaypoints();
  pidOffset = 0;
#if USE_SERVO_LIBRARY
  steeringServo.attach(SERVO_PIN);
  speedServo.attach(MOTOR_PIN);
#endif
  previousCte = 0;
  maxSpeed = AvcEeprom::getMaxSpeed();
  previousSpeed = 0;
  steerHeading = 0;
}

void AvcNav::updateWaypoints() {
  numWaypointsSet = max(AvcEeprom::getWayCount(), 0);
  nextWaypoint = 0;
  if (numWaypointsSet > 0) {
    AvcEeprom::readLatLon (nextWaypoint, &dLat, &dLon);
    if (numWaypointsSet > 1) {
      AvcEeprom::readLatLon ((nextWaypoint - 1 + numWaypointsSet) % numWaypointsSet, &sLat, &sLon);
    }
  }
}

void AvcNav::pickWaypoint() {
  if (numWaypointsSet >= 2) {
    float distanceFromWaypoint = TinyGPS::distance_between(toFloat(latitude), 0.0f, 
        toFloat(dLat), toFloat(dLon - longitude));
    float distanceBetweenWaypoints = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(dLat), toFloat(dLon - sLon));
    float distanceFromStartWaypoint = TinyGPS::distance_between(toFloat(sLat), 0.0f, 
        toFloat(latitude), toFloat(longitude - sLon));
    if (distanceFromWaypoint < WAYPOINT_RADIUS || distanceFromStartWaypoint > distanceBetweenWaypoints) {
      nextWaypoint = (nextWaypoint + 1) % numWaypointsSet;
      sLat = dLat;
      sLon = dLon;
      AvcEeprom::readLatLon (nextWaypoint, &dLat, &dLon);
      int latOffset, lonOffset;
      AvcEeprom::getRunOffset (&latOffset, &lonOffset);
      dLat += latOffset;
      dLon += lonOffset;
      reorienting = true;
    }
  }
}

void AvcNav::steer (AvcImu::Mode imuMode) {
  pickWaypoint();
  if (numWaypointsSet > 0) {
    int h = getHeadingToWaypoint();
    if (h >= 0 && goodHdop) {
      bestKnownHeading = h;
      steerHeading = bestKnownHeading;
#if USE_LINE_INTERSECT
//      if (!reorienting) {
        steerHeading = lineCircle(latitude, longitude, sLat, sLon, dLat, dLon);
//      }
#endif
    } else {
      steerHeading = bestKnownHeading;
    }
  } else {
    bestKnownHeading = 0;
  }
//  float cte = crossTrackError();
  // there might be a problem with fixtime and previous fix time if this code runs a long time. or maybe even a short time.
//  if (gpsUpdated) {
//    pidOffset = pid.compute(cte, (fixTime - previousFixTime)/1000.0, odometerSpeed);
//    previousPidOffset = pidOffset;
//  }
  int steering = SERVO_CENTER;
  int adj = 0;
  if (!goodHdop || reorienting || COMPASS_ONLY || odometerSpeed < 1 || DISTANCE_FROM_LINE_CORRECTION) {
    pidOffset = 0;
    int headingOffset = (heading - steerHeading + 360) % 360;
    if (abs(heading - bestKnownHeading) < 30) {
      reorienting = false;
    }
#if AGRESSIVE_STEERING
    if(headingOffset >= 180) {
//      headingOffset = constrain(360 - (360 - headingOffset) * 2, 180, 360);
      
      steering = map(headingOffset, 180, 360, MAX_SERVO, SERVO_CENTER);
    } else {
//      headingOffset = constrain(headingOffset * 2, 0, 180);
      steering = map(headingOffset, 180, 0, MIN_SERVO, SERVO_CENTER);
    }
#else
    if(headingOffset >= 180) {
      steering = map(headingOffset, 180, 360, MAX_SERVO, SERVO_CENTER);
    } else {
      steering = map(headingOffset, 180, 0, MIN_SERVO, SERVO_CENTER);
    }
#endif
    byte maxSteeringDiff = percentOfServo(.75);//(.05);
//#if DISTANCE_FROM_LINE_CORRECTION
////  float cte = crossTrackError();
//    // if approaching the line then allow for faster steering
////    if (abs(cte) < abs(previousCte)) {
//      maxSteeringDiff = percentOfServo(.3);
////    } 
//    if (cte > 0) {
//      adj = int(min(percentOfServo(.2), int(cte * percentOfServo(.05))));
//    } else {
//      adj = int(max(-percentOfServo(.2), int(cte * percentOfServo(.05))));
//    }
//    previousCte = cte;
////  Serial << bestKnownHeading << "\t";
////  Serial << heading << "\t";
////  Serial << cte << "\t";
////  Serial << adj << "\t";
////  Serial << steering - SERVO_CENTER << "\t";
//    steering = constrain(steering + adj, MIN_SERVO, MAX_SERVO);
////  Serial << steering - SERVO_CENTER << endl;
//#endif
//    if (steering - previousSteering > maxSteeringDiff && steering > SERVO_CENTER) {
//      steering = previousSteering + maxSteeringDiff;
//    } else if (steering - previousSteering < -maxSteeringDiff && steering < SERVO_CENTER) {
//      steering = previousSteering - maxSteeringDiff;
//    }
    previousSteering = steering;
//#if !COMPASS_ONLY
//  } else { // use pid
//    if (!gpsUpdated) {
//      pidOffset = previousPidOffset;
//    }
//    if(pidOffset <= 0) {
//      steering = map(pidOffset, -100, 0, MAX_SERVO, SERVO_CENTER);
//    } else {
//      steering = map(pidOffset, 100, 0, MIN_SERVO, SERVO_CENTER);
//    }
//    const byte maxSteeringDiff = 2;
//    if (steering - previousSteering > maxSteeringDiff) {
//      steering = previousSteering + maxSteeringDiff;
//    } else if (steering - previousSteering < -maxSteeringDiff) {
//      steering = previousSteering - maxSteeringDiff;
//    }
//    previousSteering = steering;
//    goodHdop = checkHdop();
//#endif
  }
  goodHdop = checkHdop();
#if LOG_HEADING
  logHeadingData(bestKnownHeading, steering - SERVO_CENTER, cte, adj);
#endif

#if USE_SERVO_LIBRARY
#if GO_STRAIGHT
  steeringServo.writeMicroseconds(SERVO_CENTER);
#else
  steeringServo.writeMicroseconds(steering);
#endif
#else
  analogWrite(SERVO_PIN, steering);
#endif
}

void AvcNav::update (AvcImu *imu) {
  gpsUpdated = false;
  if (imu->getHdop() > .0001 && imu->getFixTime() != fixTime) {
    updateGps(imu);
  }
  updateCompass(imu);
}

void AvcNav::updateCompass (AvcImu *imu) {
  gpsUpdated = false;
  heading = imu->getHeading();
}

void AvcNav::updateGps (AvcImu *imu) {
#if LOG_MAPPER
  logMapper();
#endif
  latitude = imu->getLatitude();
  longitude = imu->getLongitude();
  hdop = imu->getHdop();
  distanceTraveled = imu->getDistanceTraveled();
  previousFixTime = fixTime;
  fixTime = imu->getFixTime();
  speed = imu->getSpeed();
  waasLock = imu->hasWaasLock();
  gpsUpdated = true;
}

void AvcNav::sample (AvcLcd *lcd) {
  if (fixTime == previousFixTime || hdop < .0001) {
    return;
  }
  if (samples >= MAX_SAMPLES) {
    long lat = tempWaypoint->getLatitude();
    long lon = tempWaypoint->getLongitude();
    AvcEeprom::writeLatLon (numWaypointsSet - 1, &lat, &lon);
    if (numWaypointsSet == 1) {
      dLat = lat;
      dLon = lon;
    } else if (numWaypointsSet > 2) {
      sLat = lat;
      sLon = lon;
    } 
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
  AvcEeprom::setRunOffset (0, 0);
  delete tempWaypoint;
  waypointSamplingIndex = -1;
  numWaypointsSet = 0;
  nextWaypoint = 0;
  sampling = false;
  sLat = sLon = dLat = dLon = 0;
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

void AvcNav::updateSpeed(float timeDelta) {
  odometerSpeed = (.1222 / 2.0) / timeDelta;
}

void AvcNav::setSpeed(float fraction) {
  if (abs(previousSpeed - fraction) > .0001) {
    previousSpeed = fraction;
    float p = constrain(fraction, 0.0, 1.0);
    speedServo.writeMicroseconds(1500 + int(p * 500.0));
  }
}

void AvcNav::setMaxSpeed() {
  float pot = AvcLcd::getPotSpeed(200);
  maxSpeed = pot;
  AvcEeprom::setMaxSpeed(maxSpeed);
}

void AvcNav::drive () {
  if (maxSpeed > 0) {
    setSpeed(maxSpeed);
  } else {
    setSpeed(.25);
  }
}

#if USE_LINE_INTERSECT
// Compute bearing to point that intersects circle of radius nn
int AvcNav::lineCircle(long cLat, long cLon, long sLat, long sLon, long dLat, long dLon) {
  sLat -= cLat;
  sLon -= cLon;
  dLat -= cLat;
  dLon -= cLon;
  float feet = 40;
  float radius = (60 * feet) / 31000000;
  float dx1 = toFloat(dLon - sLon);
  float dy1 = toFloat(dLat - sLat);
  // compute the euclidean distance between A and B
  float lab = sqrt(dx1 * dx1 + dy1 * dy1);
  // compute the direction vector D from A to B
  float dvx = dx1 / lab;
  float dvy = dy1 / lab;
  // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
  float t = dvx * toFloat(0.0 - sLon) + dvy * toFloat(0.0 - sLat);
  // compute the coordinates of the point E on line and closest to C
  float ex = t * dvx + toFloat(sLon);
  float ey = t * dvy + toFloat(sLat);
  // compute the euclidean distance from E to C
  float lec = sqrt(ex * ex + ey * ey);
  // test if the line intersects the circle
  if (lec < radius) {
    // compute distance from t to circle intersection point
    float dt = sqrt(radius * radius - lec * lec);
    float iLon = (t + dt) * dvx + toFloat(sLon);
    float iLat = (t + dt) * dvy + toFloat(sLat);
    return (int) TinyGPS::course_to(0.0, 0.0, iLat, iLon);
  }
  // Return bearing from car to point where perpendicular intersects line
  return (int) TinyGPS::course_to(0.0, 0.0, ey, ex);
}

// Compute bearing to point that intersects circle of radius nn
int AvcNav::lineCircle() {
  
//  sLat -= cLat;
//  sLon -= cLon;
//  dLat -= cLat;
//  dLon -= cLon;
  float feet = 15;
  float radius = (60 * feet) / 31000000;
  float distanceBetweenWaypoints = abs(TinyGPS::distance_between(toFloat(sLat), 0.0f, 
      toFloat(dLat), toFloat(dLon - sLon)));
  float dx1 = toFloat(dLon - sLon);
  float dy1 = toFloat(dLat - sLat);
//  // compute the euclidean distance between A and B
//  float lab = sqrt(dx1 * dx1 + dy1 * dy1);
  // compute the direction vector D from A to B
  float dvx = dx1 / distanceBetweenWaypoints;
  float dvy = dy1 / distanceBetweenWaypoints;
  // Now the line equation is x = Dx*t + Ax, y = Dy*t + Ay with 0 <= t <= 1.
  float t = dvx * toFloat(longitude - sLon) + dvy * toFloat(latitude - sLat);
  // compute the coordinates of the point E on line and closest to C
  float ex = t * dvx + toFloat(sLon - longitude);
  float ey = t * dvy + toFloat(sLat);
  // compute the euclidean distance from E to C
  float distanceToLine = abs(TinyGPS::distance_between(toFloat(latitude), 0.0f, 
      toFloat(ey), toFloat(longitude - ex)));
//  float lec = sqrt(ex * ex + ey * ey);
  // test if the line intersects the circle
  if (distanceToLine < radius) {
    // compute distance from t to circle intersection point
    float dt = sqrt(radius * radius - distanceToLine * distanceToLine);
    float iLon = (t + dt) * dvx + toFloat(longitude - sLon);
    float iLat = (t + dt) * dvy + toFloat(sLat);
    return (int) TinyGPS::course_to(latitude, 0.0, iLat, iLon);
  }
  // Return bearing from car to point where perpendicular intersects line
  return (int) TinyGPS::course_to(latitude, 0.0, ey, ex);
}
#endif

int AvcNav::getLatPotentialOffset () {
  long eLat, eLon;
  AvcEeprom::readLatLon(0, &eLat, &eLon);
  return latitude - eLat;
}

int AvcNav::getLonPotentialOffset () {
  long eLat, eLon;
  AvcEeprom::readLatLon(0, &eLat, &eLon);
  return longitude - eLon;
}

void AvcNav::setOffset () {
  long eLat, eLon;
  AvcEeprom::readLatLon(0, &eLat, &eLon);
  int oLat, oLon;
  oLat = latitude - eLat;
  oLon = longitude - eLon;
  AvcEeprom::setRunOffset(oLat, oLon);
}

void AvcNav::nextRunLocation () {
  runLocation = (runLocation + 1) % LOC_COUNT;
  AvcEeprom::setRunLocation(runLocation);
  updateWaypoints();
  pickWaypoint();
}
