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
  odometerSpeed = 0;
  previousOdometerCount = 0;
  previousOdometerMillis = 0;
  gpsUpdated = false;
  previousPidOffset = 0;
  previousSteering = STEERING_CENTER;
  if (numWaypointsSet > 0) {
    AvcEeprom::readLatLon (nextWaypoint, &dLat, &dLon);
    if (numWaypointsSet > 1) {
      AvcEeprom::readLatLon ((nextWaypoint - 1 + numWaypointsSet) % numWaypointsSet, &sLat, &sLon);
    }
  }
  pidOffset = 0;
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
      reorienting = true;
    }
  }
}

void AvcNav::steer () {
  pickWaypoint();
  if (numWaypointsSet > 0) {
    int h = getHeadingToWaypoint();
    if (h >= 0 && goodHdop) {
      bestKnownHeading = h;
    }
  } else {
    bestKnownHeading = 0;
  }
  float cte = crossTrackError();
  // there might be a problem with fixtime and previous fix time if this code runs a long time. or maybe even a short time.
  if (gpsUpdated) {
    pidOffset = pid.compute(cte, (fixTime - previousFixTime)/1000.0, odometerSpeed);
    previousPidOffset = pidOffset;
  }
  int steering = STEERING_CENTER;
  if (!goodHdop || reorienting || COMPASS_ONLY || odometerSpeed < 1 || DISTANCE_FROM_LINE_CORRECTION) {
    pidOffset = 0;
    int headingOffset = (heading - bestKnownHeading + 360) % 360;
    if (abs(headingOffset) < 30) {
      reorienting = false;
    }
    if(headingOffset >= 180) {
      steering = map(headingOffset, 180, 360, MAX_STEERING, STEERING_CENTER);
    } else {
      steering = map(headingOffset, 180, 0, MIN_STEERING, STEERING_CENTER);
    }
#if DISTANCE_FROM_LINE_CORRECTION
    int adj = 0;
    if (cte > 0) {
      adj = int(min(10, int(cte * 5.0)));
    } else {
      adj = int(max(-10, int(cte * 5.0)));
    }
//  Serial << bestKnownHeading << "\t";
//  Serial << heading << "\t";
//  Serial << cte << "\t";
//  Serial << adj << "\t";
//  Serial << steering - STEERING_CENTER << "\t";
    steering = constrain(steering + adj, MIN_STEERING, MAX_STEERING);
//  Serial << steering - STEERING_CENTER << endl;
#endif
    const byte maxSteeringDiff = 5;
    if (steering - previousSteering > maxSteeringDiff && steering > STEERING_CENTER) {
      steering = previousSteering + maxSteeringDiff;
    } else if (steering - previousSteering < -maxSteeringDiff && steering < STEERING_CENTER) {
      steering = previousSteering - maxSteeringDiff;
    }
    previousSteering = steering;
    goodHdop = checkHdop();
  } else { // use pid
    if (!gpsUpdated) {
      pidOffset = previousPidOffset;
    }
    if(pidOffset <= 0) {
      steering = map(pidOffset, -100, 0, MAX_STEERING, STEERING_CENTER);
    } else {
      steering = map(pidOffset, 100, 0, MIN_STEERING, STEERING_CENTER);
    }
    const byte maxSteeringDiff = 2;
    if (steering - previousSteering > maxSteeringDiff) {
      steering = previousSteering + maxSteeringDiff;
    } else if (steering - previousSteering < -maxSteeringDiff) {
      steering = previousSteering - maxSteeringDiff;
    }
    previousSteering = steering;
    goodHdop = checkHdop();
  }
#if LOG_HEADING
  Logger::logHeadingData(dLat, dLon, latitude, longitude, heading, bestKnownHeading, hdop, steering - STEERING_CENTER);
#endif

  analogWrite(SERVO_PIN, steering);
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

