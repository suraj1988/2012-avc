long ledTimer = 0;

// send values SET_WAYPOINT_BUTTON or RESET_BUTTON
int isButtonPressed (int button) {
  int buttonPressed = digitalRead(button);
  if (buttonPressed == LOW) {
    return 1;
  }
  return 0;
}

void controlGpsLed (AvcNav *nav) {
  long now = millis();
  int diff = now - ledTimer;
  if (nav->isSampling()) {
    if (diff > SAMPLING_BLINK_MILLIS || diff < 0) {
      if (isLedOn == 1) {
        digitalWrite(GPS_LED, LOW);
        isLedOn = 0;
      } else {
        digitalWrite(GPS_LED, HIGH);
        isLedOn = 1;
      }
      ledTimer = now;
    }
  } else if (nav->hasWaasLock()) {
    if (diff > WAAS_BLINK_MILLIS || diff < 0) {
      if (isLedOn == 1) {
        digitalWrite(GPS_LED, LOW);
        isLedOn = 0;
      } else {
        digitalWrite(GPS_LED, HIGH);
        isLedOn = 1;
      }
      ledTimer = now;
    }
  } else {
    digitalWrite(GPS_LED, LOW);
    isLedOn = 0;
  }
}

void checkButtons (AvcNav *nav) {
  if (isButtonPressed(SET_WAYPOINT_PIN)) {
//    if (waypointSamplingIndex == WAYPOINT_COUNT - 1 || isSamplingGps) {
//      return;
//    }
//    isSamplingGps = true;
//    waypointSamplingIndex++;
//    numWaypointsSet++;
    nav->startSampling();
  } else if (isButtonPressed(RESET_BUTTON_PIN)) {
    nav->resetWaypoints();
//    waypointSamplingIndex = -1;
//    numWaypointsSet = 0;
//    nextWaypoint = 0;
//    for (int ii = 0; ii < WAYPOINT_COUNT; ii++) {
//      delete waypoints[ii];
//      waypoints[ii] = 0;
//    }
  }
}
