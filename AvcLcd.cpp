#include "AvcLcd.h"

AvcLcd::AvcLcd (): lcd(0) {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  time = 0;
  init = true;
  backlit = false;
  mode = NONE;
  previousMode = NONE;
  waypointIndex = 0;
}

void AvcLcd::setMode (Mode m) {
  previousMode = mode;
  mode = m;
  init = true;
  if (mode == NONE) {
    lcd.clear();
  }
}

void AvcLcd::display () {
  switch(mode) {
    case SAMPLING:
      sampling();
      break;
    case WAYPOINTS:
      waypointSlideshow();
      break;
  }
}

void AvcLcd::sampling () {
  if (init) {
    lcd.clear();
    lcd.print("SAMPLING");
    init = false;
    time = millis();
    lcd.setBacklight(HIGH);
    backlit = true;
  }
  if ((millis() - time) > 200) {
    if (backlit) {
      lcd.setBacklight(LOW);
      backlit = false;
    } else {
      lcd.setBacklight(HIGH);
      backlit = true;
    }
    time = millis();
  } 
}

void AvcLcd::resetMode () {
  mode = previousMode;
  display();
}

void AvcLcd::printGps (long lat, long lon, float hdop, boolean refresh) {
  if (refresh) {
    lcd.clear();
  }
  lcd.home();
  lcd << _FLOAT(lat/1000000.0, 6);
  lcd.setCursor(0, 1);
  lcd << _FLOAT(lon/1000000.0, 6);
  lcd.setCursor(12, 0);
  lcd << _FLOAT(hdop, 2);
  lcd.setBacklight(HIGH);
}

void AvcLcd::printStartSampling(byte waypoints) {
  lcd.clear();
  lcd.home();
  lcd << "START SAMPLING?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::askReset(byte waypoints) {
  lcd.clear();
  lcd.home();
  lcd << "RESET WAYPOINTS?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::askWaypointSlideshow(byte waypoints) {
  lcd.clear();
  lcd << "SHOW WAYPOINTS?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::waypointSlideshow () {
  if (init) {
    lcd.clear();
    init = false;
    time = millis();
    waypointIndex = 0;
    lcd.setBacklight(HIGH);
  }
  if ((millis() - time) > 1000) {
    byte count = AvcEeprom::getWayCount();
    long lat, lon;
    AvcEeprom::readLatLon (waypointIndex, &lat, &lon);
    lcd.home();
    lcd << _FLOAT(lat/1000000.0, 6);
    lcd.setCursor(0, 1);
    lcd << _FLOAT(lon/1000000.0, 6);
    lcd.setCursor(12, 0);
    lcd << waypointIndex;
    time = millis();
    waypointIndex = (waypointIndex + 1) % count;
  } 
}

