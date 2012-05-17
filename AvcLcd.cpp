#include "AvcLcd.h"

AvcLcd::AvcLcd (): lcd(0) {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  time = 0;
  init = true;
  backlit = false;
  mode = NONE;
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
  lcd << _FLOAT(lat/1000000.0, 6);
  lcd.setCursor(0, 1);
  lcd << _FLOAT(lon/1000000.0, 6);
  lcd.setCursor(12, 0);
  lcd << _FLOAT(hdop, 2);
  lcd.setBacklight(HIGH);
}

void AvcLcd::printStartSampling(byte waypoints) {
  lcd.clear();
  lcd << "START SAMPLING?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}

void AvcLcd::askReset(byte waypoints) {
  lcd.clear();
  lcd << "RESET WAYPOINTS?";
  lcd.setCursor(0, 1);
  lcd << "WAYPOINTS: " << waypoints;
  lcd.setBacklight(HIGH);
}
