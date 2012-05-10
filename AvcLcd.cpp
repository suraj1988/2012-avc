#include "AvcLcd.h"

AvcLcd::AvcLcd (): lcd(0) {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  time = 0;
  init = true;
  backlit = false;
  mode = NONE;
}

void AvcLcd::printHello () {
  lcd.print("hello, LCD world!");
  lcd.setBacklight(HIGH);
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

