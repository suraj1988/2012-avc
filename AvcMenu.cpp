#include "AvcMenu.h"

AvcMenu::AvcMenu (AvcLcd *l, AvcNav *n) {
  lcd = l;
  nav = n;
  currentMenu = 0;
  refreshTime = millis();
  debouncing = false;
}

int AvcMenu::buttonPressed () {
  if (isButtonPressed(MENU_SCROLL_PIN)) {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = MENU_SCROLL_PIN;
  } else if (isButtonPressed(MENU_SELECT_PIN)) {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = MENU_SELECT_PIN;
  } else {
    for (int ii = 1; ii < 4; ii++) {
      bounce[ii-1] = bounce[ii];
    }
    bounce[3] = 0;
  }
  if (bounce[0] != bounce[3] && bounce[1] == bounce[3] && bounce[2] == bounce[3]) {
    return bounce[3];
  }
  return 0;
}

// send values SET_WAYPOINT_BUTTON or RESET_BUTTON
boolean AvcMenu::isButtonPressed (int button) {
  int buttonPressed = digitalRead(button);
  if (buttonPressed == LOW) {
    return true;
  }
  return false;
}

void AvcMenu::checkButtons (boolean refresh) {
  byte button = buttonPressed();
  if (button == MENU_SCROLL_PIN) {
    currentMenu = (currentMenu + 1) % total;
    refresh = true;
    lcd->setMode(lcd->NONE);
  } else if (button == MENU_SELECT_PIN) {
    switch(currentMenu) {
      case 1:
        nav->startSampling(lcd);
        break;
      case 2:
        nav->resetWaypoints();
        break;
      case 3:
        lcd->setMode(lcd->WAYPOINTS);
        break;
    }
    refresh = true;
  }
  if (refresh || (millis() - refreshTime) > 500 && lcd->getMode() == lcd->NONE) {
    refreshTime = millis();
    switch(currentMenu) {
      case 0:
        lcd->printGps(nav->getLatitude(), nav->getLongitude(), nav->getHdop(), refresh);
        break;
      case 1:
        if (refresh) {
          lcd->printStartSampling(nav->getNumWaypoints());
        }
        break;
      case 2:
        if (refresh) {
          lcd->askReset(nav->getNumWaypoints());
        }
        break;
      case 3:
        if (refresh) {
          lcd->askWaypointSlideshow(nav->getNumWaypoints());
        }
        break;
    }
  }
}

