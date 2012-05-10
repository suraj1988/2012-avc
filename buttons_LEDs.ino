long ledTimer = 0;

// send values SET_WAYPOINT_BUTTON or RESET_BUTTON
int isButtonPressed (int button) {
  int buttonPressed = digitalRead(button);
  if (buttonPressed == LOW) {
    return 1;
  }
  return 0;
}

void checkButtons (AvcNav *nav, AvcLcd *lcd) {
  if (isButtonPressed(SET_WAYPOINT_PIN)) {
    nav->startSampling(lcd);
  } else if (isButtonPressed(RESET_BUTTON_PIN)) {
    nav->resetWaypoints();
  }
}
