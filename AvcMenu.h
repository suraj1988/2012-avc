#ifndef AvcMenu_h
#define AvcMenu_h

#include "Arduino.h"
#include <Streaming.h>
#include "AvcLcd.h"
#include "AvcNav.h"

class AvcMenu {
//  const static byte total = 7;
  AvcLcd *lcd;
  AvcNav *nav;
  byte currentMenu;
  byte bounce[4];
  unsigned long refreshTime;
  boolean debouncing;
  
  int buttonPressed();
  boolean isButtonPressed (int button);
  
  enum MENU {GPS, SAMPLE, RESET, SLIDESHOW, SPEED, HEADING, OFFSET, MENU_RUN_LOC, ENUM_COUNT};

public:
  AvcMenu(AvcLcd*, AvcNav*);
  void checkButtons (boolean refresh);
};

#endif
