#ifndef AvcMenu_h
#define AvcMenu_h

#include "Arduino.h"
#include <Streaming.h>
#include "AvcLcd.h"
#include "AvcNav.h"

class AvcMenu {
  const static byte total = 4;
  AvcLcd *lcd;
  AvcNav *nav;
  byte currentMenu;
  byte bounce[4];
  unsigned long refreshTime;
  boolean debouncing;
  
  int buttonPressed();
  boolean isButtonPressed (int button);

public:
  AvcMenu(AvcLcd*, AvcNav*);
  void checkButtons (boolean refresh);
};

#endif
