#include "AvcImu.h"

AvcImu::AvcImu () {
  reset();
}

void AvcImu::parse (char c) {
  if (invalid) {
    if (c == '\n') {
      objectComplete = true;
    }
    return;
  }
  charCount++;
  if (charCount > BUF_SIZE) {
    invalid = true;
    return;
  }
  buf[charCount-1] = c;
  switch (c) {
    case '\n':
      objectComplete = true;
#if LOG_IMU
      print();
#endif
      return;
    case '\r': {
      buf[charCount-1] = NULL;
      // get checksum
      int count = atoi(&buf[checksumIdx + 1]);
      // test sum
      if (count != checksumIdx) {
        invalid = true;
        return;
      }
      char * pch;
      pch = strtok (buf,",");
      byte convertCount = 0;
      while (pch != NULL)
      {
        switch (convertCount) {
          case 0:
            latitude = atol(pch);
            break;
          case 1:
            longitude = atol(pch);
            break;
          case 2:
            hdop = atof(pch);
            break;
          case 3:
            distanceTraveled = atof(pch);
            break;
          case 4:
            fixTime = atol(pch);
            break;
          case 5:
            speed = atof(pch);
            break;
          case 6:
            waasLock = atoi(pch);
            break;
          case 7:
            heading = atoi(pch);
            break;
        }
        convertCount++;
        pch = strtok (NULL, ",*");
      }
      return;
    }
    case '*':
      checksumIdx = charCount - 1;
      return;
  }
}

void AvcImu::reset () {
  latitude = 0;
  longitude = 0;
  hdop = 0;
  distanceTraveled = 0;
  fixTime = 0;
  speed = 0;
  waasLock = false;
  heading = 0;
  checksumIdx = 0;
  charCount = 0;
  invalid  = false;
  objectComplete = false;
}

