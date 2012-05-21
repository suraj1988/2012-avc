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
            if (strcmp(pch, "COMP") == 0) {
              mode = COMPASS;
            } else if (strcmp(pch, "GPS") == 0) {
              mode = GPS;
            } else {
              mode = IMU;
              latitude = atol(pch);
            }
            break;
          case 1:
            if (mode == COMPASS) {
              heading = atoi(pch);
            } else if (mode == GPS) {
              latitude = atol(pch);
            } else if (mode == IMU) {
              longitude = atol(pch);
            }
            break;
          case 2:
            if (mode == COMPASS) {
              xOffset = atoi(pch);
            } else if (mode == GPS) {
              longitude = atol(pch);
            } else if (mode == IMU) {
              hdop = atof(pch);
            }
            break;
          case 3:
            if (mode == COMPASS) {
              yOffset = atoi(pch);
            } else if (mode == GPS) {
              hdop = atof(pch);
            } else if (mode == IMU) {
              distanceTraveled = atof(pch);
            }
            break;
          case 4:
            if (mode == COMPASS) {
              zOffset = atoi(pch);
            } else if (mode == GPS) {
              distanceTraveled = atof(pch);
            } else if (mode == IMU) {
              fixTime = atol(pch);
            }
            break;
          case 5:
            if (mode == GPS) {
              fixTime = atol(pch);
            } else if (mode == IMU) {
              speed = atof(pch);
            }
            break;
          case 6:
            if (mode == GPS) {
              speed = atof(pch);
            } else if (mode == IMU) {
              waasLock = atoi(pch);
            }
            break;
          case 7:
            if (mode == GPS) {
              waasLock = atoi(pch);
            } else if (mode == IMU) {
              heading = atoi(pch);
            }
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
  mode = IMU;
  xOffset = 0;
  yOffset = 0;
  zOffset = 0;
}

