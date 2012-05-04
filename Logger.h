#ifndef Logger_h
#define Logger_h

#include "Pid.h"
#define LOG_CTE 0
#define PRINT_DATA 0
#define PRINT_HEADINGS 0
#define LOG_PID 1

class Logger {
  public:
#if LOG_PID == 1
  static void logPid(float error, float integral, float differential, float kp, float ki, float kd, Gps *start, Gps *dest, Gps *car, float pid, int useCompass, int steeringDiff, int nextWaypoint) {
    Serial.print(Pid::getCrosstrackError(start, dest, car));
    Serial.print("\t");
    Serial.print(pid);
    Serial.print("\t");
    Serial.print(steeringDiff);
    Serial.print("\t");

    printFloat(error, 2);
    Serial.print("\t");
    printFloat(integral, 2);
    Serial.print("\t");
    printFloat(differential, 2);
    Serial.print("\t");
    printFloat(kp, 2);
    Serial.print("\t");
    printFloat(ki, 7);
    Serial.print("\t");
    printFloat(kd, 2);
    Serial.print("\t");

    Serial.print(useCompass);
    Serial.print("\t");
    Serial.print(nextWaypoint);
    Serial.print("\t");
    Serial.println();
  }
#endif

#if LOG_CTE == 1
  static void logCrossTrackError(Gps *start, Gps *dest, Gps *car, float magHeading, float pid, int steering,int useCompass, int steeringDiff, boolean pidActive, int nextWaypoint) {
    Serial.print(nextWaypoint);
    Serial.print("\t");
    Serial.print(Pid::getCrosstrackError(start, dest, car));
    Serial.print("\t");
    Serial.print(pid);
    Serial.print("\t");
    Serial.print(steeringDiff);
    Serial.print("\t");
    Serial.print(useCompass);
    Serial.print("\t");
    Serial.print(steering);
    Serial.print("\t");
    Serial.print(start->getLatitude());
    Serial.print("\t");
    Serial.print(start->getLongitude());
    Serial.print("\t");
    Serial.print(dest->getLatitude());
    Serial.print("\t");
    Serial.print(dest->getLongitude());
    Serial.print("\t");
    Serial.print(car->getLatitude());
    Serial.print("\t");
    Serial.print(car->getLongitude());
    Serial.print("\t");
    Serial.print(car->getHeadingTo(dest));
    Serial.print("\t");
    Serial.print(magHeading);
    Serial.print("\t");
    if (pidActive) {
      Serial.print(100);
    } else {
      Serial.print(0);
    }
    Serial.println();
  }
#endif

  static void printFloat (double number, int digits) {
    // Handle negative numbers
    if (number < 0.0) {
       Serial.print('-');
       number = -number;
    }
    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding /= 10.0;
    number += rounding;
    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long) number;
    double remainder = number - (double) int_part;
    Serial.print(int_part);
    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
      Serial.print("."); 
      // Extract digits from the remainder one at a time
      while (digits-- > 0) {
        remainder *= 10.0;
        int toPrint = int(remainder);
        Serial.print(toPrint);
        remainder -= toPrint;
      }
    } 
  }

#if PRINT_DATA == 1
  static void logHeadingData(Gps *dest, Gps *car, int currentHeading, int normHeading) {
    Serial.print(dest->getLatitude());
    Serial.print("\t");
    Serial.print(dest->getLongitude());
    Serial.print("\t");
    printFloat(car->getHeadingTo(dest), 0);
    Serial.print("\t");
    Serial.print(car->getLatitude());
    Serial.print("\t");
    Serial.print(car->getLongitude());
    Serial.print("\t");
    Serial.print(currentHeading);
    Serial.print("\t");
    Serial.print(normHeading);
    Serial.println();
  }
#endif

#if PRINT_HEADINGS == 1
  static void logHeadingColumnNames() {
    Serial.print("dLat\t");
    Serial.print("dLon\t");
    Serial.print("gps Heading\t");
    Serial.print("gps lat\t");
    Serial.print("gps lon\t");
    Serial.print("curr Head\t");
    Serial.print("norm Heading");
  }
#endif
};

#endif
