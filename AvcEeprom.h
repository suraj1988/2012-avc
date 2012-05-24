#ifndef Eeprom_h
#define Eeprom_h

#include "Arduino.h"
#include <EEPROM.h>

#define WAY_COUNT  0
#define MAX_SPEED 1 //int
#define LOG_ENABLE 3
#define LOG_TYPE   4
#define LINE_STEER 5
#define WAY_BASE   16

class AvcEeprom {
public:
  static void writeLong (int idx, long *val) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      EEPROM.write((idx * sizeof(long)) + ii + WAY_BASE, bytes[ii]);
    }
  }
  
  static void readLong (int idx, long *val) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      bytes[ii] = EEPROM.read((idx * sizeof(long)) + ii + WAY_BASE);
    }
  }
  
  static void writeInt (int idx, int *val) {
    byte *bytes = (byte *) val;
    EEPROM.write(idx, bytes[0]);
    EEPROM.write(idx + 1, bytes[1]);
  }

  static void readInt (int idx, int *val) {
    byte *bytes = (byte *) val;
    bytes[0] = EEPROM.read(idx);
    bytes[1] = EEPROM.read(idx + 1);
  }

  static byte getWayCount () {
    return EEPROM.read(WAY_COUNT);
  }
  
  static void setWayCount (byte cnt) {
    EEPROM.write(WAY_COUNT, cnt);
  }
  
  static float getMaxSpeed () {
    int val;
    readInt(MAX_SPEED, &val);
    float r = val  * .001;
    return r;
  }
  
  static void setMaxSpeed (float maxSpeed) {
    int val = maxSpeed * 1000.0;
    writeInt(MAX_SPEED, &val);
  }
  
  static boolean getLogEnable () {
    return EEPROM.read(LOG_ENABLE) == '1';
  }
  
  static void setLogEnable (boolean enable) {
    EEPROM.write(LOG_ENABLE, enable ? '1' : '0');
  }
  
  static byte getLogType () {
    return EEPROM.read(LOG_TYPE);
  }
  
  static void setLogType (byte type) {
    EEPROM.write(LOG_TYPE, type);
  }
  
  static boolean getLineSteer () {
    return EEPROM.read(LINE_STEER) == '1';
  }
  
  static void setLineSteer (boolean enable) {
    EEPROM.write(LINE_STEER, enable ? '1' : '0');
  }
  
  static void writeLatLon (int idx, long *lat, long *lon) {
    idx *= 2;
    writeLong(idx, lat);
    writeLong(idx + 1, lon);
  }
  
  static void readLatLon (int idx, long *lat, long *lon) {
    idx *= 2;
    readLong(idx, lat);
    readLong(idx + 1, lon);
  }
};
#endif
