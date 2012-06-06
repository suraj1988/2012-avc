#ifndef Eeprom_h
#define Eeprom_h

#include "Arduino.h"
#include <EEPROM.h>
#include "Avc.h"

#define WAY_COUNT  0
#define MAX_SPEED 1 //int
#define RUN_LOCATION 3
#define LOG_TYPE   4
#define LINE_STEER 5
#define WAY_BASE   16
#define PERRY_HIGH (LOC_PERRY_HIGH + 1)  * 50

class AvcEeprom {
  static void writeLatLon (int idx, long *lat, long *lon, int loc) {
    idx = idx * 2 + 8;
    writeLong(idx, lat, loc * 50);
    writeLong(idx + 1, lon, loc * 50);
  }
  
  static void readLatLon (int idx, long *lat, long *lon, int loc) {
    idx = idx * 2 + 8;
    readLong(idx, lat, loc * 50);
    readLong(idx + 1, lon, loc * 50);
  }

public:
  static void init () {
#if RESET_RUN_LOCATIONS
    int z = 0;
    EEPROM.write(PERRY_HIGH, byte(4));
    writeInt(PERRY_HIGH + 1, &z);
    writeInt(PERRY_HIGH + 2, &z);
    long lat = 33261509, lon = -111751025;
    writeLatLon(0, &lat, &lon, PERRY_HIGH);
    lon = -111750192;
    writeLatLon(1, &lat, &lon, PERRY_HIGH);
    lat = 33261814;
    writeLatLon(2, &lat, &lon, PERRY_HIGH);
    lon = -111751025;
    writeLatLon(3, &lat, &lon, PERRY_HIGH);
#endif
  }
  static void writeLong (int idx, long *val, int offset) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      EEPROM.write((idx * sizeof(long)) + ii + offset, bytes[ii]);
    }
  }
  
  static void readLong (int idx, long *val, int offset) {
    byte *bytes = (byte *) val;
    for (int ii = 0; ii < sizeof(long); ii++) {
      bytes[ii] = EEPROM.read((idx * sizeof(long)) + ii + offset);
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
    return EEPROM.read(getRunLocation() * 50 + WAY_COUNT);
  }
  
  static void setWayCount (byte cnt) {
    EEPROM.write(getRunLocation() * 50 + WAY_COUNT, cnt);
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
  
  static byte getRunLocation () {
    return EEPROM.read(RUN_LOCATION);
  }
  
  static void setRunLocation (byte loc) {
    // locations must start at 1
    EEPROM.write(RUN_LOCATION, loc + 1);
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
    writeLatLon(idx, lat, lon, getRunLocation() * 50);
  }
  
  static void readLatLon (int idx, long *lat, long *lon) {
    readLatLon(idx, lat, lon, getRunLocation() * 50);
  }
  
  static void setRunOffset (int lat, int lon) {
    writeInt(getRunLocation() * 50 + 1, &lat);
    writeInt(getRunLocation() * 50 + 3, &lon);
  }
  
  static void getRunOffset (int *lat, int *lon) {
    readInt(getRunLocation() * 50 + 1, lat);
    readInt(getRunLocation() * 50 + 3, lon);
  }
};
#endif
