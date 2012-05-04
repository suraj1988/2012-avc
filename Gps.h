#ifndef Gps_h
#define Gps_h

#include "Arduino.h"
#include <TinyGPS.h>
#include <SoftwareSerial.h>

// radius of waypoint in meters
#define WAYPOINT_RADIUS 3.0
#define PRINT_GPS 0

class Gps {
  long latitude;
  long longitude;
  float hdop;
  char inStatus;
  boolean lock;
  boolean _waasLock;
  int           idx;
  TinyGPS       gps;
  float toFloat(long fixed);
  boolean _waypointChanged;
  public:
  Gps();
  void checkGps(SoftwareSerial *serial);
  boolean checkGps(SoftwareSerial *serial, boolean sample);
  int pickWaypoint(Gps *waypoints[], int nextWaypoint, int totalWaypoints);
  long getLatitude();
  long getLongitude();
  int sample(SoftwareSerial *serial);
  boolean hasLock();
  int getHeadingTo (Gps *dest);
  boolean hasWaypointChanged();
  boolean hasWaasLock();

  static void init(SoftwareSerial *mySerial) {
      //byte factory[3]    = {0x02, 0x04, 0x01};
      //byte PedMode[4]    = {0x03, 0x3C, 0x01, 0x00};
      //byte CarMode[4]    = {0x03, 0x3C, 0x00, 0x00};
      //byte Baud19200[5]  = {0x04, 0x05, 0x00, 0x02, 0x00};
      //byte Update10Hz[4] = {0x03, 0x0E, 0x0A, 0x00};
    byte Update5Hz[]  = {0x03, 0x0E, 0x05, 0x00};
    byte WAASEnable[] = {0x03, 0x37, 0x01, 0x00};
      //byte WAASCheck[2]  = {0x01, 0x38};
      //byte fullPwr[4]    = {0x03, 0x0C, 0x00, 0x00};
      //byte NoNMEA[10]     = {0x09, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte GGA_RMC[10]    = {0x09, 0x08, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    //byte GGA[]        = {0x09, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                     // 1 = GGA Enable
                                     //       1 = GSA Enable (*)
                                     //             1 = GSV Enable
                                     //                   1 = GLL Enable
                                     //                         1 = RMC Enable (*)
                                     //                               1 = VTG Enable
                                     //                                     1 = ZDA Enable
                                     //                                           1 = Attributes
    mySerial->begin(9600);
    sndByte(mySerial, 0x0A);
    sndByte(mySerial, 0x0D);
    //Serial.println("Reset");
    //sendCmd(factory);
    // delay(10000);
    // Disable all but GGA Messages
    sendCmd(mySerial, GGA_RMC);
    // Full Power Mode
    //sendCmd(fullPwr);
    // Set update rate to 10Hz (Forces Reset)
    //sendCmd(Update10Hz);
    // Set update rate to 5Hz (Forces Reset)
    sendCmd(mySerial, Update5Hz);
    // Set Car Mode (Forces Reset)
    //sendCmd(CarMode);
    // Enable WAAS
    sendCmd(mySerial, WAASEnable);
    // Check WAAS Status
    //sendCmd(WAASCheck);
  }
  static void sndByte (SoftwareSerial *mySerial, byte cc) {
    mySerial->write(cc);
  }
  
  static void sendCmd (SoftwareSerial *mySerial, byte *msg) {
    sndByte(mySerial, 0xA0);
    sndByte(mySerial, 0xA1);
    sndByte(mySerial, 0x00);
    byte len = msg[0];
    sndByte(mySerial, len);
    byte crc = 0;
    for (int ii = 1; ii <= len; ii++) {
      byte cc = msg[ii];
      sndByte(mySerial, cc);
      crc ^= cc;
    }
    sndByte(mySerial, crc);
    sndByte(mySerial, 0x0D);
    sndByte(mySerial, 0x0A);
  }
  
};

#endif


