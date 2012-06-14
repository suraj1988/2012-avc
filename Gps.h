#ifndef Gps_h
#define Gps_h
/*
  $GPGGA,061959.000,3255.1992,N,11706.8458,W,2,6,2.15,174.9,M,-35.4,M,0000,0000*69
  
  $PMTK605*31                                          Query Firmware Version
  $PMTK104*37                                          Full Cold Reset (to Factory settings)
  $PMTK251,38400*27                                    Set Baud Rate to 4800,9600,14400, 19200,38400,57600,115200
  $PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28    GGA & RMC
  $PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29    GGA Only
  $PMTK220,200*2C                                      Set Update Rate to 5 Hz (200 ms interval)
  $PMTK220,1000*2C                                     Set Update Rate to1 Hz (10000 ms interval)
  $PMTK313,1*2E                                        SBAS Enable
  $PMTK301,2*2E                                        Enable WAAS as DGPS Source
  
  Sample output:
    Lat: 32.91993, Long: -117.11415, Alt: 16980 cm, Fix age: 4 ms.
*/

#include "Arduino.h"
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Streaming.h>

#define SET_UP_GPS 1
#define USE_SKYTRACK 0

class Gps {
  long latitude;
  long longitude;
  float hdop;
  boolean lock;
  boolean waasLock;
  TinyGPS       gps;
  float distanceTraveled;
  unsigned long previousFixTime;
  //time of day in hundredths of a second
  unsigned long fixTime;
  unsigned long age;
  unsigned long previousMillis;
  unsigned long updateCheck;
  boolean valid;

  static float toFloat(long fixed) {
    return fixed / 1000000.0;
  }

#if USE_SKYTRACK
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
#else
  static void sendCmd (Stream *mySerial, char *msg) {
    char cc;
    char chk = 0;
    mySerial->write('$');
    while ((cc = *msg++) != '\0') {
      chk ^= cc;
      mySerial->write(cc);
    }
    mySerial->write('*');
    mySerial->println(chk, HEX);
  }
#endif

  public:
  Gps();
  void checkGps(Stream *serial);
  int getHeadingTo (Gps *dest);
  inline void testPrint() {
    Serial << "Lat: " << getLatitude() << ", Lon: " << getLongitude() << ", hdop: " <<
        getHdop() << ", Distance Traveled: " << getDistanceTraveled() << ", Elapsed Time: " <<
        getTimeDelta() << ", WAAS Lock: " << hasWaasLock() << ", Speed: " << _FLOAT(getSpeed(), 5) << " m/s" << endl;
  }

  inline void excelPrint() {
    int m = millis() - previousMillis;
    previousMillis = millis();
    Serial << 
        getLatitude() << "\t" << 
        getLongitude() << "\t" <<
        getHdop() << "\t" << 
        getDistanceTraveled() << "\t" <<
        getFixTime() * 10 << "\t" <<
        getTimeDelta() * 10 << "\t" << 
        age << "\t" << 
        m << "\t" <<
        _FLOAT(getSpeed(), 5) << "\t" << 
        hasWaasLock() <<
        endl;
  }

  inline long getLatitude() {
    return latitude;
  }
  
  inline long getLongitude() {
    return longitude;
  }
  
  inline boolean hasLock() {
    return lock;
  }

  inline boolean hasWaasLock() {
    return waasLock;
  }
  
  inline unsigned long getFixTime () {
    return fixTime;
  }
  
  inline int getTimeDelta () {
    return fixTime - previousFixTime;
  }

  inline float getHdop () {
    return hdop/100.0;
  }
  
  inline float getDistanceTraveled() {
    return distanceTraveled;
  }
  
  inline float getSpeed () {
    return getDistanceTraveled() / (getTimeDelta() / 100.0);
  }
  
  inline boolean isUpdated () {
    if (updateCheck != fixTime) {
      updateCheck = fixTime;
      return true;
    }
    return false;
  }
  
  inline boolean isValid() {return valid;}

  static void init(Stream *mySerial) {
//    mySerial->begin(9600);
#if SET_UP_GPS
#if USE_SKYTRACK
    byte Update5Hz[]  = {0x03, 0x0E, 0x05, 0x00};
    byte WAASEnable[] = {0x03, 0x37, 0x01, 0x00};
    // car 0x03, 0x3C, 0x00, 0x00
    // ped 0x03, 0x3C, 0x01, 0x00
    byte PedestrianMode[] = {0x03, 0x3C, 0x01, 0x00};
    byte GGA_RMC[10]    = {0x09, 0x08, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
                                       // 1 = GGA Enable
                                       //       1 = GSA Enable (*)
                                       //             1 = GSV Enable
                                       //                   1 = GLL Enable
                                       //                         1 = RMC Enable (*)
                                       //                               1 = VTG Enable
                                       //                                     1 = ZDA Enable
                                       //                                           1 = Attributes
    sndByte(mySerial, 0x0A);
    sndByte(mySerial, 0x0D);
    // Disable all but GGA Messages
    sendCmd(mySerial, GGA_RMC);
    // Enable WAAS
    sendCmd(mySerial, WAASEnable);
    // pedestrian mode
    sendCmd(mySerial, PedestrianMode);
    // Set update rate to 5Hz (Forces Reset)
    sendCmd(mySerial, Update5Hz);
#else
//    Serial << "set up mtk" << endl;
    delay(500);
    // Send GGA messages
    sendCmd(mySerial, "PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
//    delay(10);
    //  Set Update Rate to 5 Hz (200 ms interval)
    sendCmd(mySerial, "PMTK220,200");
//    delay(10);
    // SBAS Enable
    sendCmd(mySerial, "PMTK313,1");
//    delay(10);
    // Enable WAAS as DGPS Source
    sendCmd(mySerial, "PMTK301,2");
    // set speed threshold
    sendCmd(mySerial, "PMTK397,0");
//    delay(10);
//    sendCmd(mySerial, "PMTK251,9600");
//    Serial << "end setup mtk" << endl;
//    delay(1000);
//    mySerial->begin(9600);
#endif
#endif
  }  
};

#endif


