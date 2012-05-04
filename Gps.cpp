#include "Gps.h"

Gps::Gps() {
  inStatus = 0;
  idx = 0;
  lock = false;
  hdop = 0;
  latitude = 0;
  longitude = 0;
  _waypointChanged = false;
  _waasLock = false;
}

void Gps::checkGps(SoftwareSerial *serial) {
  checkGps(serial, false);
}

boolean Gps::checkGps(SoftwareSerial *serial, boolean sample) {
  char          buf[256];
  boolean sampled = false;
  while (serial->available()) {
    unsigned char cc = serial->read();
    if (cc == 0xA0) {
      inStatus = 1;
      idx = 0;
    }
    if (inStatus) {
      if (cc == 0x0D) {
        inStatus = 0;
#if PRINT_GPS == 1
        Serial.print("Status: ");
        for (int ii = 0; ii < idx; ii++) {
          Serial.print((unsigned char) buf[ii], HEX);
          Serial.print(" ");
        }
        Serial.println();
#endif
      } else {
        buf[idx++] = cc;
      }
    } else {
      if (cc != 0x0A && cc != 0x0D  &&  idx < 254)
        buf[idx++] = cc;
      if (gps.encode(cc)) {
        lock = true;
        unsigned long age;
        long lat, lon;
        gps.get_position(&lat, &lon, &age);
        float currentHdop = gps.hdop();
        if (sample) {
          sampled = true;
          if (hdop == 0 || currentHdop < hdop) {
            latitude = lat;
            longitude = lon;
            hdop = currentHdop;
//#if TEST_WAYPOINT_SAMPLING == 1
//            logWaypointSamples();
//#endif
          }
        } else {
          latitude = lat;
          longitude = lon;
          hdop = currentHdop;
          if (gps.fix_type() == '2') {
            _waasLock = true;
          } else {
            _waasLock = false;
          }
        }
#if PRINT_GPS == 1
        Serial.print("Lat: ");
        Serial.print(latitude);
        Serial.print(", Long: ");
        Serial.print(longitude);
        Serial.print(", Alt: ");
        Serial.print(gps.f_altitude());
        Serial.print(" Meters, GPS Heading: ");
        Serial.print(gps.f_course());
        Serial.print(" degrees");
        unsigned char fix_type = gps.fix_type();
        Serial.print(", Fix type: ");
        Serial.write(fix_type);
        unsigned char sats = gps.satellites();
        Serial.print(", Sats: ");
        Serial.print(sats, DEC);
        Serial.print(", HDOP: ");
        Serial.print(hdop);
        Serial.println();
        buf[idx] = 0;
        Serial.println(buf);
#endif
        idx = 0;
      } else if (cc == 0x0D) {
        buf[idx] = 0;
#if PRINT_GPS == 1
        Serial.println(buf);
#endif
        idx = 0;
      }
    }
  }
  return sampled;
}

int Gps::pickWaypoint(Gps *waypoints[], int nextWaypoint, int totalWaypoints) {
  if (totalWaypoints >= 2) {
    float distanceFromWaypoint = gps.distance_between(toFloat(latitude), 0.0f, 
        toFloat(waypoints[nextWaypoint]->getLatitude()), toFloat(waypoints[nextWaypoint]->getLongitude() - longitude));
    if (distanceFromWaypoint < WAYPOINT_RADIUS) {
      nextWaypoint = (nextWaypoint + 1) % totalWaypoints;
      _waypointChanged = true;
    } else {
      _waypointChanged = false;
    }
  }
  return nextWaypoint;
}

float Gps::toFloat (long fixed) {
  return fixed / 1000000.0;
}

long Gps::getLatitude() {
  return latitude;
}

long Gps::getLongitude() {
  return longitude;
}

int Gps::sample(SoftwareSerial *serial) {
  if (checkGps(serial, true))
    return 1;
  return 0;
}

boolean Gps::hasLock() {
  return lock;
}

int Gps::getHeadingTo (Gps *dest) {
  return (int) TinyGPS::course_to(toFloat(latitude), 0.0f, toFloat(dest->getLatitude()), toFloat(dest->getLongitude() - longitude));
}

boolean Gps::hasWaypointChanged() {
  return _waypointChanged;
}

boolean Gps::hasWaasLock() {
  return _waasLock;
}

