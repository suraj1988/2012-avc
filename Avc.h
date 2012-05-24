#ifndef Avc_h
#define Avc_h

#define MAX_SAMPLES 50
#define WAYPOINT_RADIUS 3.0 // meters

#define LOOP_SPEED 40 //Hertz

#define SERVO_PIN 10
#define MOTOR_PIN 11
#define SPEED_CONTROL_PIN 12
#define MENU_SELECT_PIN A0
#define MENU_SCROLL_PIN A1
#define HALL_SENSOR_PIN 2
#define RXPIN 4
#define TXPIN 5

#define COMPASS_ONLY 1
#define LOG_PID 0
#define LOG_HEADING 0
#define LOG_IMU 0
#define LOG_NAV 0
#define DISTANCE_FROM_LINE_CORRECTION 1
#define USE_SERVO_LIBRARY 1
#define GO_STRAIGHT 0

#if USE_SERVO_LIBRARY
#define SERVO_CENTER 1500
#define MIN_SERVO 1000
#define MAX_SERVO 2000
#else
#define SERVO_CENTER 180
#define MIN_SERVO 105
#define MAX_SERVO 255
#endif

#endif
