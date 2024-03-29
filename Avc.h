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
#define PUSH_BUTTON_START_PIN A2

#define COMPASS_ONLY 1
#define LOG_PID 0
#define LOG_HEADING 0
#define LOG_IMU 0
#define LOG_NAV 0
#define LOG_MAPPER 0
#define LOG_EKF 1
#define DISTANCE_FROM_LINE_CORRECTION 0
#define USE_SERVO_LIBRARY 1
#define GO_STRAIGHT 0
#define USE_LINE_INTERSECT 1
#define RESET_RUN_LOCATIONS 0
#define AGRESSIVE_STEERING 1
#define RACE_MODE 0
#define BREAK_BEFORE_TURN 0
#define PUSH_BUTTON_START 0
#define SPEED_THROUGH_TURN 1
#define EVASIVE_ACTION 0
#define SKIP_FIRST_WAYPOINT 0

enum RunLocation {LOC_MANUAL, LOC_PERRY_HIGH, LOC_COUNT};
static char* runLocNames[] = {"MANUAL", "PERRY HIGH"};

#if USE_SERVO_LIBRARY
#define SERVO_CENTER 1500
#define MIN_SERVO 1000
#define MIN_SERVO_75_PERCENT 1125
#define MIN_SERVO_50_PERCENT 1250
#define MAX_SERVO 2000
#define MAX_SERVO_75_PERCENT 1875
#define MAX_SERVO_50_PERCENT 1750
#else
#define SERVO_CENTER 180
#define MIN_SERVO 105
#define MAX_SERVO 255
#endif

// locations
//#define LOC_MANUAL 1
//#define LOC_PERRY_HIGH 2
//#define RUN_LOCATION 1


#endif
