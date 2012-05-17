#include "AvcPid.h"

// time is in seconds, speed in meters / second
int AvcPid::compute (float error, float timeDelta, float speed) {
  float a = speed * sin(alpha);
  float optimumError = 5.0;
  float minimumKd = optimumError / a;
  float kd = max(minimumKd, error / a);
  float derivative = (error - previousError) / timeDelta;
  float result = -kp * error - kd * derivative;
  int output = constrain(int(result), -50, 50);
  //int output = 
#if LOG_PID == 1
//error  timedelta  speed  kd  deltaError  derivative  result  output  a  optimumError  minkd
  Serial <<
      error << "\t" <<
      timeDelta << "\t" <<
      speed << "\t" <<
      kd << "\t" <<
      error - previousError << "\t" <<
      derivative << "\t" <<
      result << "\t" <<
      output << "\t" <<
      a << "\t" <<
      optimumError << "\t" <<
      minimumKd <<
      endl;
#endif
  if (timeDelta > .0001) {
    previousError = error;
  }
  return output;
}

