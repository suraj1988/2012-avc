#include "Pid.h"

Pid::Pid(float minOutput, float maxOutput):pc(&_input, &_output, &_setpoint, PROPORTIONAL_COEFICIENT, INTEGRAL_COEFICIENT/SAMPLE_RATE/1000, DIFFERENTIAL_COEFICIENT*SAMPLE_RATE/1000, REVERSE) {
  pc.SetSampleTime(SAMPLE_RATE);
  pc.SetOutputLimits(minOutput, maxOutput);
  _setpoint = 0.0;
  _input = 0.0;
}

float Pid::update(Gps *start, Gps *dest, Gps *car) {
  _lastInput = _input;
  _input = getCrosstrackError(start, dest, car);
//  if (_input > _setpoint) {
//    pc.SetControllerDirection(REVERSE);
//  } else {
//    pc.SetControllerDirection(DIRECT);
//  }
  pc.Compute();
  return _output;
}

void Pid::start() {
  pc.SetMode(AUTOMATIC);
}

