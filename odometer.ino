


void countRotations () {
  rotations++;
  odometerMicrosDelta = micros() - previousOdometerMicros;
  previousOdometerMicros = micros();
}
