float getCrosstrackError2(Gps *start, Gps *dest, Gps *car) { 
  float base = sqrt(pow(dest->getLatitude() - start->getLatitude(), 2.0) +
      pow(dest->getLongitude() - start->getLongitude(), 2.0));
  float leg1 = sqrt(pow(car->getLatitude() - start->getLatitude(), 2.0) +
      pow(car->getLongitude() - start->getLongitude(), 2.0));
  float leg2 = sqrt(pow(dest->getLatitude() - car->getLatitude(), 2.0) +
      pow(dest->getLongitude() - car->getLongitude(), 2.0));
  float s = (base + leg1 + leg2)/2;
  // Heron's formula
  float height = (2.0/base) * sqrt(s * (s - leg1) * (s - leg2) * (s - base));
  return height;
}

float getCrosstrackError(Gps *start, Gps *dest, Gps *car) {
  //x1,y1 = start
  //x2,y2 = end
  //x0,y0 = car
  //((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
  float minutes = ((dest->getLatitude() - start->getLatitude()) *
      (start->getLongitude() - car->getLongitude()) - (start->getLatitude() - car->getLatitude()) *
      (dest->getLongitude() - start->getLongitude())) /
      sqrt(pow(dest->getLatitude() - start->getLatitude(), 2) +
      pow(dest->getLongitude() - start->getLongitude(), 2));
  return minutes;
}

void distances () {
  float x1 = 33.250833;
  float y1 = -111.799583;
  float x2 = 33.25075;
  float y2 = -111.799278;
  float x3 = 33.250722;
  float y3 = -111.799472;
  
  float a = TinyGPS::distance_between(x1, y1, x2, y2);
  float b = TinyGPS::distance_between(x2, y2, x3, y3);
  float c = TinyGPS::distance_between(x3, y3, x1, y1);
  
  Serial << "1 to 2: " << a << endl;
  Serial << "2 to 3: " << b << endl;
  Serial << "3 to 1: " << c << endl;
  
  float cosc = (sq(a) + sq(b) - sq(c)) / (2 * a * b);
  float C = acos(cosc);
  float d = sin(C) * b;
  
  Serial << "sin c: " << sin(C) << endl;
  Serial << "cos c: " << cosc << endl;
  Serial << "Angle C: " << C << endl;
  Serial << "Distance d: " << d << endl;
  
}
