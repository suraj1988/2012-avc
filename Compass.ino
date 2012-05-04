#define LOG_RAW_COMPASS 0

int CompassAddress = 0x1E;

HMC5883L compass;

int magnetom_x;
int magnetom_y;
int magnetom_z;
int magraw_x;
int magraw_y;
int magraw_z;

float xOffset = 14.0;
float yOffset = -125.0;
float zOffset = 0.0;

void Compass_Init()
{
//  Wire.beginTransmission(CompassAddress);
//  Wire.write((uint8_t)0x02); 
//  Wire.write((uint8_t)0x00);   // Set continouos mode (default to 10Hz)
//  Wire.endTransmission(); //end transmission
  Wire.begin();
  compass = HMC5883L();
  // Set scale to +/- 1.3 Ga
  int error = compass.SetScale(1.3);
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  // Set measurement mode to continous
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void Read_Compass() {
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  magnetom_x = scaled.XAxis - xOffset;
  magnetom_y = scaled.YAxis - yOffset;
  magnetom_z = scaled.ZAxis - zOffset;
#if LOG_RAW_COMPASS == 1
  MagnetometerRaw raw = compass.ReadRawAxis();
  magraw_x = raw.XAxis;
  magraw_y = raw.YAxis;
  magraw_z = raw.ZAxis;
  Serial.print(magraw_x);
  Serial.print("\t");
  Serial.print(magraw_y);
  Serial.print("\t");
  Serial.print(magraw_z);
  Serial.print("\t");
  Serial.print(magnetom_x);
  Serial.print("\t");
  Serial.print(magnetom_y);
  Serial.print("\t");
  Serial.println(magnetom_z);
#endif
}

void Compass_Heading()
{
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(magnetom_y, magnetom_x);
  float declinationAngle = 190.5 / 1000.0;
  heading -= declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  MAG_Heading = heading * 180/M_PI; 
}


void Read_Compass_old()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(CompassAddress); 
  Wire.write((uint8_t)0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission

    //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission

    if (i==6)  // All bytes received?
  {
    // MSB byte first, then LSB, X,Y,Z
    magnetom_x = ((((int)buff[4]) << 8) | buff[5]);    // X axis (internal y axis)
    magnetom_y = ((((int)buff[0]) << 8) | buff[1]);    // Y axis (internal x axis)
    magnetom_z = ((((int)buff[2]) << 8) | buff[3]);    // Z axis
    
  }
  else
    Serial.println("!ERR: Mag data");
}


