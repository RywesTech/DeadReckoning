#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

int lastMillis = 0;
float lastVel = 0.0;
float lastPos = 0.0;
bool started = false;

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.print("Could not communicate with IMU.");
    while (1);
  }

  delay(500);
}

void loop() {

  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print("X: ");
  Serial.print(lin_accel.x());
  Serial.print(" Y: ");
  Serial.print(lin_accel.y());
  Serial.print(" Z: ");
  Serial.print(lin_accel.z());
  Serial.print("\t\t");

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.print(mag, DEC);
  Serial.print("\t\t");

  int currentMillis = millis();
  float dts = (currentMillis - lastMillis) / 1000.0;
  lastMillis = currentMillis;

  if (started) {

    float Accel = -lin_accel.x();
    if(Accel <= 0.05 && Accel >= -0.05){
      Accel = 0.0;
    }
    
    float vel = (Accel * dts) + lastVel;
    if(vel <= 0.02 && vel >= -0.02){
      vel = 0.0;
    }
    lastVel = vel;

    float pos = (vel * dts) + lastPos;
    lastPos = pos;

    //printDouble(pos, 4);
    Serial.print(pos);
    Serial.print("  ");
    Serial.print(vel);
    Serial.print("  ");
    Serial.print(Accel);
    Serial.print("  ");
    Serial.println();
  } else {
    Serial.println();
  }

  if (Serial.available() > 0) {
    started = true;
  }

  delay(10);
}

void printDouble( double val, byte precision) {
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if ( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision - 1;
    while (precision--)
      mult *= 10;

    if (val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val) - val ) * mult;
    unsigned long frac1 = frac;
    while ( frac1 /= 10 )
      padding--;
    while (  padding--)
      Serial.print("0");
    Serial.print(frac, DEC) ;
  }
}
