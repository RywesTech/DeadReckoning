#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

int lastMillis = 0;

float lastPosX, lastPosY, lastPosZ;
float posX, posY, posZ;

float lastPosPitch, lastPosYaw, lastPosRoll;
//float posPitch, posYaw, posRoll;

float pitch, yaw, roll;

bool started = false;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (!bno.begin()) {
    Serial.print("Could not communicate with IMU.");
    while (1);
  }

  bno.setExtCrystalUse(true);

  delay(2000);
  Serial.println("roll,pitch,yaw");
}

void loop() {

  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /*
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.print(mag, DEC);
  Serial.print("\t\t");
  */

  int currentMillis = millis();
  float dts = (currentMillis - lastMillis) / 1000.0;
  lastMillis = currentMillis;

  if (started) {

    // p = roll vel
    // q = pitch vel
    // r = yaw vel

    float velPitch = gyr.y() * DEG_TO_RAD; // Rad per second
    float velYaw = gyr.z() * DEG_TO_RAD;
    float velRoll = gyr.x() * DEG_TO_RAD;

    float velX = velRoll + velPitch * (sin(roll) * tan(pitch)) + velYaw * (cos(roll) * tan(pitch));
    roll = (velX * dts) + lastPosX;
    lastPosX = roll;

    float velY = velPitch * cos(roll) - velYaw * sin(roll);
    pitch = (velY * dts) + lastPosY;
    lastPosY = pitch;

    float velZ = velPitch * (sin(roll) / cos(pitch)) + velYaw * (cos(roll) / cos(pitch));
    yaw = (velZ * dts) + lastPosZ;
    lastPosZ = yaw;
    
    Serial.print(roll * RAD_TO_DEG);
    Serial.print(",");
    Serial.print(pitch * RAD_TO_DEG);
    Serial.print(",");
    Serial.print(yaw * RAD_TO_DEG);
    Serial.println();
  } else {
    Serial.println();
  }

  if (Serial.available() > 0) {
    started = true;
  }

  delay(10);
}
