#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28); // IMU @ I2C address

int currentMillis, lastMillis = 0;

float lastPosX, lastPosY, lastPosZ;
float posX, posY, posZ;

float lastPosPitch, lastPosYaw, lastPosRoll;

float pitch, yaw, roll;

bool started = false;

void setup() {
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // we're going to turn on the LED once we're running

  if (!bno.begin()) {
    Serial.print("Could not communicate with IMU.");
    while (1);
  }

  bno.setExtCrystalUse(true);

  delay(500);
  Serial.println("roll,pitch,yaw,gyro_calibration,accel_calibration"); // set the arduino serial monitor legend
}


void loop() {

  // Get BNO055 calibration values
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // Get gyro (angular velocity relative to the body frame) and accelerometer vectors
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Find how much time has passed since the last loop
  currentMillis = millis();
  float dt = (currentMillis - lastMillis) / 1000.0; // time difference in seconds
  lastMillis = currentMillis;

  if (started) {

    float velPitch = gyr.y() * DEG_TO_RAD; // Rad per second
    float velYaw = gyr.z() * DEG_TO_RAD;
    float velRoll = gyr.x() * DEG_TO_RAD;

    float velX = velRoll + velPitch * (sin(roll) * tan(pitch)) + velYaw * (cos(roll) * tan(pitch));
    roll = (velX * dt) + lastPosX;
    lastPosX = roll;

    float velY = velPitch * cos(roll) - velYaw * sin(roll);
    pitch = (velY * dt) + lastPosY;
    lastPosY = pitch;

    float velZ = velPitch * (sin(roll) / cos(pitch)) + velYaw * (cos(roll) / cos(pitch));
    yaw = (velZ * dt) + lastPosZ;
    lastPosZ = yaw;

  } else {
    // Only start once the gyro is calibrated (takes less than a second)
    if (gyro == 3) {
      digitalWrite(13, HIGH); // turn on LED
      started = true;
    }
  }

  if(Serial.available() > 0){
    reorient(acc.x(), acc.y(), acc.z());
  }

  // Print out data to serial plotter:
  Serial.print(roll * RAD_TO_DEG);
  Serial.print(",");
  Serial.print(pitch * RAD_TO_DEG);
  Serial.print(",");
  Serial.print(yaw * RAD_TO_DEG);
  Serial.print(",");
  Serial.print(gyro, DEC);
  Serial.print(",");
  Serial.println(accel, DEC);

  // Normally, delay == big bad. In this case, however, it's here to make the integration more precise as I haven't implemented micros() for dt
  delay(5);
}

void reorient(float accX, float accY, float accZ) {
  float totalAccelVec = sqrt(sq(accX) + sq(accY) + sq(accZ));
  pitch = -asin(accX / totalAccelVec);
  roll = asin(accY / totalAccelVec);
}
