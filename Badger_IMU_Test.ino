//============================================================
/*
  Quadcopter UAV IMU Test Code: Version 1.0
  Callsign "Badger"
  Arduino Mega 2560
  Bluetooth Configuration: N/A
*/
//============================================================
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


//============================================================
// Begin IMU Configuration
/*
Adafruit LSM9DS0
Connections (For default I2C)
  ===========
  SCL to SCL 21 (Mega)
  SDA to SDL 20 (Mega)
  VDD to 5V DC
  GND to GND
*/

float angle_x = 0;                                                  // Initialize angles generated from IMU data
float angle_y = 0;
float angle_z = 0;

float dt = 0;
float timer = 0;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);                      // Use I2C, ID #1000, **Assign a unique base ID for this sensor**

void ReadIMU(void);

// End IMU Configuration


//============================================================
void setup() {
  Serial.begin(9600);
  #ifndef ESP8266
  while (!Serial);                                                  // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  if(!lsm.begin())                                                  // initialize the sensor
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  //configureSensor();
  Serial.println("Sensors Configured");

  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void loop() {
  dt = (micros() - timer)/1000000;                                            // calculate dt between each sample ****** CONSIDER "IF STATEMENT" IF EXECUTING INTERRUPT ****** 
  
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  float gyro_x = gyro.gyro.x;
  float gyro_y = gyro.gyro.y;
  float gyro_z = gyro.gyro.z;

  float accel_x = accel.acceleration.x;
  float accel_y = accel.acceleration.y;
  float accel_z = accel.acceleration.z;

  // ROLL ===================
  float accel_X_angle = (atan2(accel_y, accel_z)+PI)*180/PI;                  // Convert accel measurement into angle for roll
  float gyro_X_angle = gyro_x/131;                                            // Convert gyro measurement into angle for roll

  angle_x = (0.97)*(angle_x + gyro_X_angle*dt) + (0.03)*(accel_X_angle);      // Complimentary filter to eliminate drift in IMU data

  // PITCH ===================
  float accel_Y_angle = (atan2(accel_x, accel_z)+PI)*180/PI;                  // Convert accel measurement into angle for pitch
  float gyro_Y_angle = gyro_y/131;                                            // Convert gyro measurement into angle for pitch

  angle_y = (0.965)*(angle_y + gyro_Y_angle*dt) + (0.035)*(accel_Y_angle);    // Complimentary filter to eliminate drift in IMU data

  // YAW ===================
  //float accel_Z_angle = (atan2(accel_y, accel_x)+PI)*180/PI;                // Convert accel measurement into angle for yaw
  float gyro_Z_angle = (gyro_z/131) + 0.05;                                   // Convert gyro measurement into angle for yaw

  //angle_z = (0.96)*(angle_z + gyro_Z_angle*dt) + (0.04)*(accel_Z_angle);    // Complimentary filter to eliminate drift in IMU data
  
  timer = micros();

  Serial.print("Angle X: "); Serial.print(angle_x); Serial.print(" "); Serial.print("Angle Y: "); Serial.print(angle_y); Serial.print(" ");  Serial.print("Angle Z: "); Serial.print(gyro_Z_angle);
  Serial.println();
}
