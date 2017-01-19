//============================================================
/*
  Quadcopter UAV IMU Test Code: Version 1.0
  Callsign "Badger"
  Arduino Mega 2560
  Bluetooth Configuration: N/A
*/
//============================================================
#include "Servo.h"
#include "SPI.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"


//============================================================
// Begin Motor Configuration
Servo motor_A, motor_B, motor_C, motor_D;

#define MOTOR_PIN_A 2                                               // Front-right motor  (CCW)
#define MOTOR_PIN_B 3                                               // Back-right motor   (CW)
#define MOTOR_PIN_C 4                                               // Back-left motor    (CCW)
#define MOTOR_PIN_D 5                                               // Front-left motor   (CW)

#define MAX_SIGNAL 2000                                             // Max PWM signal allowed
#define MIN_SIGNAL 1200                                             // Min PWM signal required to turn motors
#define STOP 1000                                                   // PWM signal to stop all motors

int ARM = 1000;                                                     // PWM signal to arm ESCs and motors, ready for flight

int throttle = 0;                                                   // Initialize throttle variable. Receives input from controller

int esc_A = 0;                                                        // Initialize esc output variables
int esc_B = 0;
int esc_C = 0;
int esc_D = 0;

void UpdateMotors(void);
void ControlMotors(void);

// End Motor Configuration


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
// Begin PID Configuration
unsigned long current_time;
float previous_time;
int delta_t;                                                
int sample_time = 100;                                              // 100 times per second

float setpoint_x = 180;                                             // Setpoint angle for x-axis (ROLL)
float setpoint_y = 180;                                             // Setpoint angle for y-axis (PITCH)
float setpoint_z = 0;                                               // Setpoint angle for z-axis (YAW)

float input_x, input_y, input_z;

float error_x, error_y, error_z;
float integral_x, integral_y, integral_z;
float d_input_x, d_input_y, d_input_z;

float output_x, output_y, output_z;
int output_MAX = 400;                                               // Limit maximum output for correcting attitude

float previous_input_x, previous_input_y, previous_input_z;

float kP_roll = 1.3;                                                  // Gain setting for ROLL P-controller
float kI_roll = 0.04;                                                  // Gain setting for ROLL I-controller
float kD_roll = 18;                                                  // Gain setting for ROLL D-controller

float kP_pitch = 1.3;                                                 // Gain setting for PITCH P-controller
float kI_pitch = 0.04;                                                 // Gain setting for PITCH I-controller
float kD_pitch = 18;                                                 // Gain setting for PITCH D-controller

float kP_yaw = 4;                                                   // Gain setting for YAW P-controller
float kI_yaw = 0.02;                                                   // Gain setting for YAW I-controller
float kD_yaw = 0;                                                   // Gain setting for YAW D-controller

float sample_time_seconds;
float ratio;
int new_sample_time;

void ComputePID(void);
void SetTuning(void);
void SetSampleTime(void);

// End PID Configuration


//============================================================
void setup() {
  Serial.begin(9600);

  // LSM9DS0 Setup ===========================================
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


// ESC/Motor Setup ========================================== 
  motor_A.attach(MOTOR_PIN_A);                                    // start PWM stream
  motor_B.attach(MOTOR_PIN_B);
  motor_C.attach(MOTOR_PIN_C);
  motor_D.attach(MOTOR_PIN_D);

  motor_A.writeMicroseconds(ARM);                                 // arm ESCs. Sends 1000us pulse to ESCs to stop beeping
  motor_B.writeMicroseconds(ARM);
  motor_C.writeMicroseconds(ARM);
  motor_D.writeMicroseconds(ARM);
}


//============================================================

void ReadIMU() {
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
  float gyro_Z_angle = (gyro_z/131) + 0.06;                                   // Convert gyro measurement into angle for yaw and add adjustment to zero sensor
  angle_z = gyro_Z_angle;
  
  timer = micros();

  //Serial.print("Angle X: "); Serial.print(angle_x); Serial.print(" "); Serial.print("Angle Y: "); Serial.print(angle_y); Serial.print(" ");  Serial.print("Angle Z: "); Serial.print(gyro_Z_angle);
  //Serial.println();
}

//============================================================
void ComputePID(){
  current_time = millis();
  delta_t = current_time - previous_time;

  input_x = angle_x;
  input_y = angle_y;
  input_z = angle_z;
  
  if (delta_t >= sample_time){                                              // compute once the sample time is reached. establishes regular interval for computation
    // ROLL ===================
    error_x = input_x - setpoint_x;                                         // compute error. + when left side up, right side dips
    integral_x += kI_roll*error_x;                                          // compute integral term with kI so changes in kI only affect future computation
    if (integral_x > output_MAX)                                            // limit I_term_x to values within output range
      integral_x = output_MAX;
    else if (integral_x < output_MAX*-1)                                    // allows for bounded negative output 
      integral_x = output_MAX*-1;
    d_input_x = input_x - previous_input_x;                                 // compute derivative term. eliminates derivative kick by using input terms
    
    output_x = (kP_roll*error_x + integral_x - kD_roll*d_input_x);          // ============ Outputs positive value when roll is positive ===============
    if (output_x > output_MAX)                                              // limit output_x to values within output range
      output_x = output_MAX;
    else if (output_x < output_MAX*-1)                                      // allows for bounded negative output
      output_x = output_MAX*-1;

    // PITCH ===================
    error_y = input_y - setpoint_y;                                         // compute error. + when nose up, tail dips
    integral_y += kI_pitch*error_y;                                         // compute integral term with kI so changes in kI only affect future computations
    if (integral_y > output_MAX)                                            // limit I_term_y to values within output range
      integral_y = output_MAX;
    else if (integral_y < output_MAX*-1)                                    // allows for bounded negative output 
      integral_y = output_MAX*-1; 
    d_input_y = input_y - previous_input_y;                                 // compute derivative term. eliminates derivative kick by using input terms
    
    output_y = (kP_pitch*error_y + integral_y - kD_pitch *d_input_y);       // ============ Outputs positive value when pitch is positive ===============
    if (output_y > output_MAX)                                              // limit output_y to values within output range
      output_y = output_MAX;
    else if (output_y < output_MAX*-1)                                      // allows for bounded negative output
      output_y = output_MAX*-1;
 
    // YAW ===================
    error_z = setpoint_z - input_z;                                         // compute error. + CW
    integral_z += kI_yaw*error_z;                                           // compute integral term with kI so changes in kI only affect future computations
    if (integral_z > output_MAX)                                            // limit I_term_x to values within output range
      integral_z = output_MAX;
    else if (integral_z < output_MAX*-1)                                    // allows for bounded negative output 
      integral_z = output_MAX*-1; 
    d_input_z = input_z - previous_input_z;                                 // compute derivative term. eliminates derivative kick by using input terms
    
    output_z = (kP_yaw*error_z + integral_z - kD_yaw*d_input_z);            // ============ Outputs positive value when yaw is CW ===============
    if (output_z > output_MAX)                                              // limit output_x to values within output range
      output_z = output_MAX;
    else if (output_z < output_MAX*-1)                                      // allows for bounded negative output
      output_z = output_MAX*-1;
 
    previous_input_x = input_x;                                             // recalculate previous input_x
    previous_input_y = input_y;                                             // recalculate previous input_y
    previous_input_z = input_z;                                             // recalculate previous input_y
    previous_time = current_time;
  }
}


//============================================================
  void SetTuning() {
  sample_time_seconds = ((float)sample_time)/1000;
  kP_roll = kP_roll;
  kI_roll = kI_roll*sample_time_seconds;
  kD_roll = kD_roll/sample_time_seconds;

  kP_pitch = kP_pitch;
  kI_pitch = kI_pitch*sample_time_seconds;
  kD_pitch = kD_pitch/sample_time_seconds;

  kP_yaw = kP_yaw;
  kI_yaw = kI_yaw*sample_time_seconds;
  kD_yaw = kD_yaw/sample_time_seconds;
}


//============================================================
void SetSampleTime() {
  if (new_sample_time > 0) {
    ratio = (float)new_sample_time/sample_time;
    kI_roll *= ratio;
    kD_roll /= ratio;

    kI_pitch *= ratio;
    kD_pitch /= ratio;

    kI_yaw *= ratio;
    kD_yaw /= ratio;
    
    sample_time = (unsigned long)new_sample_time;
  }
}


//============================================================
void UpdateMotors(){
  if (throttle > 1800) throttle = 1800;                                                     // Build in buffer so PID has control at full throttle
  
  esc_A = throttle + output_x/2 - output_y/2 + output_z/2;                                  // front right motor    * dips in positive roll, lifts in positive pitch
  esc_B = throttle + output_x/2 + output_y/2 - output_z/2;                                  // back right motor     * dips in positive roll, dips in positive pitch
  esc_C = throttle - output_x/2 + output_y/2 + output_z/2;                                  // back left motor      * lifts in positive roll, dips in positive pitch
  esc_D = throttle - output_x/2 - output_y/2 - output_z/2;                                  // front right motor    * lifts in positive roll, lifts in positive pitch

  // Insert battery voltage compensator here

  if (esc_A < 1200) esc_A = 1200;                                                           // Keep motors running
  if (esc_B < 1200) esc_B = 1200;
  if (esc_C < 1200) esc_C = 1200;
  if (esc_D < 1200) esc_D = 1200;

  if (esc_A > 2000) esc_A = 2000;                                                           // Limit pulse to 2000 us
  if (esc_B > 2000) esc_B = 2000;
  if (esc_C > 2000) esc_C = 2000;
  if (esc_D > 2000) esc_D = 2000;
  
  Serial.print("A: "); Serial.print(esc_A); Serial.print(" "); Serial.print("B: "); Serial.print(esc_B); Serial.print(" "); Serial.print("C: "); Serial.print(esc_C); Serial.print(" "); Serial.print("D: "); Serial.print(esc_D);
  Serial.println();
}


//============================================================
void loop(){
  ReadIMU();
  ComputePID();
  SetTuning();
  SetSampleTime();
  UpdateMotors();
  //ControlMotors();

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'x') {
      throttle = 1500;
    }
    if (ch == 'z') {
      throttle = 0;
    }
  }
}
