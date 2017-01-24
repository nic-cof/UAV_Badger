//============================================================
/*
  Quadcopter UAV Test Flight Code: Version 2.0
  Callsign "Badger"
  Arduino Mega 2560
  Bluetooth Configuration: N/A
*/
//============================================================
#include <Wire.h>

//============================================================
// LSM9DS0 Variables
byte CTRL_REG1_G = 0B00100000;
byte CTRL_REG4_G = 0B00100011;
byte OUT_X_L_G = 0B00101000;
byte OUT_X_H_G = 0B00101001;
byte OUT_Y_L_G = 0B00101010;
byte OUT_Y_H_G = 0B00101011;
byte OUT_Z_L_G = 0B00101100;
byte OUT_Z_H_G = 0B00101101;
byte CTRL_REG1_XM = 0B00100000;
byte CTRL_REG2_XM = 0B00100001;
byte OUT_X_L_A = 0B00101000;
byte OUT_X_H_A = 0B00101001;
byte OUT_Y_L_A = 0B00101010;
byte OUT_Y_H_A = 0B00101011;
byte OUT_Z_L_A = 0B00101100;
byte OUT_Z_H_A = 0B00101101;

byte Read = 0B00000001;
byte Write = 0B00000000;
byte Address_XM = 0B00111010;                                       // Address of accelerometer/magnetometer with SAO connected to Vdd
byte Address_G = 0B11010110;                                        // Address of gyro with SAO connected to Vdd

long accel_x, accel_y, accel_z, accel_vector;
float angle_roll_accel, angle_pitch_accel;
float angle_roll, angle_pitch;
float roll_level_adjust, pitch_level_adjust;
double gyro_roll, gyro_pitch, gyro_yaw;

double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int cal_int;

//============================================================
// PID Variables
float roll_setpoint, pitch_setpoint, yaw_setpoint;

float input_roll, input_pitch, input_yaw;
float error_roll, error_pitch, error_yaw;
float integral_roll, integral_pitch, integral_yaw;
float d_input_roll, d_input_pitch, d_input_yaw;
float previous_input_roll, previous_input_pitch, previous_input_yaw;

float output_roll, output_pitch, output_yaw;
int output_MAX = 400;                                                 // Limit maximum output for correcting attitude

float kP_roll = 0.4;                                                    // Gain setting for ROLL P-controller
float kI_roll = 0.01;                                                    // Gain setting for ROLL I-controller
float kD_roll = 1;                                                    // Gain setting for ROLL D-controller

float kP_pitch = 0.4;                                                   // Gain setting for PITCH P-controller
float kI_pitch = 0.01;                                                   // Gain setting for PITCH I-controller
float kD_pitch = 1;                                                   // Gain setting for PITCH D-controller

float kP_yaw = 0;                                                     // Gain setting for YAW P-controller
float kI_yaw = 0.0;                                                     // Gain setting for YAW I-controller
float kD_yaw = 0;                                                     // Gain setting for YAW D-controller

//============================================================
// Begin Receiver Configuration
byte last_ch1, last_ch2, last_ch3, last_ch4, last_ch5;
volatile int receiver_input_ch1, receiver_input_ch2, receiver_input_ch3, receiver_input_ch4, receiver_input_ch5;
unsigned long timer_ch1, timer_ch2, timer_ch3, timer_ch4, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, current_time, loop_timer;
int throttle;
int esc_A, esc_B, esc_C, esc_D;
int start;

//===========================================================
// Setup Routine
void setup() {
  Serial.begin(9600);
  Wire.begin();
  start = 0;
  DDRH |= B01111000;                                                  // Set ports 6, 7, 8, and 9 as outputs
  DDRB |= B01000000;                                                  // Set port 12 as output

  digitalWrite(12, HIGH);                                             //Turn on the warning led.

  WriteRegister_G(CTRL_REG1_G,0B00001111);                            // Activate gyro in normal mode and enable x, y, and z outputs, data rate defauts to 95Hz
  WriteRegister_G(CTRL_REG4_G,0B00001000);                            // Set gyro full scale to +/-500 dps
  
  WriteRegister_XM(CTRL_REG1_XM,0B00110111);                          // Activate accelerometer in normal mode, enable x, y, and z outputs, data rate defaults to 12.5Hz
  WriteRegister_XM(CTRL_REG2_XM,0B00011000);                          // Set accel full scale to +/-8 g
  
  delay(250);                                                         // Allow time to initialize sensor

  Serial.print("Calibrating gyro. Standby");
  for (cal_int = 0; cal_int < 1000; cal_int++){                       // Take 1000 samples from the gyro
    ReadIMU();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    if (cal_int % 100 == 0)
      Serial.print(".");
    PORTH |= B01111000;                                               // Set ports 6, 7, 8, and 9 HIGH
    delayMicroseconds(1000);                                          // Send a 1000us pulse
    PORTH &= B10000111;                                               // Set ports 6, 7, 8, and 9 LOW
    delay(3);
  }
  
  gyro_roll_cal /= 1000;                                              // Take the average value of those samples to use for calibration
  gyro_pitch_cal /= 1000;
  gyro_yaw_cal /= 1000;
  
  Serial.println();
  Serial.println("Calibration complete.");

// Receiver Setup ============================================
  PCICR |= (1 << PCIE0);                                              // Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT3);                                            // Set PCINT3 (digital input 50) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                                            // Set PCINT2 (digital input 51) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                                            // Set PCINT1 (digital input 52) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT0);                                            // Set PCINT0 (digital input 53) to trigger an interrupt on state change

  while(receiver_input_ch3 < 990 || receiver_input_ch3 > 1120 || receiver_input_ch4 < 1400){
    start++;
    PORTH |= B01111000;
    delayMicroseconds(1000);
    PORTH &= B10000111;
    delay(3);
    if (start == 0) {
      digitalWrite(12, !digitalRead(12));
      start = 0;
    }
  }

  start = 0;
  // Insert batter voltage compensator
  loop_timer = micros();
  digitalWrite(12, LOW);
}

// ============================================================
// Main loop
void loop() {
  ReadIMU();
  // Linear Acceleration Sensitivity: +/-8 g............. 0.244 mg/LSB 
  // Angular Rate Sensitivity:        +/-500 dps......... 17.50 mdps/digit
  
  // Gyro angle conversions ==================
  // 1dps / 0.0175 dps = 57.14286
  // 0.000175 = 1 / (100Hz / 57.14286)
  // 0.000003054 = 0.000175 * (3.1415 / 180deg) *** Arduino sin function is in radians ***
  
  angle_roll += gyro_roll * 0.000175;                                                   // Traveled roll angle over the sample time
  angle_pitch += gyro_pitch * 0.000175;                                                 // Traveled pitch angle over the sample time

  angle_roll += angle_pitch * sin(gyro_yaw * 0.000003054);                              // If the quadcopter yaws, transfer the roll angle to the pitch angle
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000003054);                              // If the quadcopter yaws, transfer the pitch angle to the roll angle

  // Accel conversions =======================
  // 0.244mg/LSB == 4096 bits/g
  // 57.296 = 1 / (3.1415 / 180) *** Arduino asin funciton is in radians ***
  
  accel_vector = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z)); // Calculate the net acceleration vector

  if (abs(accel_x) < accel_vector) {                                                    // Prevent asin function from producing NaN
    angle_pitch_accel = asin((float)accel_x / accel_vector) * -57.296;                   // Calculate roll angle
  }
  if (abs(accel_y) < accel_vector) {                                                    // Prevent asin function from producing NaN
    angle_roll_accel = asin((float)accel_y / accel_vector) * 57.296;                   // Calculate pitch angle
  }

  angle_roll_accel -= 0.1;                                                              // Calibration value for roll
  angle_pitch_accel -= 2.6;                                                             // Calibration value for pitch

  angle_roll = angle_roll * 0.96 + angle_roll_accel * 0.04;                           // Correct for gyro drift with accel roll angle
  angle_pitch = angle_pitch * 0.96 + angle_pitch_accel * 0.04;                        // Correct for gyro drift with accel pitch angle

  roll_level_adjust = angle_roll * 15;                                                  // Calculate the roll angle correction
  pitch_level_adjust = angle_pitch * 15;                                                // Calculate the pitch angle correction
  

  if(receiver_input_ch3 < 1150 && receiver_input_ch4 < 1140)start = 1;                  // Starting the motors: throttle low and yaw left (step 1)

  if(start == 1 && receiver_input_ch3 < 1150 && receiver_input_ch4 > 1450){             // When yaw stick is back in the center position start the motors (step 2)
    start = 2;
    angle_pitch = angle_pitch_accel;                                                    // Set gyro pitch angle equal to accelerometer pitch angle when quadcopter is started
    angle_roll = angle_roll_accel;                                                      // Set gyro roll angle equal to accelerometer roll angle when quadcopter is started
    //gyro_angles_set = true;                                                               //Set the IMU started flag.

    angle_roll = angle_roll_accel;
    angle_pitch = angle_pitch_accel;
    
    // Reset PID inputs for a bumpless start
    previous_input_roll = 0;
    integral_roll = 0;
    previous_input_pitch = 0;
    integral_pitch = 0;
    previous_input_yaw = 0;
    integral_yaw = 0;
  }
  if(start == 2 && receiver_input_ch3 < 1150 && receiver_input_ch4 > 1860) {            // Stopping the motors: throttle low and yaw right
    start = 0;    
  }
  
  //In the case of dividing by 3 the max roll/pitch/yaw rate is aprox 164 degrees per second ((500-8)/3 = 164d/s)
  // ROLL =================================
  roll_setpoint = 0;                                                                    // PID set point in degrees per second is determined by the roll receiver input
  if(receiver_input_ch1 > 1508)
    roll_setpoint = receiver_input_ch1 - 1508;                                          // Build in a dead band of 16us to smooth inputs
  else if(receiver_input_ch1 < 1492)
    roll_setpoint = receiver_input_ch1 - 1492;

  roll_setpoint -= roll_level_adjust;                                                   // Subtract angle correction from standardized receiver roll input value
  roll_setpoint /= 3.0;                                                                 // Divide setpoint for PID roll controller by 3 to get angles in degrees

  // PITCH ================================
  pitch_setpoint = 0;                                                                   // PID set point in degrees per second is determined by the pitch receiver input
  if(receiver_input_ch2 > 1508)                                                         // Build in a dead band of 16us to smooth inputs
    pitch_setpoint = receiver_input_ch2 - 1508;
  else if(receiver_input_ch2 < 1492)
    pitch_setpoint = receiver_input_ch2 - 1492;

  pitch_setpoint -= pitch_level_adjust;                                                 // Subtract angle correction from standardized receiver pitch input value
  pitch_setpoint /= 3.0;                                                                // Divide setpoint for PID pitch controller by 3 to get angles in degrees

  // YAW ==================================
  yaw_setpoint = 0;                                                                     // PID set point in degrees per second is determined by the yaw receiver input
  if(receiver_input_ch3 > 1050){                                                        // Do not yaw when turning off the motors
    if(receiver_input_ch4 > 1508) 
      yaw_setpoint = (receiver_input_ch4 - 1508)/3.0;
    else if(receiver_input_ch4 < 1492)
      yaw_setpoint = (receiver_input_ch4 - 1492)/3.0;
  }
  
  ComputePID();

  throttle = receiver_input_ch3;                                            // Set throttle equal to receiver input 3
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_A = throttle + output_pitch/2 - output_roll/2 - output_yaw/2;       //Calculate the pulse for esc 1 (front-right - CCW)
    esc_B = throttle - output_pitch/2 - output_roll/2 + output_yaw/2;       //Calculate the pulse for esc 2 (rear-right - CW)
    esc_C = throttle - output_pitch/2 + output_roll/2 - output_yaw/2;       //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_D = throttle + output_pitch/2 + output_roll/2 + output_yaw/2;       //Calculate the pulse for esc 4 (front-left - CW)

    // Battery voltage compensator
    
    if (esc_A < 1150) esc_A = 1150;                                         //Keep the motors running.
    if (esc_B < 1150) esc_B = 1150;                                         //Keep the motors running.
    if (esc_C < 1150) esc_C = 1150;                                         //Keep the motors running.
    if (esc_D < 1150) esc_D = 1150;                                         //Keep the motors running.

    if (esc_A > 2000)esc_A = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_B > 2000)esc_B = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_C > 2000)esc_C = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_D > 2000)esc_D = 2000;                                          //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_A = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_B = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_C = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_D = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  if(micros() - loop_timer > 10050)digitalWrite(12, HIGH);
  
  // The refresh rate is 100Hz. That means the ESC's need a pulse every 8ms
  while(micros() - loop_timer < 10000);                                     // Wait until 8000us are passed.
  loop_timer = micros();                                                    // Set the timer for the next loop.
  
  PORTH |= B01111000;                                                       // Set digital outputs 6,7,8, and 9 high.
  timer_ch1 = esc_A + loop_timer;                                           // Calculate the time of the faling edge of the esc-1 pulse.
  timer_ch2 = esc_B + loop_timer;                                           // Calculate the time of the faling edge of the esc-2 pulse.
  timer_ch3 = esc_C + loop_timer;                                           // Calculate the time of the faling edge of the esc-3 pulse.
  timer_ch4 = esc_D + loop_timer;                                           // Calculate the time of the faling edge of the esc-4 pulse.

  while(PORTH >= 8) {                                                       // Stay in this loop until output 6,7,8, and 9 are low.
    esc_loop_timer = micros();                                              // Read the current time.
    if(timer_ch1 <= esc_loop_timer) PORTH &= B10111111;                     // Set digital output 6 to low if the time is expired.
    if(timer_ch2 <= esc_loop_timer) PORTH &= B11011111;                     // Set digital output 7 to low if the time is expired.
    if(timer_ch3 <= esc_loop_timer) PORTH &= B11101111;                     // Set digital output 8 to low if the time is expired.
    if(timer_ch4 <= esc_loop_timer) PORTH &= B11110111;                     // Set digital output 9 to low if the time is expired.
  }
}

//============================================================
// Interrupt Service Routine for Receiver
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(last_ch1 == 0 && PINB & B00001000){                                    // Is input 50 high?
    last_ch1 = 1;                                                           // Remember current input state.
    timer_1 = current_time;                                                 // Set timer_1 to current_time.
  }
  else if(last_ch1 == 1 && !(PINB & B00001000)){                            // Input 50 is not high and changed from 1 to 0.
    last_ch1 = 0;                                                           // Remember current input state.
    receiver_input_ch1 = current_time - timer_1;                            // Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(last_ch2 == 0 && PINB & B00000100){                                    // Is input 51 high?
    last_ch2 = 1;                                                           // Remember current input state.
    timer_2 = current_time;                                                 // Set timer_1 to current_time.
  }
  else if(last_ch2 == 1 && !(PINB & B00000100)){                            // Input 51 is not high and changed from 1 to 0.
    last_ch2 = 0;                                                           // Remember current input state.
    receiver_input_ch2 = current_time - timer_2;                            // Channel 1 is current_time - timer_1.
  }
  //Channel 3=========================================
  if(last_ch3 == 0 && PINB & B00000010){                                    // Is input 52 high?
    last_ch3 = 1;                                                           // Remember current input state.
    timer_3 = current_time;                                                 // Set timer_1 to current_time.
  }
  else if(last_ch3 == 1 && !(PINB & B00000010)){                            // Input 52 is not high and changed from 1 to 0.
    last_ch3 = 0;                                                           // Remember current input state.
    receiver_input_ch3 = current_time - timer_3;                            // Channel 1 is current_time - timer_1.
  }
  //Channel 4=========================================
  if(last_ch4 == 0 && PINB & B00000001){                                    // Is input 53 high?
    last_ch4 = 1;                                                           // Remember current input state.
    timer_4 = current_time;                                                 // Set timer_1 to current_time.
  }
  else if(last_ch4 == 1 && !(PINB & B00000001)){                            // Input 53 is not high and changed from 1 to 0.
    last_ch4 = 0;                                                           // Remember current input state.
    receiver_input_ch4 = current_time - timer_4;                            // Channel 1 is current_time - timer_1.
  }
}

//============================================================
// Read IMU Functions
void ReadIMU() {
  byte X_L = ReadRegister_G(OUT_X_L_G);
  byte X_H = ReadRegister_G(OUT_X_H_G);
  byte Y_L = ReadRegister_G(OUT_Y_L_G);
  byte Y_H = ReadRegister_G(OUT_Y_H_G);
  byte Z_L = ReadRegister_G(OUT_Z_L_G);
  byte Z_H = ReadRegister_G(OUT_Z_H_G);

  gyro_roll = X_H <<8 | X_L;
  if (cal_int == 1000) gyro_roll -= gyro_roll_cal;
  gyro_pitch = Y_H <<8 | Y_L;
  if (cal_int == 1000) gyro_pitch -= gyro_pitch_cal;
  gyro_yaw = Z_H <<8 | Z_L;
  if (cal_int == 1000) gyro_yaw -= gyro_yaw_cal;

  byte X_L_A = ReadRegister_XM(OUT_X_L_A);
  byte X_H_A = ReadRegister_XM(OUT_X_H_A);
  byte Y_L_A = ReadRegister_XM(OUT_Y_L_A);
  byte Y_H_A = ReadRegister_XM(OUT_Y_H_A);
  byte Z_L_A = ReadRegister_XM(OUT_Z_L_A);
  byte Z_H_A = ReadRegister_XM(OUT_Z_H_A);

  accel_x = X_H_A <<8 | X_L_A;
  accel_y = Y_H_A <<8 | Y_L_A;
  accel_z = Z_H_A <<8 | Z_L_A; 
}

byte ReadRegister_G(int Register){
byte result = 0;
  Wire.beginTransmission((Address_G | Write) >>1 );           // Slave ID start talking
  Wire.write(Register);                                       // Ask for info in register
  Wire.endTransmission(0);                                    // Complete the send

  Wire.requestFrom((Address_G | Read) >>1 , 1);               // Request 1 byte
  while( Wire.available() == 0);                              // Wait for info
  result = Wire.read();                                       // Get info
  Wire.endTransmission();
  return(result);  
}

void WriteRegister_G(byte Register, byte Value){
  Wire.beginTransmission((Address_G | Write) >>1 );
  Wire.write(Register);
  Wire.write(Value);
  Wire.endTransmission();
}

byte ReadRegister_XM(int Register){
byte result = 0;
  Wire.beginTransmission((Address_XM | Write) >>1 );          // Slave ID start talking
  Wire.write(Register);                                       // Ask for info in register
  Wire.endTransmission(0);                                    // Complete the send
  
  Wire.requestFrom((Address_XM | Read) >>1 , 1);              // Request 1 byte
  while( Wire.available() == 0);                              // Wait for info
  result = Wire.read();                                       // Get info
  Wire.endTransmission();
  return(result);  
}

void WriteRegister_XM(byte Register, byte Value){
  Wire.beginTransmission((Address_XM | Write) >>1 );
  Wire.write(Register);
  Wire.write(Value);
  Wire.endTransmission();
}

//============================================================
// Compute PID
void ComputePID() {
  // ROLL ===================
  error_roll = input_roll - roll_setpoint;                                            // compute error. + when left side up, right side dips
  integral_roll += kI_roll*error_roll;                                                // compute integral term with kI so changes in kI only affect future computation
  if (integral_roll > output_MAX)                                                     // limit I_term_roll to values within output range
    integral_roll = output_MAX;
  else if (integral_roll < output_MAX*-1)                                             // allows for bounded negative output 
    integral_roll = output_MAX*-1;
  d_input_roll = input_roll - previous_input_roll;                                    // compute derivative term. eliminates derivative kick by using input terms
    
  output_roll = (kP_roll*error_roll + integral_roll - kD_roll*d_input_roll);          // ============ Outputs positive value when roll is positive ===============
  if (output_roll > output_MAX)                                                       // limit output_roll to values within output range
    output_roll = output_MAX;
  else if (output_roll < output_MAX*-1)                                               // allows for bounded negative output
    output_roll = output_MAX*-1;

  // PITCH ===================
  error_pitch = input_pitch - pitch_setpoint;                                         // compute error. + when nose up, tail dips
  integral_pitch += kI_pitch*error_pitch;                                             // compute integral term with kI so changes in kI only affect future computations
  if (integral_pitch > output_MAX)                                                    // limit I_term_pitch to values within output range
    integral_pitch = output_MAX;
  else if (integral_pitch < output_MAX*-1)                                            // allows for bounded negative output 
    integral_pitch = output_MAX*-1; 
  d_input_pitch = input_pitch - previous_input_pitch;                                 // compute derivative term. eliminates derivative kick by using input terms
    
  output_pitch = (kP_pitch*error_pitch + integral_pitch - kD_pitch*d_input_pitch);    // ============ Outputs positive value when pitch is positive ===============
  if (output_pitch > output_MAX)                                                      // limit output_pitch to values within output range
    output_pitch = output_MAX;
  else if (output_pitch < output_MAX*-1)                                              // allows for bounded negative output
    output_pitch = output_MAX*-1;
 
  // YAW ===================
  error_yaw = input_yaw - yaw_setpoint;                                               // compute error. + CW
  integral_yaw += kI_yaw*error_yaw;                                                   // compute integral term with kI so changes in kI only affect future computations
  if (integral_yaw > output_MAX)                                                      // limit I_term_yaw to values within output range
    integral_yaw = output_MAX;
  else if (integral_yaw < output_MAX*-1)                                              // allows for bounded negative output 
    integral_yaw = output_MAX*-1; 
  d_input_yaw = input_yaw - previous_input_yaw;                                       // compute derivative term. eliminates derivative kick by using input terms
    
  output_yaw = (kP_yaw*error_yaw + integral_yaw - kD_yaw*d_input_yaw);                // ============ Outputs positive value when yaw is CW ===============
  if (output_yaw > output_MAX)                                                        // limit output_yaw to values within output range
    output_yaw = output_MAX;
  else if (output_yaw < output_MAX*-1)                                                // allows for bounded negative output
    output_yaw = output_MAX*-1;
 
  previous_input_roll = input_roll;                                                   // recalculate previous input_roll
  previous_input_pitch = input_pitch;                                                 // recalculate previous input_pitch
  previous_input_yaw = input_yaw;                                                     // recalculate previous input_yaw
/*
  Serial.print(receiver_input_ch1); Serial.print(" ");
  Serial.print(receiver_input_ch2); Serial.print(" ");
  Serial.print(receiver_input_ch3); Serial.print(" ");
  Serial.print(receiver_input_ch4); Serial.print(" "); 
  Serial.println();
*/ 
}
