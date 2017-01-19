//============================================================
/*
 * Quadcopter UAV Motor Thrust Test Code: Version 1.0
 * Callsign "Badger"
 * Arduino Uno
 * Bluetooth Configuration: N/A
 */
//============================================================
#include "Servo.h"
#include "HX711.h"

Servo motor;

#define MOTOR 3
#define MAX_SIGNAL 2000                                       // Max PWM signal for ESCs (Full throttle)
#define MIN_SIGNAL 1100                                       // Min PWM signal for ESCs (Motors start spinning)

int ARM = 1000;                                               // Signal to arm ESCs for test
int throttle = 0;                                             // Initial throttle value

#define calibration_factor 1526                               // Obtained using calibration sketch in HX711-master > libraries > firmware
#define DOUT  A1                                              // Yellow
#define CLK  A0                                               // Green

HX711 scale(DOUT, CLK);

void setup() {
  Serial.begin(9600);
  Serial.println("Configuring scale: remove all weight.");

  scale.set_scale(calibration_factor);
  scale.tare();                                               // Zero scale with no weight on it

  Serial.println("n/");
  Serial.println("Scale is zeroed.");
      
  motor.attach(MOTOR);                                        // Mount motor to motor pin 

  motor.writeMicroseconds(ARM);                               // Arm ESCs
      
  Serial.println("Motor armed.");
  Serial.println("Begin motor throttle test...");
  Serial.println("Input 'o' to arm motors. Input 'w' to increase throttle. Input 's' to decrease throttle. 'x' will shut off the motor.");
}

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == 'o') {
      throttle = 1000;                                        // Increase throttle by 10%
      motor.writeMicroseconds(throttle);                      // Arm ESCs
      }
    if (ch == 'w') {
      throttle += 100;                                        // Increase throttle by 10%
      motor.writeMicroseconds(throttle);                      // Arm ESCs
    }
    else if (ch == 's') {
      throttle -= 100;                                        // Decrease throttle by 10%
      motor.writeMicroseconds(throttle);                      // Arm ESCs
    }
    else if (ch == 'x') {
      throttle = 0;                                           // Kill power
      motor.writeMicroseconds(throttle);                      // Arm ESCs
    }
  }
  Serial.print(throttle), Serial.print("\t"), Serial.print(scale.get_units(), 1), Serial.print(" g");
  Serial.println();
}
