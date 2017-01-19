//============================================================
/*
  Quadcopter UAV Battery Monitor Test Code: Version 1.0
  Callsign "Badger"
  Arduino Mega 2560
  Bluetooth Configuration: N/A
*/
//============================================================
#define BatMonPin A0

float ADC_value;
float pin_voltage;
float bat_voltage;
float voltage_ratio = 4.16667;

// Look into using steps to create ratio and reduce operations in code.

void setup() {
  Serial.begin(9600);
}

void loop() {
  ADC_value = analogRead(BatMonPin);                    // Read bits on pin
  pin_voltage = ADC_value*0.00488;                      // Convert to voltage on pin

  bat_voltage = pin_voltage*voltage_ratio;              // Adjust to actual battery voltage
  Serial.println(ADC_value);
}
