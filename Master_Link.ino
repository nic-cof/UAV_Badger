//============================================================
// Master Code: Arduino Uno with HC-05 in master configuration
// Callsign "Base"
//============================================================
#include <SoftwareSerial.h>

SoftwareSerial BTserial(10, 11);                  // TX|RX (TX-TX and RX-RX between BT and Arduino)
const int LEDpin = 8;
char New_Status[4];

void setup() {
  Serial.begin(9600);
  BTserial.begin(38400);
  
  Serial.println("Input 'x' to turn LED on, or 'z' to turn LED off.");
}

void loop() {
  // Sending commands from master to slave
  if (Serial.available() > 0){                    // Checks whether data is coming from the serial port
    char ch = Serial.read();
    
    if (ch == 'x'){
      BTserial.write('x');                        // Sends 'x' through the bluetooth serial link to slave            
    }
    else if (ch == 'z'){
      BTserial.write('z');                        // Sends 'z' through the bluetooth serial link to slave        
    }
  }
  // Receiving status from slave
  if (BTserial.available() > 0){
    while(BTserial.available() > 0){
      BTserial.readBytes(New_Status, 4);          // Writes incoming bytes into array
      Serial.write(New_Status);                   // Prints array in ASCII characters
    }
    Serial.println();
    for (int i = 0; i < sizeof(New_Status); i++)
      New_Status[i] = (char)0;                    // Resets array
  }
}
