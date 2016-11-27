//===========================================================
// Slave Code: Arduino Mega with HC-05 in slave configuration
// Callsign: "Badger"
//===========================================================
#include <SoftwareSerial.h>

SoftwareSerial BTserial(10, 11);          // TX|RX (TX-TX and RX-RX between BT and Arduino)
const int LEDpin = 8;
char LEDstatus[4];                        // Creates a string with a max length of 3 characters and one null

void setup() {
  Serial.begin(9600);                    
  BTserial.begin(38400);                  // Set baud rate equal to BTserial on master
  pinMode(LEDpin, OUTPUT);

  BTserial.println("Input 'x' to turn LED on, or 'z' to turn LED off.");
}

void loop() {
  if (BTserial.available() > 0){          // Checks whether data is coming from master
    char ch = BTserial.read();            
    
    if (ch == 'x'){
      digitalWrite(LEDpin, HIGH);         // If serial receives 'x' from BTserial, turn on LED
      strcpy(LEDstatus, "On");            // Replaces content from old string with "On"
      BTserial.write(LEDstatus);          // Send LED status to master
    }
    else if (ch == 'z'){
      digitalWrite(LEDpin, LOW);          // If serial receives 'z' from BTserial, turn off LED
      strcpy(LEDstatus, "Off");           // Replaces content from old string with "Off"
      BTserial.write(LEDstatus);          // Send LED status to master
    }
  }
}
