//============================================================
/*
  Quadcopter UAV RC Controller Test Code: Version 1.0
  Callsign "Badger"
  Arduino Mega 2560
  Bluetooth Configuration: N/A
*/
//============================================================
// Begin Receiver Configuration
byte last_ch1, last_ch2, last_ch3, last_ch4;
volatile int receiver_input_ch1, receiver_input_ch2, receiver_input_ch3, receiver_input_ch4;
unsigned long timer_1, timer_2, timer_3, timer_4;

// End Receiver Configuration

void setup() {
  // Receiver Setup ===========================================
  PCICR |= (1 << PCIE0);                                                   // Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT3);                                                 // Set PCINT3 (digital input 50) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                 // Set PCINT2 (digital input 51) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                 // Set PCINT1 (digital input 52) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT0);                                                 // Set PCINT0 (digital input 53) to trigger an interrupt on state change.
  Serial.begin(9600);
}

void loop() {
  delay(250);
  printSignals();
}

ISR(PCINT0_vect){
  //Channel 1=========================================
  if(last_ch1 == 0 && PINB & B00001000){                                    // Is input 50 high?
    last_ch1 = 1;                                                           // Remember current input state.
    timer_1 = micros();                                                     // Set timer_1 to current_time.
  }
  else if(last_ch1 == 1 && !(PINB & B00001000)){                            // Input 50 is not high and changed from 1 to 0.
    last_ch1 = 0;                                                           // Remember current input state.
    receiver_input_ch1 = micros() - timer_1;                                // Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(last_ch2 == 0 && PINB & B00000100){                                    // Is input 51 high?
    last_ch2 = 1;                                                           // Remember current input state.
    timer_2 = micros();                                                     // Set timer_1 to current_time.
  }
  else if(last_ch2 == 1 && !(PINB & B00000100)){                            // Input 51 is not high and changed from 1 to 0.
    last_ch2 = 0;                                                           // Remember current input state.
    receiver_input_ch2 = micros() - timer_2;                                // Channel 1 is current_time - timer_1.
  }
  //Channel 3=========================================
  if(last_ch3 == 0 && PINB & B00000010){                                    // Is input 52 high?
    last_ch3 = 1;                                                           // Remember current input state.
    timer_3 = micros();                                                     // Set timer_1 to current_time.
  }
  else if(last_ch3 == 1 && !(PINB & B00000010)){                            // Input 52 is not high and changed from 1 to 0.
    last_ch3 = 0;                                                           // Remember current input state.
    receiver_input_ch3 = micros() - timer_3;                                // Channel 1 is current_time - timer_1.
  }
  //Channel 4=========================================
  if(last_ch4 == 0 && PINB & B00000001){                                    // Is input 53 high?
    last_ch4 = 1;                                                           // Remember current input state.
    timer_4 = micros();                                                     // Set timer_1 to current_time.
  }
  else if(last_ch4 == 1 && !(PINB & B00000001)){                            // Input 53 is not high and changed from 1 to 0.
    last_ch4 = 0;                                                           // Remember current input state.
    receiver_input_ch4 = micros() - timer_4;                                // Channel 1 is current_time - timer_1.
  }
}

void printSignals() {
  Serial.print("Roll: "); Serial.print(receiver_input_ch1); Serial.print(" "); 
  Serial.print("Pitch: "); Serial.print(receiver_input_ch2); Serial.print(" "); 
  Serial.print("Throttle: "); Serial.print(receiver_input_ch3); Serial.print(" "); 
  Serial.print("Yaw: "); Serial.print(receiver_input_ch4);
  Serial.println();
}
