const int LEDpin = 8;

void setup() {
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);
  Serial.println("Input 'x' to turn LED on, or 'z' to turn LED off.");
}

void loop() {
  if (Serial.available() > 0){
    char ch = Serial.read();

    if (ch == 'x'){
      digitalWrite(LEDpin, HIGH);
      Serial.println("LED on");
    }
    else if (ch == 'z'){
      digitalWrite(LEDpin, LOW);
      Serial.println("LED off");
    }
  }
}
