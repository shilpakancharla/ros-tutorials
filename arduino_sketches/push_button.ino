/*
  DigitalReadSerial

  Reads a digital input on pin 2, prints the result to the Serial Monitor

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/DigitalReadSerial
*/

// digital pin 12 has a pushbutton attached to it. Give it a name:
int pushButton = 12;
int LED = 3;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  pinMode(LED, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int buttonState = digitalRead(pushButton);
  // print out the state of the button:
  if (buttonState == HIGH) {
    digitalWrite(LED, HIGH);
    Serial.print("Push button is pressed, value is: ");
    Serial.println(buttonState);
  } else {
    digitalWrite(LED, LOW);
  }
  delay(1);        // delay in between reads for stability
}
