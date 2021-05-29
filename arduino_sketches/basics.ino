/**
 * Turn on LED for some time, and turn it off for some time.
 */

int BLUE_LED = 10;
int GREEN_LED = 11;
int RED_LED = 12;

void setup() {
  // put your setup code here, to run once:
  /**
   * Configure the specific pin nin our Arduino to behave either as an input or an output.
   * @param pin: number of the pin to configure
   * @param mode: input or output
   * 
   * Interface Mode Settings:
   * 1. INPUT: set as input
   * 2. INPUT_PULLUP: set it as input - using an internal pull-up resistor
   * 3. OUTPUT: set as output
   * 
   * Needs to be inside setup()
   */
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Initialize LED to be off
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  /**
   * Outputs a value on that specific pin.
   * @param pin: the pin number
   * @param value: high or low
   * 
   * HIGH or LOW
   * 
   * Needs to be inside loop()
   */
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  Serial.println("Light Mode : Green Mode");
  delay(1000);

  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  Serial.println("Light Mode : Blue Mode");
  delay(1000);

  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);

  Serial.println("Light Mode : Red Mode");
  delay(1000);
}
