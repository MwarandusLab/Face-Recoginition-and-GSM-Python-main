// Define the LED pins
const int redLedPin = 9;
const int greenLedPin = 10;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  // Set the LED pins as output
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();

    // Turn on/off the LEDs based on the command
    if (command == '1') {
      // Command to turn on the green LED
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(redLedPin, LOW);
    } else if (command == '2') {
      // Command to turn on the red LED
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
    }
  }
}
