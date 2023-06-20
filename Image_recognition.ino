#include <SoftwareSerial.h>

//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(3, 2); //SIM800L Tx & Rx is connected to Arduino #3 & #2

// Define the LED pins
const int redLedPin = 9;
const int greenLedPin = 10;

int Sms = 0;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);

  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  // Set the LED pins as output
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  Serial.println("Initializing..."); 
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  delay(1000);

}

void loop() {
  updateSerial();
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();

    // Turn on/off the LEDs based on the command
    if (command == '1') {
      // Command to turn on the green LED
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(redLedPin, LOW);
      Sms = 0;
    }else if (command == '2') {
      // Command to turn on the red LED
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      if(Sms == 0){
        mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
        updateSerial();
        mySerial.println("AT+CMGS=\"+254712345678\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
        updateSerial();
        mySerial.print("Intruder Detected"); //text content
        updateSerial();
        mySerial.write(26);
        delay(1000);
        Sms = 1;
      }else{
        //Print Nothing
      }
    }else if(command == '3') {
      // Command to turn off the red LED and also turns off Green LED
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      Sms = 0;
    }
  }
}
void updateSerial(){
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
