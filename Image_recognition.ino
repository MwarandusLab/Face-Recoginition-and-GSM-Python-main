#include <SoftwareSerial.h>

//Create software serial object to communicate with SIM800L
SoftwareSerial sim(7, 8); // TX, RX GSM Module

// Define the LED pins
int redLedPin = 3;
int greenLedPin = 2;

int Sms = 0;

bool SendSms = false;

int _timeout;
String _buffer;
String number = "+254712345678"; //-> change with your number


void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  // Set the LED pins as output
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  _buffer.reserve(50);

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
      delay(2000);
      Sms = 0;
    }else if (command == '2') {
      // Command to turn on the red LED
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      if(Sms == 0){
        sim.begin(9600);
        SendMessage();
        sim.end();
        Serial.begin(9600);
        Sms = 1;
      }else{
        //Do nothing
      }
      delay(2000);
    }else if(command == '3') {
      // Command to turn off the red LED and also turns off Green LED
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      Sms = 0;
    }
  }
}
String _readSerial() {
  _timeout = 0;
  while  (!sim.available() && _timeout < 12000  )
  {
    delay(13);
    _timeout++;
  }
  if (sim.available()) {
    return sim.readString();
  }
}
void SendMessage(){
  //Serial.println ("Sending Message");
  sim.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(200);
  //Serial.println ("Set SMS Number");
  sim.println("AT+CMGS=\"" + number + "\"\r"); //Mobile phone number to send message
  delay(200);
  String SMS = "Intruder Detected";
  sim.println(SMS);
  delay(100);
  sim.println((char)26);// ASCII code of CTRL+Z
  delay(200);
  _buffer = _readSerial();
}
