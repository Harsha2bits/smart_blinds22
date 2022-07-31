//include libraries
#include <Arduino.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
  // set the speed at 5 rpm
  myStepper.setSpeed(5);
  // initialize the serial port



void setup() {
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT); // set the digital pin as output:
}

void loop() {
  if (Serial.available()) { // if there is data comming
    String command = Serial.readStringUntil('\n'); // read string until newline character
    //Serial.println("THE INPUT IS " + command );

    if (command == "open")
    {
      digitalWrite(LED_BUILTIN, HIGH); // turn on LED
      Serial.println("Turn LED ON");
      myStepper.step(stepsPerRevolution/2);
    }
    else if (command == "close")
    {
      digitalWrite(LED_BUILTIN, LOW);  // turn off LED
      Serial.println("Turn LED OFF");
      myStepper.step(-stepsPerRevolution/2);
    }
  }
  //else
    //Serial.println("No Serial Detected");
}