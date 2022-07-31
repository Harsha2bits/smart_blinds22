#include <Arduino.h>
#include <WiFi.h>
#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

void setup() {
  // set the speed at 5 rpm
  myStepper.setSpeed(20);
  // initialize the serial port
  Serial.begin(921600);
}

void loop() {
  // step one revolution in one direction:
  
  for (int i = 0; i < 50; i++) {
  //whileSerial.println("clockwise");
  myStepper.step(stepsPerRevolution/2);
  delay(10);
  myStepper.step(-stepsPerRevolution/2);
  i++;
  }
  

}