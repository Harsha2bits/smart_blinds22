/*
monitor_flags =
    --filter
    debug
    --filter
    send_on_enter
    --echo
    --eol
    LF
    */



//include libraries
#include <Arduino.h>
//#include <esp_now.h>
//#include <WiFi.h>


void setup() {
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT); // set the digital pin as output:
}

void loop() {
  if (Serial.available()) { // if there is data comming
    String command = Serial.readStringUntil('\n'); // read string until newline character
    Serial.println("THE INPUT IS " + command );

    if (command == "ON")
    {
      digitalWrite(LED_BUILTIN, HIGH); // turn on LED
      Serial.println("Turn LED ON");
    }
    else if (command == "OFF")
    {
      digitalWrite(LED_BUILTIN, LOW);  // turn off LED
      Serial.println("Turn LED OFF");
    }
  }
  //else
    //Serial.println("No Serial Detected");
}