//Calibration Steps

//include libraries
#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"
#include "variables.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32
#include <iostream>
#include <esp_now.h>
#include "Button2.h"
#include "calibration.h"
#include "ledblinks.h"
#include "deepsleep.h"

// // Encoder

// #define ROTARY_ENCODER_A_PIN 33
// #define ROTARY_ENCODER_B_PIN 32
// #define ROTARY_ENCODER_BUTTON_PIN 25

// #define ROTARY_ENCODER_STEPS 4



// AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

// Button2 button;

// RTC_DATA_ATTR int wakeup_level = 0;
// RTC_DATA_ATTR bool calibrationDone = false;
// RTC_DATA_ATTR long min = 0;


// static unsigned long lastEncoderChanged = 0;

// //sleep
// #define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
// #define TIME_TO_SLEEP 5
// // #define wakeup_level 0




    int
    getCounter()
{
    return (int)rotaryEncoder.readEncoder() ;
}

void setCounter(int i)
{
  rotaryEncoder.setEncoderValue(i);
}



void IRAM_ATTR readEncoderISR()
{
	rotaryEncoder.readEncoder_ISR();
}



void rotary_loop()
{
	//dont print anything unless value changed
	button.loop();
  if (rotaryEncoder.encoderChanged())
	{
		Serial.print("Value: ");
		Serial.println(rotaryEncoder.readEncoder());
    myStepper.moveTo(rotaryEncoder.readEncoder() * encMotorMultiplier); //set motor position
    lastEncoderChanged = millis();
  }

  myStepper.run();
}

void longpress(Button2& btn) 
{
    unsigned int time = btn.wasPressedFor();
    if (time > 1500) 
    {
      Serial.print("a really really long time.");
      Serial.print(" (");        
      Serial.print(time);        
      Serial.println(" ms)");
      Serial.println("Entering Calibration mode");
      calibration(rotaryEncoder,ROTARY_ENCODER_BUTTON_PIN,minenc,maxenc,myStepper,encMotorMultiplier);
      calibrationDone = true;
    }
}



void sendSleep()
{
  if(millis()-lastEncoderChanged > 7000)
  {
    Serial.print(millis() - lastEncoderChanged > 7000);
    Serial.println("Going to Sleep as no inpur received");
    // wakeup_level=!gpio_get_level(GPIO_NUM_25);
    Serial.printf("Wakeup_level is:%i \n", wakeup_level);

    encPosition = rotaryEncoder.readEncoder();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();


    Serial.println("This should not be printed");
  }

}








void setup() {
  // led
  Serial.begin(921600);
  pinMode(LED_BUILTIN, OUTPUT);
  //rtc_gpio_deinit(GPIO_NUM_33);

    //rotaryEncoder setup
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(minenc,maxenc, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setEncoderValue(encPosition);
    rotaryEncoder.setAcceleration(0);
    
    //stepper setup
    myStepper.setCurrentPosition(encPosition * encMotorMultiplier);
    myStepper.setMaxSpeed(1000.0);
    myStepper.setAcceleration(50.0);
	  myStepper.setSpeed(200);
    
    //button setup
    button.begin(ROTARY_ENCODER_BUTTON_PIN);
    button.setLongClickHandler(longpress);

    while(!calibrationDone)
    {
            button.loop();
            calibrateLed();
    }
    lastEncoderChanged = millis();
    Serial.println("reached here");

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, wakeup_level);
    print_wakeup_reason();

}


void loop()
{
  

  rotary_loop();
  sendSleep();

  //delay(50); //or do whatever you need to do...
}

