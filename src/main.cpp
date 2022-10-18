//Calibration Steps

//include libraries
#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32
#include <iostream>
#include <esp_now.h>
#include "Button2.h"
#include "calibration.h"
#include "ledblinks.h"

// Encoder

#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25

#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

Button2 button;
bool calibrationDone = false;


int getCounter()
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
	if (rotaryEncoder.encoderChanged())
	{
		Serial.print("Value: ");
		Serial.println(rotaryEncoder.readEncoder());
	}

  button.loop();
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
      calibration(rotaryEncoder,ROTARY_ENCODER_BUTTON_PIN);
      calibrationDone = true;
    }
}












void setup() {
  // led
  pinMode(LED_BUILTIN, OUTPUT);
  //rtc_gpio_deinit(GPIO_NUM_33);

    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(-100, 100, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(0);
    Serial.begin(921600);
    button.begin(ROTARY_ENCODER_BUTTON_PIN);
    button.setLongClickHandler(longpress);

    while(!calibrationDone)
    {
            button.loop();
            calibrateLed();
    }


}


void loop()
{

  rotary_loop();

  //delay(50); //or do whatever you need to do...
}

