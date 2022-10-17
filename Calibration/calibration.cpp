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

// Encoder

#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25

#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

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




typedef enum
{
	BUT_DOWN = 0,
	BUT_PUSHED = 1,
	BUT_UP = 2,
	BUT_RELEASED = 3,
	BUT_DISABLED = 99,
} ButtonState;

ButtonState buttonState;


typedef struct {

    ButtonState buttonState;
    bool longPress = false;
    int lastPressDuration = 0;

} buttonData;

buttonData encoderButton;


const int longPressTime  = 1000;   //we can convert this to define instead of const int






void IRAM_ATTR readButtonISR()
{
    static unsigned long buttonTime = 0;
    static unsigned long lastButtonTime = 0;
    static unsigned long pressTime = 0;
    static unsigned long releaseTime = 0;
    static bool buttonValidInput = false;
    static bool previous_butt_state = false;
    unsigned long pressDuration = 0;

    buttonTime = millis();
    uint8_t butt_state = !digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    
    if (buttonTime - lastButtonTime > 250)
    {
        buttonValidInput = true;
        lastButtonTime = buttonTime;
    }

    if(buttonValidInput)
    {
        encoderButton.longPress = false;

        if (butt_state && !previous_butt_state)
        {
            previous_butt_state = true;
            Serial.println("Button Pushed");
            encoderButton.buttonState = BUT_PUSHED;
            pressTime = buttonTime;
        }
        else if (!butt_state && previous_butt_state)
        {
            previous_butt_state = false;
            Serial.println("Button Released");
            encoderButton.buttonState = BUT_RELEASED;
            releaseTime = buttonTime;
            pressDuration = releaseTime - pressTime;
            if(pressDuration>longPressTime )
            {
                encoderButton.longPress = true;
                encoderButton.lastPressDuration = pressDuration;
                Serial.println("Long Press recieved: Will enter calibration mode");
            }
            else
            {
                encoderButton.longPress = false;
                encoderButton.lastPressDuration = pressDuration;
                Serial.println("Short Press recieved: Will go to center");
            }
        }
        else
        {
            buttonState = (butt_state ? BUT_DOWN : BUT_UP);
            Serial.println(butt_state ? "BUT_DOWN" : "BUT_UP");
        }

        
    }


}


void rotary_loop()
{
	//dont print anything unless value changed
	if (rotaryEncoder.encoderChanged())
	{
		Serial.print("Value: ");
		Serial.println(rotaryEncoder.readEncoder());
	}
    if (encoderButton.longPress)
    {
        int i;
        for (i = 0; i < 5;i++)
        {
            digitalWrite(LED_BUILTIN, HIGH); // turn off LED
            delay(200);
            digitalWrite(LED_BUILTIN, LOW); // turn off LED
            delay(200);
        }

    }
}




void setup() {
  
  //led
  pinMode(LED_BUILTIN, OUTPUT);
  rtc_gpio_deinit(GPIO_NUM_33);

    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR,readButtonISR);
    rotaryEncoder.setBoundaries(0, 102, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(0);

    Serial.begin(921600);
}


void loop()
{
	//in loop call your custom function which will process rotary encoder values
	int change = 0;
    int countertemp=0;
    int serialavailable = Serial.available();

    countertemp = getCounter();
    
	//delay(50); //or do whatever you need to do...
}

