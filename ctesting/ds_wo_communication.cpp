//Add necessary libraries
#include <Arduino.h>
#include <esp_now.h>  //To access the esp now functions
#include <WiFi.h>     //To Add Wifi Capabilities on ESP32

#include <AccelStepper.h>  //Stepper motor library
#include <Encoder.h>  // Not sure if we need it
#include "AiEsp32RotaryEncoder.h"  // Encoder library


#include <iostream>
#include <string>


#include "driver/gpio.h"
#include "driver/rtc_io.h"

using namespace std;

const int stepsPerRevolution = 2038;  // change this to fit the number of steps per revolution

// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

#define FULLSTEP 4
#define HALFSTEP 8

//Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

AccelStepper myStepper(FULLSTEP, IN1, IN3, IN2, IN4);


//sleep
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5
// #define wakeup_level 0

// Encoder

#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25


#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);




//Register Memory Variables
RTC_DATA_ATTR int counter = 0;  //Stores new counter value
RTC_DATA_ATTR int counterprev = 0; // stores previous counter value
RTC_DATA_ATTR int position = 0; // defining position of stepper as 1,2,3
RTC_DATA_ATTR int motorPosition = 0;  //stores motor position
RTC_DATA_ATTR int wakeup_level = 0;
int interrupt = 0;
static unsigned long lastEncoderChanged = 0;

// Other variables
String command;
int received = 0;




//Encoder function

 int getCounter()
{
    return (int)rotaryEncoder.readEncoder() ;
}

void setCounter(int i)
{
  rotaryEncoder.setEncoderValue(i);
}

void rotary_onButtonClick()
{
    static unsigned long lastTimePressed = 0;
    if (millis() - lastTimePressed < 200)
        return;
    lastTimePressed = millis();

    Serial.print("Resetting Counter Value ");
    rotaryEncoder.setEncoderValue(51);
}



void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}


//wakeup reason function
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}



// Setup function


void setup() {
  
  //led
  pinMode(LED_BUILTIN, OUTPUT);
  //rtc_gpio_deinit(GPIO_NUM_33); 

  //Encoder
    // Initialize encoder pins
  //pinMode(ENCODER_CLK, INPUT);
  //pinMode(ENCODER_DT, INPUT);
  //pinMode(ENCODER_SW, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, FALLING);
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.setBoundaries(0, 102, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
    rotaryEncoder.setAcceleration(0);
    //rotaryEncoder.setEncoderValue(counterprev);

    
    if(esp_sleep_get_wakeup_cause()== ESP_SLEEP_WAKEUP_EXT0)
    {
      Serial.println("wakeup because of encoder");
      rotaryEncoder.setEncoderValue(counterprev + 1);
    } // set default to 92.1 MHz
      else rotaryEncoder.setEncoderValue(counterprev);
  
    myStepper.setCurrentPosition(motorPosition);
    
    myStepper.setMaxSpeed(1000.0);
	  myStepper.setAcceleration(50.0);
	  myStepper.setSpeed(200);
	  //myStepper.moveTo(2038);
    //myStepper.run();

    // Set the baud rate for serial communication with ESP
    Serial.begin(921600);

//code for sleep interrupts

esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, wakeup_level);
print_wakeup_reason();

//delay(3000);
}


void loop() {

int change = 0;
int countertemp=0;
int serialavailable = Serial.available();

    countertemp = getCounter();
    if (countertemp!= counterprev && countertemp>=0 && countertemp <=102 )
    {

      interrupt = 1;

      Serial.println("Entered encoder loop");
      
      /*
  Each counter step is equal to 4 steps of stepper.
  So to make 90 degree turn we need 512 steps which is 128 steps
   counter= 0 is position 0
   counter = 64 is pisition 1
   counter =128 is pisition 2
  */
      // change = countertemp - counterprev;
      lastEncoderChanged = millis();

      motorPosition = countertemp * 10;

      if (countertemp <= 0)
        position = 0;
      else if (countertemp >= 50 || countertemp <= 52)
        position = 1;
      else if (countertemp >= 102)
        position = 2;

      counterprev = countertemp;


      Serial.println(motorPosition);
      Serial.println(myStepper.currentPosition());
    }

myStepper.moveTo(motorPosition);
          //myStepper.run();
          
          
          
           if(myStepper.distanceToGo()!=0)
          {

            myStepper.run();
          }
          else
          {
              digitalWrite(IN1, LOW);
              digitalWrite(IN2, LOW);
              digitalWrite(IN3, LOW);
              digitalWrite(IN4, LOW);
          }

  Serial.printf("Counter:%i, M pos set: %i, M current pos: %i, M target pos: %i , M Distance: %i \n", getCounter(),
                        motorPosition, myStepper.currentPosition(), myStepper.targetPosition(), myStepper.distanceToGo());

          
          
          
          if (interrupt == 1 || myStepper.distanceToGo() != 0)
          {
            if (millis() - lastEncoderChanged > 7000 && myStepper.distanceToGo() == 0)
            {
              Serial.print(millis() - lastEncoderChanged > 7000);
              Serial.println("Going to Sleep as no inpur received");
              // wakeup_level=!gpio_get_level(GPIO_NUM_25);
              Serial.printf("Wakeup_level is:%i \n", wakeup_level);
              esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
              esp_deep_sleep_start();
              Serial.println("This should not be printed");
            }
            else
              return;
    }
    else 
    { 
      Serial.println("Regular Sleep as no inpur received");
      //wakeup_level=!gpio_get_level(GPIO_NUM_33);
      //Serial.printf("Wakeup_level is:%i \n", wakeup_level);

      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
      
    }




}