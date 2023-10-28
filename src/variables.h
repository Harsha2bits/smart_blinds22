
#pragma once
#ifndef variables_h
#define variables_h

#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"
#include "Button2.h"
#include "AccelStepper.h"

// Encoder
#define ROTARY_ENCODER_A_PIN 33
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 25
#define ROTARY_ENCODER_STEPS 4

//stepper Motor
// ULN2003 Motor Driver Pins
#define IN1 19
#define IN2 18
#define IN3 17
#define IN4 16

#define FULLSTEP 4
#define HALFSTEP 8


AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

AccelStepper myStepper(FULLSTEP, IN1, IN3, IN2, IN4);

Button2 button;


//RTC Attributesß
RTC_DATA_ATTR int wakeup_level = 0;
RTC_DATA_ATTR bool calibrationDone = false;
RTC_DATA_ATTR long minenc = -200;
RTC_DATA_ATTR long maxenc = 200;
RTC_DATA_ATTR long encPosition = 0;
//RTC_DATA_ATTR long maxMotorPosition = 200;
RTC_DATA_ATTR long encMotorMultiplier = 10;
//RTC_DATA_ATTR long motorPosition = 0;


//Gloabl Definitions

//sleep
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 10
// #define wakeup_level 0



//Global Variables
static unsigned long lastEncoderChanged = 0;
const int stepsPerRevolution = 2038;  // change this to fit the number of steps per revolution

#endif