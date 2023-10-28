/////////////////////////////////////////////////////////////////
/*
  Callibration code.
  Created by Harsh Nannur.
*/
/////////////////////////////////////////////////////////////////
#pragma once

#ifndef calibration_h
#define calibration_h

#include "Button2.h"
#include "AiEsp32RotaryEncoder.h"
#include "AccelStepper.h"
#include "ledblinks.h"
//#include "variables.h"

bool calibraitonLoop(AiEsp32RotaryEncoder &enc, int buttonPin, AccelStepper &motor, long encMotorMultiplier);

void calibration(AiEsp32RotaryEncoder &enc, int buttonPin, long &minenc, long &maxenc, AccelStepper &motor, long encMotorMultiplier);

#endif