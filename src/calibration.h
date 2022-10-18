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
#include "ledblinks.h"

bool calibraitonLoop(AiEsp32RotaryEncoder &enc, int ROTARY_ENCODER_BUTTON_PIN);

void calibration(AiEsp32RotaryEncoder &enc, int ROTARY_ENCODER_BUTTON_PIN);

#endif