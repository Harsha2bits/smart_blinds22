#include "calibration.h"

bool calibraitonLoop(AiEsp32RotaryEncoder& enc, int ROTARY_ENCODER_BUTTON_PIN)
{

  Button2 caliButton;

  caliButton.begin(ROTARY_ENCODER_BUTTON_PIN);
  //caliButton.setLongClickHandler(longpress);


  while(true)
  {
     caliButton.loop();
    if (enc.encoderChanged())
    {
      Serial.print("Setting Value: ");
      Serial.println(enc.readEncoder());
	}

  if (caliButton.wasPressed())
  {
      //Serial.println(caliButton.read());
      if (caliButton.read()==double_click) 
      {
        Serial.println(" Position confirmed");
        positionConfirmaiton();
        return true;
      }
  }
  }
}

void calibration(AiEsp32RotaryEncoder& enc,int ROTARY_ENCODER_BUTTON_PIN)
{
    long minb, maxb;

    Serial.println("Entering Calibration mode");
    Serial.println("Move till it reached the bottom");
    if (calibraitonLoop(enc, ROTARY_ENCODER_BUTTON_PIN))
    {
        enc.setEncoderValue(0);
        minb = enc.readEncoder();
        Serial.println(minb);
    }

  Serial.println("Move till it reached the bottom");
  if(calibraitonLoop(enc,ROTARY_ENCODER_BUTTON_PIN))
  {
    maxb = enc.readEncoder();
    Serial.println(maxb);
    enc.setBoundaries(minb,maxb,false);
  }
  Serial.println("Finished Calibration");

}