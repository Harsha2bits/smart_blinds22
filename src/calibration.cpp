#include "calibration.h"

bool calibraitonLoop(AiEsp32RotaryEncoder& enc, int buttonPin, AccelStepper& motor, long encMotorMultiplier)
{

  Button2 caliButton;

  caliButton.begin(buttonPin);
  //caliButton.setLongClickHandler(longpress);


  while(true)
  {
     caliButton.loop();
    if (enc.encoderChanged())
    {
      Serial.print("Setting Value: ");
      Serial.println(enc.readEncoder());
      motor.moveTo(enc.readEncoder() * encMotorMultiplier);     
	}

    

    if (caliButton.wasPressed() && motor.distanceToGo() == 0)
    {
        // Serial.println(caliButton.read());
        if (caliButton.read() == double_click)
        {
            Serial.println(" Position confirmed");
            positionConfirmaitonLed();
            return true;
        }
    }

    motor.run();



  }
}




void calibration(AiEsp32RotaryEncoder& enc,int buttonPin,  long& minenc, long& maxenc, AccelStepper& motor, long encMotorMultiplier) 
{

    //ensuring that whenever calibration is called everything is reset to zero and sufficcent boundries are given
    enc.setEncoderValue(0);
    enc.setBoundaries(-200, 200, false);
    motor.setCurrentPosition(0);

    //long minenc, maxenc;
    Serial.println("Entering Calibration mode");
    Serial.println("Move till it reached the bottom");
    if (calibraitonLoop(enc, buttonPin, motor, encMotorMultiplier ))
    {
        enc.setEncoderValue(0);
        motor.setCurrentPosition(0);
        minenc = enc.readEncoder();
        Serial.println(minenc);
    }

  Serial.println("Move till it reached the bottom");
  if(calibraitonLoop(enc,buttonPin,motor, encMotorMultiplier))
  {
    maxenc = enc.readEncoder();
    motor.setCurrentPosition(maxenc * encMotorMultiplier);
    Serial.println(maxenc);
    enc.setBoundaries(minenc,maxenc,false);
  }
  Serial.println("Finished Calibration");

}