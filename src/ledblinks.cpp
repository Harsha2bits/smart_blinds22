
#include "ledblinks.h"

void calibrateLed()
{
digitalWrite(LED_BUILTIN, HIGH); // turn off LED
delay(300);
digitalWrite(LED_BUILTIN, LOW); // turn off LED
delay(300);
}


void positionConfirmaitonLed()
{
    for (int i = 0; i < 5; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH); // turn off LED
        delay(200);
        digitalWrite(LED_BUILTIN, LOW); // turn off LED
        delay(100);

    }

}