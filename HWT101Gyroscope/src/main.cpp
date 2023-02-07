#include <Arduino.h>
#include "HWT101Gyro.h"

HWT101Gyro HWT101(&Serial1, 15, 2);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  if (HWT101.available())
  {
    HWT101.printValue();
  }
}