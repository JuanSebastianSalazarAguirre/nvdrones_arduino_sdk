

#include "Callback.h"

#include <Arduino.h>

void defaultErrorHandler(int16_t error) {
  Serial.print("received error code: "); Serial.println(error);
}

Callback::Callback():
errorHandler(&defaultErrorHandler)
{

}
