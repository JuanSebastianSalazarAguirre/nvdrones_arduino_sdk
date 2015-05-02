#include "GPIO.h"
#include "IDs.h"
#include "IncomingPacket.h"
#include "Utils.h"

GPIO::GPIO() {};

GPIO::GPIO(SerialIO *_serialIO, Callback *_callbacks, ResponseHandler *_responseHandler) {
  serialIO = _serialIO;
  callbacks = _callbacks;
  responseHandler = _responseHandler;
}

void GPIO::digitalWrite(uint8_t pin, bool logicLevel) {
  serialIO->sendPacket((uint8_t)logicLevel, resourceID::digitalWrite, pin);
}

void GPIO::pinMode(uint8_t pin, int logicLevel) {
  serialIO->sendPacket((int16_t)logicLevel, resourceID::pinMode, pin);
}

void GPIO::digitalRead(uint8_t pin) {
  serialIO->sendPacket((int16_t)0, resourceID::digitalRead, pin);
}

void GPIO::analogWrite(uint8_t pin, uint8_t value) {
  serialIO->sendPacket(value, resourceID::analogWrite, pin);
}

void GPIO::pulseIn(uint8_t pin) {
  serialIO->sendPacket((int16_t)0, resourceID::pulseIn, pin);
}

void GPIO::analogRead(uint8_t pin) {
  serialIO->sendPacket((int16_t)0, resourceID::analogRead, pin);
}

void GPIO::digitalReadCallback(void (*cb)(byte), uint8_t pin) {
  if(pin == 1)
    callbacks->digitalRead1 = cb;
  else if(pin == 2)
    callbacks->digitalRead2 = cb;
  else if(pin == 3)
    callbacks->digitalRead3 = cb;
  else if(pin == 4)
    callbacks->digitalRead4 = cb;
  else if(pin == 5)
    callbacks->digitalRead5 = cb;
  else if(pin == 6)
    callbacks->digitalRead6 = cb;
  else if(pin == 7)
    callbacks->digitalRead7 = cb;
  else if(pin == 8)
    callbacks->digitalRead8 = cb;
  else if(pin == 9)
    callbacks->digitalRead9 = cb;
  else if(pin == 10)
    callbacks->digitalRead10 = cb;
}

int16_t GPIO::digitalReadSync(uint8_t pin) {
  digitalRead(pin);
  return Utils::blockForByteData(resourceID::digitalRead, pin, responseHandler);
}


void GPIO::pulseInCallback(void (*cb)(long), uint8_t pin) {
  if(pin == 1)
    callbacks->pulseIn1 = cb;
  if(pin == 2)
    callbacks->pulseIn2 = cb;
  if(pin == 3)
    callbacks->pulseIn3 = cb;
  if(pin == 4)
    callbacks->pulseIn4 = cb;
  if(pin == 5)
    callbacks->pulseIn5 = cb;
  if(pin == 6)
    callbacks->pulseIn6 = cb;
  if(pin == 7)
    callbacks->pulseIn7 = cb;
  if(pin == 8)
    callbacks->pulseIn8 = cb;
  if(pin == 9)
    callbacks->pulseIn9 = cb;
  if(pin == 10)
    callbacks->pulseIn10 = cb;
}

long GPIO::pulseInSync(uint8_t pin) {
  pulseIn(pin);
  return Utils::blockForLongData(resourceID::pulseIn, pin, responseHandler);
}

void GPIO::analogReadCallback(void (*cb)(byte), uint8_t pin) {
  if(pin == 1)
    callbacks->analogRead1 = cb;
  if(pin == 2)
    callbacks->analogRead2 = cb;
  if(pin == 3)
    callbacks->analogRead3 = cb;
  if(pin == 4)
    callbacks->analogRead4 = cb;
}

int GPIO::analogReadSync(uint8_t pin) {
  analogRead(pin);
  return Utils::blockForByteData(resourceID::digitalRead, pin, responseHandler);
}

void GPIO::interruptCallback(void (*cb)(void), uint8_t interrupt) {
  if (interrupt == 0) callbacks->interrupt0 = cb;
  if (interrupt == 1) callbacks->interrupt1 = cb;
}

void GPIO::attachServo(uint8_t servoNumber, uint8_t pin) {
  uint8_t actionID = 0;
  switch (servoNumber) {
    case 1:
      actionID = actionID::attachServo1;
      break;
    case 2:
      actionID = actionID::attachServo2;
      break;
    case 3:
      actionID = actionID::attachServo3;
      break;
    default:
      // TODO: add error handling.
      return;
  }

  serialIO->sendPacket(pin, resourceID::servo, actionID);
}

void GPIO::detachServo(uint8_t servoNumber) {
  uint8_t actionID = 0;
  switch (servoNumber) {
    case 1:
      actionID = actionID::detachServo1;
      break;
    case 2:
      actionID = actionID::detachServo2;
      break;
    case 3:
      actionID = actionID::detachServo3;
      break;
    default:
      // TODO: add error handling.
      return;
  }
  serialIO->sendPacket((int16_t)0, resourceID::servo, actionID);
}

void GPIO::writeServo(uint8_t servoNumber, uint8_t data) {
  uint8_t actionID = 0;
  switch (servoNumber) {
    case 1:
      actionID = actionID::writeServo1;
      break;
    case 2:
      actionID = actionID::writeServo2;
      break;
    case 3:
      actionID = actionID::writeServo3;
      break;
    default:
      // TODO: add error handling.
      return;
  }
  serialIO->sendPacket(data, resourceID::servo, actionID);
}
