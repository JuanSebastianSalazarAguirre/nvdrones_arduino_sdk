#include "GPIO.h"
#include "IDs.h"
#include "IncomingPacket.h"
#include "Utils.h"


// TODO: add error checking for valid pins/servos?

GPIO::GPIO() {};

GPIO::GPIO(SerialIO *serialIO, Callback *callbacks, IncomingPacketReader *incomingPacketReader):
_serialIO(serialIO),
_callbacks(callbacks),
_incomingPacketReader(incomingPacketReader)
{

}

void GPIO::digitalWrite(int16_t pin, bool logicLevel) {
  _serialIO->sendPacket((uint8_t)logicLevel, resourceID::digitalWrite, (uint8_t)pin);
}

void GPIO::pinMode(int16_t pin, int16_t logicLevel) {
  _serialIO->sendPacket(logicLevel, resourceID::pinMode, (uint8_t)pin);
}

void GPIO::digitalRead(int16_t pin) {
  _serialIO->sendPacket((int16_t)0, resourceID::digitalRead, (uint8_t)pin);
}

void GPIO::analogWrite(int16_t pin, int16_t value) {
  _serialIO->sendPacket((uint8_t)value, resourceID::analogWrite, (uint8_t)pin);
}

void GPIO::pulseIn(int16_t pin, int16_t value) {
  // combine the two values.
  _serialIO->sendPacket((int16_t)0, resourceID::pulseIn, (uint8_t)pin);
}

void GPIO::pulseIn(int16_t pin, int16_t value, uint32_t timeout) {
  // combine the three values
  _serialIO->sendPacket((int16_t)0, resourceID::pulseIn, 0);
}

uint32_t GPIO::pulseInSync(int16_t pin, int16_t value) {
  pulseIn(pin, value);
  return Utils::blockForFloatData(resourceID::pulseIn, pin, _incomingPacketReader);
}

void GPIO::analogRead(int16_t pin) {
  _serialIO->sendPacket((int16_t)0, resourceID::analogRead, (uint8_t)pin);
}

void GPIO::digitalReadCallback(void (*cb)(int16_t), int16_t pin) {
  if(pin == 1)
    _callbacks->digitalRead1 = cb;
  else if(pin == 2)
    _callbacks->digitalRead2 = cb;
  else if(pin == 3)
    _callbacks->digitalRead3 = cb;
  else if(pin == 4)
    _callbacks->digitalRead4 = cb;
  else if(pin == 5)
    _callbacks->digitalRead5 = cb;
  else if(pin == 6)
    _callbacks->digitalRead6 = cb;
  else if(pin == 7)
    _callbacks->digitalRead7 = cb;
  else if(pin == 8)
    _callbacks->digitalRead8 = cb;
  else if(pin == 9)
    _callbacks->digitalRead9 = cb;
  else if(pin == 10)
    _callbacks->digitalRead10 = cb;
}

int16_t GPIO::digitalReadSync(int16_t pin) {
  digitalRead(pin);
  return Utils::blockForByteData(resourceID::digitalRead, pin, _incomingPacketReader);
}


void GPIO::pulseInCallback(void (*cb)(uint32_t), int16_t pin) {
  if(pin == 1)
    _callbacks->pulseIn1 = cb;
  if(pin == 2)
    _callbacks->pulseIn2 = cb;
  if(pin == 3)
    _callbacks->pulseIn3 = cb;
  if(pin == 4)
    _callbacks->pulseIn4 = cb;
  if(pin == 5)
    _callbacks->pulseIn5 = cb;
  if(pin == 6)
    _callbacks->pulseIn6 = cb;
  if(pin == 7)
    _callbacks->pulseIn7 = cb;
  if(pin == 8)
    _callbacks->pulseIn8 = cb;
  if(pin == 9)
    _callbacks->pulseIn9 = cb;
  if(pin == 10)
    _callbacks->pulseIn10 = cb;
}

void GPIO::analogReadCallback(void (*cb)(int16_t), int16_t pin) {
  if(pin == 1)
    _callbacks->analogRead1 = cb;
  if(pin == 2)
    _callbacks->analogRead2 = cb;
  if(pin == 3)
    _callbacks->analogRead3 = cb;
  if(pin == 4)
    _callbacks->analogRead4 = cb;
}

int GPIO::analogReadSync(int16_t pin) {
  analogRead(pin);
  return Utils::blockForByteData(resourceID::digitalRead, pin, _incomingPacketReader);
}

void GPIO::interruptCallback(void (*cb)(void), int16_t interrupt) {
  if (interrupt == 0) _callbacks->interrupt0 = cb;
  if (interrupt == 1) _callbacks->interrupt1 = cb;
}

void GPIO::attachServo(int16_t servoNumber, int16_t pin) {
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

  _serialIO->sendPacket((uint8_t)pin, resourceID::servo, actionID);
}

void GPIO::detachServo(int16_t servoNumber) {
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
  _serialIO->sendPacket((int16_t)0, resourceID::servo, actionID);
}

void GPIO::writeServo(int16_t servoNumber, int16_t data) {
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
  _serialIO->sendPacket((uint8_t)data, resourceID::servo, actionID);
}
