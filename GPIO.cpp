#include "GPIO.h"

GPIO::GPIO() {};

GPIO::GPIO(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void GPIO::digitalWrite(uint8_t pin, bool logicLevel) {
  service->sendPacket((uint8_t)logicLevel, 6, pin);
}

void GPIO::pinMode(uint8_t pin, int logicLevel) {
  service->sendPacket((int16_t)logicLevel, 5, pin);
}

void GPIO::digitalRead(uint8_t pin) {
  service->sendPacket((int16_t)0, 8, pin);
}

void GPIO::analogWrite(uint8_t pin, uint8_t value) {
  service->sendPacket(value, 7, pin);
}

void GPIO::pulseIn(uint8_t pin) {
  service->sendPacket((int16_t)0, 16, pin);
}

void GPIO::analogRead(uint8_t pin) {
  service->sendPacket((int16_t)0, 17, pin);
}

void GPIO::setDigitalReadCallback(void (*cb)(byte), int pin) {
  if(pin == 1)
    (*myCallback).digitalRead1 = cb;
  else if(pin == 2)
    (*myCallback).digitalRead2 = cb;
  else if(pin == 3)
    (*myCallback).digitalRead3 = cb;
  else if(pin == 4)
    (*myCallback).digitalRead4 = cb;
  else if(pin == 5)
    (*myCallback).digitalRead5 = cb;
  else if(pin == 6)
    (*myCallback).digitalRead6 = cb;
  else if(pin == 7)
    (*myCallback).digitalRead7 = cb;
  else if(pin == 8)
    (*myCallback).digitalRead8 = cb;
  else if(pin == 9)
    (*myCallback).digitalRead9 = cb;
  else if(pin == 10)
    (*myCallback).digitalRead10 = cb;
}

void GPIO::setPulseInCallback(void (*cb)(long), uint8_t pin) {
  if(pin == 1)
    (*myCallback).pulseIn1 = cb;
  if(pin == 2)
    (*myCallback).pulseIn2 = cb;
  if(pin == 3)
    (*myCallback).pulseIn3 = cb;
  if(pin == 4)
    (*myCallback).pulseIn4 = cb;
  if(pin == 5)
    (*myCallback).pulseIn5 = cb;
  if(pin == 6)
    (*myCallback).pulseIn6 = cb;
  if(pin == 7)
    (*myCallback).pulseIn7 = cb;
  if(pin == 8)
    (*myCallback).pulseIn8 = cb;
  if(pin == 9)
    (*myCallback).pulseIn9 = cb;
  if(pin == 10)
    (*myCallback).pulseIn10 = cb;
}

void GPIO::setAnalogReadCallback(void (*cb)(byte), uint8_t pin) {
  if(pin == 1)
    (*myCallback).analogRead1 = cb;
  if(pin == 2)
    (*myCallback).analogRead2 = cb;
  if(pin == 3)
    (*myCallback).analogRead3 = cb;
  if(pin == 4)
    (*myCallback).analogRead4 = cb;
}

void GPIO::attachServo(uint8_t servoNumber, uint8_t pin) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 1;
  service->sendPacket(pin, 18, actionID);
}

void GPIO::detachServo(uint8_t servoNumber) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 3;
  service->sendPacket((int16_t)0, 18, actionID);
}

void GPIO::writeServo(uint8_t servoNumber, uint8_t data) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 2;
  service->sendPacket(data, 18, actionID);
}