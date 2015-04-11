/*
 * AvantGPIO.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: amey
 */
#include "AvantGPIO.h"



//************************************************
//AvantGPIO Class Implementation
//************************************************
AvantGPIO::AvantGPIO() {};

AvantGPIO::AvantGPIO(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantGPIO::digitalWrite(uint8_t pin, bool logicLevel) {
  service->sendPacket((uint8_t)logicLevel, 6, pin);
}

void AvantGPIO::pinMode(uint8_t pin, int logicLevel) {
  service->sendPacket((int16_t)logicLevel, 5, pin);
}

void AvantGPIO::digitalRead(uint8_t pin) {
  service->sendPacket((int16_t)0, 8, pin);
}

void AvantGPIO::analogWrite(uint8_t pin, uint8_t value) {
  service->sendPacket(value, 7, pin);
}

void AvantGPIO::pulseIn(uint8_t pin) {
  service->sendPacket((int16_t)0, 16, pin);
}

void AvantGPIO::analogRead(uint8_t pin) {
  service->sendPacket((int16_t)0, 17, pin);
}

void AvantGPIO::digitalReadCallback(void (*function)(byte), int pin) {
  if(pin == 1)
    (*myCallback).digitalRead1 = function;
  else if(pin == 2)
    (*myCallback).digitalRead2 = function;
  else if(pin == 3)
    (*myCallback).digitalRead3 = function;
  else if(pin == 4)
    (*myCallback).digitalRead4 = function;
  else if(pin == 5)
    (*myCallback).digitalRead5 = function;
  else if(pin == 6)
    (*myCallback).digitalRead6 = function;
  else if(pin == 7)
    (*myCallback).digitalRead7 = function;
  else if(pin == 8)
    (*myCallback).digitalRead8 = function;
  else if(pin == 9)
    (*myCallback).digitalRead9 = function;
  else if(pin == 10)
    (*myCallback).digitalRead10 = function;
}

void AvantGPIO::pulseInCallback(void (*function)(long), uint8_t pin) {
  if(pin == 1)
    (*myCallback).pulseIn1 = function;
  if(pin == 2)
    (*myCallback).pulseIn2 = function;
  if(pin == 3)
    (*myCallback).pulseIn3 = function;
  if(pin == 4)
    (*myCallback).pulseIn4 = function;
  if(pin == 5)
    (*myCallback).pulseIn5 = function;
  if(pin == 6)
    (*myCallback).pulseIn6 = function;
  if(pin == 7)
    (*myCallback).pulseIn7 = function;
  if(pin == 8)
    (*myCallback).pulseIn8 = function;
  if(pin == 9)
    (*myCallback).pulseIn9 = function;
  if(pin == 10)
    (*myCallback).pulseIn10 = function;
}

void AvantGPIO::analogReadCallback(void (*function)(byte), uint8_t pin) {
  if(pin == 1)
    (*myCallback).analogRead1 = function;
  if(pin == 2)
    (*myCallback).analogRead2 = function;
  if(pin == 3)
    (*myCallback).analogRead3 = function;
  if(pin == 4)
    (*myCallback).analogRead4 = function;
}

void AvantGPIO::attachServo(uint8_t servoNumber, uint8_t pin) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 1;
  service->sendPacket(pin, 18, actionID);
}

void AvantGPIO::detachServo(uint8_t servoNumber) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 3;
  service->sendPacket((int16_t)0, 18, actionID);
}

void AvantGPIO::writeServo(uint8_t servoNumber, uint8_t data) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 2;
  service->sendPacket(data, 18, actionID);
}


