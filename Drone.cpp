#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Drone.h"
#include <avr/io.h>

Drone::Drone() {
  serialIO = SerialIO(serialPort0);
  callback = Callback();
  avantRC = RC(&serialIO, &callback);
  avantGPIO = GPIO(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback);
  avantI2C = I2C(&serialIO, &callback);
  avantPose = Pose(&serialIO, &callback);
  avantAutoPilot = AutoPilot(&serialIO, &callback);
}

Drone::Drone(SerialPort serialPort) {
  serialIO = SerialIO(serialPort);
  callback = Callback();
  avantRC = RC(&serialIO, &callback);
  avantGPIO = GPIO(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback);
  avantI2C = I2C(&serialIO, &callback);
  avantPose = Pose(&serialIO, &callback);
  avantAutoPilot = AutoPilot(&serialIO, &callback);
}
Drone::Drone(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  callback = Callback();
  avantRC = RC(&serialIO, &callback);
  avantGPIO = GPIO(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback);
  avantI2C = I2C(&serialIO, &callback);
  avantPose = Pose(&serialIO, &callback);
  avantAutoPilot = AutoPilot(&serialIO, &callback);
}

GPIO& Drone::gpio() {return avantGPIO;} 
ResponseHandler& Drone::avantResponseHandler(){return responseHandler;}
RC& Drone::rc() {return avantRC;}
I2C& Drone::i2c() {return avantI2C;}
Pose& Drone::pose() {return avantPose;}
AutoPilot& Drone::autoPilot() {return avantAutoPilot;}

void Drone::initialize() {
  serialIO.softwareSerial.begin(57600);
}

void Drone::arm() {
  serialIO.sendPacket((int8_t)-100, 2, 1);
  serialIO.sendPacket((int8_t)-100, 2, 2);
  serialIO.sendPacket((int8_t)-100, 2, 3);
  serialIO.sendPacket((int8_t)-100, 2, 4);
  delay(500);
  serialIO.sendPacket((int8_t)0, 2, 1);
  serialIO.sendPacket((int8_t)-100, 2, 2);
  serialIO.sendPacket((int8_t)0, 2, 3);
  serialIO.sendPacket((int8_t)0, 2, 4);
}