// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Drone.h"
#include <avr/io.h>

// ***********************************************
// Avant Class Implementation
// ***********************************************
Drone::Drone() {
  serialIO = SerialIO(serialPort0);
  callback = Callback();
  avantRC = AvantRC(&serialIO, &callback);
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}

Drone::Drone(SerialPort serialPort) {
  serialIO = SerialIO(serialPort);
  callback = Callback();
  avantRC = AvantRC(&serialIO, &callback);
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}
Drone::Drone(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  callback = Callback();
  avantRC = AvantRC(&serialIO, &callback);
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}

AvantGPIO& Drone::GPIO() {return avantGPIO;} 
AvantResponseHandler& Drone::avantResponseHandler(){return responseHandler;}
AvantRC& Drone::RC() {return avantRC;} //functionality for sending RC data to the drone
AvantI2C& Drone::I2C() {return avantI2C;}
AvantPose& Drone::pose() {return avantPose;}
AvantAutoPilot& Drone::AutoPilot() {return avantAutoPilot;}

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

