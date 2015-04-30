#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Drone.h"
#include "ResponseHandler.h"
#include <avr/io.h>


//
// Constructors
//

Drone::Drone() {
  serialIO = SerialIO(serialPort0);
  callback = Callback();
  responseHandler = ResponseHandler(&serialIO, &callback);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback);
  i2c = I2C(&serialIO, &callback);
  pose = Pose(&serialIO, &callback, &responseHandler);
}

Drone::Drone(SerialPort serialPort) {
  serialIO = SerialIO(serialPort);
  callback = Callback();
  responseHandler = ResponseHandler(&serialIO, &callback);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback);
  i2c = I2C(&serialIO, &callback);
  pose = Pose(&serialIO, &callback, &responseHandler);
}

Drone::Drone(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  callback = Callback();
  responseHandler = ResponseHandler(&serialIO, &callback);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback);
  i2c = I2C(&serialIO, &callback);
  pose = Pose(&serialIO, &callback, &responseHandler);
}

//
// Initialization methods
//

// TODO: figure out how to make this work in the constructor.
void Drone::initialize() {
  serialIO.softwareSerial.begin(57600);
}

void Drone::arm() {
  rc.setRudder(-100);
  rc.setThrottle(-100);
  rc.setElevator(-100);
  rc.setAileron(-100);
  delay(500);
  rc.setRudder(0);
  rc.setThrottle(-100);
  rc.setElevator(0);
  rc.setAileron(0);
}


void Drone::listen() {
  responseHandler.listen();
}

// 
// Pose Methods
//

// Async Getters
void Drone::getGPSData()                              { pose.getGPSData(); }
void Drone::getLongitude()                            { pose.getLongitude(); }
void Drone::getLatitude()                             { pose.getLatitude(); }
void Drone::getAltitude()                             { pose.getAltitude(); }
void Drone::getSatellites()                           { pose.getSatellites(); }
void Drone::getSpeed()                                { pose.getSpeed(); }
void Drone::getOrientation()                          { pose.getOrientation(); }
void Drone::setLongitudeCallback(void (*cb)(float))   { pose.setLongitudeCallback(cb); }
void Drone::setLatitudeCallback(void (*cb)(float))    { pose.setLatitudeCallback(cb); }
void Drone::setAltitudeCallback(void (*cb)(float))    { pose.setAltitudeCallback(cb); }
void Drone::setSatelliteCallback(void (*cb)(int16_t)) { pose.setSatelliteCallback(cb); }
void Drone::setSpeedCallback(void (*cb)(float))       { pose.setSpeedCallback(cb); }
void Drone::setOrientationCallback(void (*cb)(float)) { pose.setOrientationCallback(cb); }

// Sync Getters
float Drone::getLatitudeSync()      { return pose.getLatitudeSync(); }
float Drone::getLongitudeSync()     { return pose.getLongitudeSync(); }
float Drone::getAltitudeSync()      { return pose.getAltitudeSync(); }
int16_t Drone::getSatellitesSync()  { return pose.getSatellitesSync(); }
float Drone::getSpeedSync()         { return pose.getSpeedSync(); }
float Drone::getOrientationSync()   { return pose.getOrientationSync(); }

//
// TODO: rename RC
// RC Methods
//

// Setters
void Drone::setAileron(int8_t value)                    { rc.setAileron(value); }
void Drone::setElevator(int8_t value)                   { rc.setElevator(value); }
void Drone::setThrottle(int8_t value)                   { rc.setThrottle(value); }
void Drone::setRudder(int8_t value)                     { rc.setRudder(value); }
void Drone::setFlightMode(int8_t value)                 { rc.setFlightMode(value); }

// Async Getters
void Drone::getAileron()                                { rc.getAileron(); }
void Drone::getElevator()                               { rc.getElevator(); }
void Drone::getThrottle()                               { rc.getThrottle(); }
void Drone::getRudder()                                 { rc.getRudder(); }
void Drone::getFlightMode()                             { rc.getFlightMode(); }
void Drone::setAileronCallback(void (*cb)(int16_t))     { rc.setAileronCallback(cb); }
void Drone::setFlightModeCallback(void (*cb)(int16_t))  { rc.setFlightModeCallback(cb); }
void Drone::setThrottleCallback(void (*cb)(int16_t))    { rc.setThrottleCallback(cb); }
void Drone::setRudderCallback(void (*cb)(int16_t))      { rc.setRudderCallback(cb); }
void Drone::setElevatorCallback(void (*cb)(int16_t))    { rc.setElevatorCallback(cb); }
void Drone::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron) {
  // TODO: rename RTEA to something less cryptic
  rc.sendRTEA(rudder, throttle, elevator, aileron);
}

// Sync Getters
int16_t Drone::getAileronSync()                         { rc.getAileronSync(); }
int16_t Drone::getElevatorSync()                        { rc.getElevatorSync(); }
int16_t Drone::getThrottleSync()                        { rc.getThrottleSync(); }
int16_t Drone::getRudderSync()                          { rc.getRudderSync(); }
int16_t Drone::getFlightModeSync()                      { rc.getFlightModeSync(); }

//
// GPIO Methods
//

void Drone::pinMode(uint8_t pin, int logicLevel)                    { gpio.pinMode(pin, logicLevel); }
void Drone::digitalWrite(uint8_t pin, bool logicLevel)              { gpio.digitalWrite(pin, logicLevel); }
void Drone::analogWrite(uint8_t pin, uint8_t value)                 { gpio.analogWrite(pin, value); }
void Drone::pulseIn(uint8_t pin)                                    { gpio.pulseIn(pin); }
void Drone::digitalRead(uint8_t pin)                                { gpio.digitalRead(pin); }
void Drone::analogRead(uint8_t pin)                                 { gpio.analogRead(pin); }
void Drone::setDigitalReadCallback(void (*cb)(uint8_t), int pin)    { gpio.setDigitalReadCallback(cb, pin); }
void Drone::setPulseInCallback(void (*cb)(long), uint8_t pin)       { gpio.setPulseInCallback(cb, pin); }
void Drone::setAnalogReadCallback(void (*cb)(uint8_t), uint8_t pin) { gpio.setAnalogReadCallback(cb, pin); }
void Drone::attachServo(uint8_t servoNumber, uint8_t pin)           { gpio.attachServo(servoNumber, pin); }
void Drone::detachServo(uint8_t servoNumber)                        { gpio.detachServo(servoNumber); }
void Drone::writeServo(uint8_t servoNumber, uint8_t data)           { gpio.writeServo(servoNumber, data); }

//
// I2C Methods
//

void Drone::deviceID(uint8_t id)                  { i2c.deviceID(id); }
void Drone::beginTransmission()                   { i2c.beginTransmission(); }
void Drone::endTransmission()                     { i2c.endTransmission(); }
void Drone::write(uint8_t data)                   { i2c.write(data); }
void Drone::read()                                { i2c.read(); }
void Drone::wireRequest(uint8_t byteCount)        { i2c.wireRequest(byteCount); }
void Drone::setReadCallback(void (*cb)(uint8_t))  { i2c.setReadCallback(cb); }
