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
  heartbeat = Heartbeat(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback, &heartbeat);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback, &responseHandler);
  i2c = I2C(&serialIO, &callback, &responseHandler);
  pose = Pose(&serialIO, &callback, &responseHandler);
}

Drone::Drone(SerialPort serialPort) {
  serialIO = SerialIO(serialPort);
  callback = Callback();
  heartbeat = Heartbeat(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback, &heartbeat);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback, &responseHandler);
  i2c = I2C(&serialIO, &callback, &responseHandler);
  pose = Pose(&serialIO, &callback, &responseHandler);
}

Drone::Drone(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  callback = Callback();
  heartbeat = Heartbeat(&serialIO, &callback);
  responseHandler = ResponseHandler(&serialIO, &callback, &heartbeat);
  rc = RC(&serialIO, &callback, &responseHandler);
  gpio = GPIO(&serialIO, &callback, &responseHandler);
  i2c = I2C(&serialIO, &callback, &responseHandler);
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

  heartbeat.tick();
}

// 
// Pose Methods
//

// Async Getters
void Drone::getGPSData()                           { pose.getGPSData(); }
void Drone::getLongitude()                         { pose.getLongitude(); }
void Drone::getLatitude()                          { pose.getLatitude(); }
void Drone::getAltitude()                          { pose.getAltitude(); }
void Drone::getSatellites()                        { pose.getSatellites(); }
void Drone::getSpeed()                             { pose.getSpeed(); }
void Drone::getOrientation()                       { pose.getOrientation(); }
void Drone::longitudeCallback(void (*cb)(float))   { pose.longitudeCallback(cb); }
void Drone::latitudeCallback(void (*cb)(float))    { pose.latitudeCallback(cb); }
void Drone::altitudeCallback(void (*cb)(float))    { pose.altitudeCallback(cb); }
void Drone::satelliteCallback(void (*cb)(int16_t)) { pose.satelliteCallback(cb); }
void Drone::speedCallback(void (*cb)(float))       { pose.speedCallback(cb); }
void Drone::orientationCallback(void (*cb)(float)) { pose.orientationCallback(cb); }

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
void Drone::setAileron(int8_t value)                 { rc.setAileron(value); }
void Drone::setElevator(int8_t value)                { rc.setElevator(value); }
void Drone::setThrottle(int8_t value)                { rc.setThrottle(value); }
void Drone::setRudder(int8_t value)                  { rc.setRudder(value); }
void Drone::setFlightMode(int8_t value)              { rc.setFlightMode(value); }

// Async Getters
void Drone::getAileron()                             { rc.getAileron(); }
void Drone::getElevator()                            { rc.getElevator(); }
void Drone::getThrottle()                            { rc.getThrottle(); }
void Drone::getRudder()                              { rc.getRudder(); }
void Drone::getFlightMode()                          { rc.getFlightMode(); }
void Drone::aileronCallback(void (*cb)(int16_t))     { rc.aileronCallback(cb); }
void Drone::flightModeCallback(void (*cb)(int16_t))  { rc.flightModeCallback(cb); }
void Drone::throttleCallback(void (*cb)(int16_t))    { rc.throttleCallback(cb); }
void Drone::rudderCallback(void (*cb)(int16_t))      { rc.rudderCallback(cb); }
void Drone::elevatorCallback(void (*cb)(int16_t))    { rc.elevatorCallback(cb); }
void Drone::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron) {
  // TODO: rename RTEA to something less cryptic
  rc.sendRTEA(rudder, throttle, elevator, aileron);
}

// Sync Getters
int16_t Drone::getAileronSync()                         { return rc.getAileronSync(); }
int16_t Drone::getElevatorSync()                        { return rc.getElevatorSync(); }
int16_t Drone::getThrottleSync()                        { return rc.getThrottleSync(); }
int16_t Drone::getRudderSync()                          { return rc.getRudderSync(); }
int16_t Drone::getFlightModeSync()                      { return rc.getFlightModeSync(); }

//
// GPIO Methods
//

void Drone::pinMode(uint8_t pin, int logicLevel)                 { gpio.pinMode(pin, logicLevel); }
void Drone::digitalWrite(uint8_t pin, bool logicLevel)           { gpio.digitalWrite(pin, logicLevel); }
void Drone::analogWrite(uint8_t pin, uint8_t value)              { gpio.analogWrite(pin, value); }
void Drone::pulseIn(uint8_t pin)                                 { gpio.pulseIn(pin); }
void Drone::digitalRead(uint8_t pin)                             { gpio.digitalRead(pin); }
void Drone::analogRead(uint8_t pin)                              { gpio.analogRead(pin); }
void Drone::digitalReadCallback(void (*cb)(uint8_t), int pin)    { gpio.digitalReadCallback(cb, pin); }
void Drone::pulseInCallback(void (*cb)(long), uint8_t pin)       { gpio.pulseInCallback(cb, pin); }
void Drone::analogReadCallback(void (*cb)(uint8_t), uint8_t pin) { gpio.analogReadCallback(cb, pin); }
void Drone::attachServo(uint8_t servoNumber, uint8_t pin)        { gpio.attachServo(servoNumber, pin); }
void Drone::detachServo(uint8_t servoNumber)                     { gpio.detachServo(servoNumber); }
void Drone::writeServo(uint8_t servoNumber, uint8_t data)        { gpio.writeServo(servoNumber, data); }

// Sync Getters
int16_t Drone::pulseInSync(uint8_t pin)     { return gpio.pulseInSync(pin); }
int16_t Drone::digitalReadSync(uint8_t pin) { return gpio.digitalReadSync(pin); }
int16_t Drone::analogReadSync(uint8_t pin)  { return gpio.analogReadSync(pin); }



//
// I2C Methods
//

void Drone::setDeviceAddress(uint8_t id)      { i2c.setDeviceAddress(id); }
void Drone::beginTransmission()               { i2c.beginTransmission(); }
void Drone::endTransmission()                 { i2c.endTransmission(); }
void Drone::write(uint8_t data)               { i2c.write(data); }
void Drone::read()                            { i2c.read(); }
void Drone::wireRequest(uint8_t byteCount)    { i2c.wireRequest(byteCount); }
void Drone::readCallback(void (*cb)(uint8_t)) { i2c.readCallback(cb); }

// Sync Getters
int16_t Drone::readSync() { return i2c.readSync(); }

//
// Heartbeat Methods
//

void Drone::heartbeatLostCallback(void (*cb)(void)) { heartbeat.heartbeatLostCallback(cb); }
