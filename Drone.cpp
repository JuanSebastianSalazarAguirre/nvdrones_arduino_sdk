#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Drone.h"

#include <avr/io.h>


//
// Constructors
//

Drone::Drone() {
  serialIO = SerialIO(serialPort0);
  callback = Callback();
  incomingPacketReader = IncomingPacketReader(&serialIO);
  vitals = Vitals(&serialIO, &incomingPacketReader, &callback);
  responseHandler = ResponseHandler(&serialIO, &incomingPacketReader, &callback, &vitals);
  rc = RC(&serialIO, &callback, &incomingPacketReader);
  gpio = GPIO(&serialIO, &callback, &incomingPacketReader);
  i2c = I2C(&serialIO, &callback, &incomingPacketReader);
  pose = Pose(&serialIO, &callback, &incomingPacketReader);
}

Drone::Drone(SerialPort serialPort) {
  serialIO = SerialIO(serialPort);
  callback = Callback();
  incomingPacketReader = IncomingPacketReader(&serialIO);
  vitals = Vitals(&serialIO, &incomingPacketReader, &callback);
  responseHandler = ResponseHandler(&serialIO, &incomingPacketReader, &callback, &vitals);
  rc = RC(&serialIO, &callback, &incomingPacketReader);
  gpio = GPIO(&serialIO, &callback, &incomingPacketReader);
  i2c = I2C(&serialIO, &callback, &incomingPacketReader);
  pose = Pose(&serialIO, &callback, &incomingPacketReader);
}

Drone::Drone(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  callback = Callback();
  incomingPacketReader = IncomingPacketReader(&serialIO);
  vitals = Vitals(&serialIO, &incomingPacketReader, &callback);
  responseHandler = ResponseHandler(&serialIO, &incomingPacketReader, &callback, &vitals);
  rc = RC(&serialIO, &callback, &incomingPacketReader);
  gpio = GPIO(&serialIO, &callback, &incomingPacketReader);
  i2c = I2C(&serialIO, &callback, &incomingPacketReader);
  pose = Pose(&serialIO, &callback, &incomingPacketReader);
}

//
// Initialization methods
//

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

  vitals.tick();
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
void Drone::setAileron(int16_t value)                 { rc.setAileron(value); }
void Drone::setElevator(int16_t value)                { rc.setElevator(value); }
void Drone::setThrottle(int16_t value)                { rc.setThrottle(value); }
void Drone::setRudder(int16_t value)                  { rc.setRudder(value); }
void Drone::setFlightMode(int16_t value)              { rc.setFlightMode(value); }

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
void Drone::setAileronElevatorRudderThrottle(int16_t aileron,
  int16_t elevator, int16_t rudder, int16_t throttle) {
  rc.setAileronElevatorRudderThrottle(aileron, elevator, rudder, throttle);
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

void Drone::pinMode(int16_t pin, int16_t logicLevel)              { gpio.pinMode(pin, logicLevel); }
void Drone::digitalWrite(int16_t pin, bool logicLevel)            { gpio.digitalWrite(pin, logicLevel); }
void Drone::analogWrite(int16_t pin, int16_t value)               { gpio.analogWrite(pin, value); }
void Drone::pulseIn(int16_t pin, int16_t value)                   { gpio.pulseIn(pin, value); }
void Drone::pulseIn(int16_t pin, int16_t value, uint32_t timeout) { gpio.pulseIn(pin, value, timeout); }
void Drone::digitalRead(int16_t pin)                              { gpio.digitalRead(pin); }
void Drone::analogRead(int16_t pin)                               { gpio.analogRead(pin); }
void Drone::digitalReadCallback(void (*cb)(int16_t), int pin)     { gpio.digitalReadCallback(cb, pin); }
void Drone::pulseInCallback(void (*cb)(uint32_t), int16_t pin)    { gpio.pulseInCallback(cb, pin); }
void Drone::analogReadCallback(void (*cb)(int16_t), int16_t pin)  { gpio.analogReadCallback(cb, pin); }
void Drone::attachServo(int16_t servoNumber, int16_t pin)         { gpio.attachServo(servoNumber, pin); }
void Drone::detachServo(int16_t servoNumber)                      { gpio.detachServo(servoNumber); }
void Drone::writeServo(int16_t servoNumber, int16_t data)         { gpio.writeServo(servoNumber, data); }
void Drone::interruptCallback(void (*cb)(void), int16_t interrupt){ gpio.interruptCallback(cb, interrupt); }

// Sync Getters
uint32_t Drone::pulseInSync(int16_t pin, int16_t value)     { return gpio.pulseInSync(pin, value); }
uint32_t Drone::pulseInSync(int16_t pin, int16_t value, uint32_t timeout)     { return gpio.pulseInSync(pin, value, timeout); }
int16_t Drone::digitalReadSync(int16_t pin) { return gpio.digitalReadSync(pin); }
int16_t Drone::analogReadSync(int16_t pin)  { return gpio.analogReadSync(pin); }



//
// I2C Methods
//

void Drone::i2cSetDeviceAddress(int16_t id)      { i2c.setDeviceAddress(id); }
void Drone::i2cBeginTransmission()               { i2c.beginTransmission(); }
void Drone::i2cEndTransmission()                 { i2c.endTransmission(); }
void Drone::i2cWrite(int16_t data)               { i2c.write(data); }
void Drone::i2cRead()                            { i2c.read(); }
void Drone::i2cRequestFrom(int16_t byteCount)    { i2c.wireRequest(byteCount); }
void Drone::i2cReadCallback(void (*cb)(uint8_t)) { i2c.readCallback(cb); }

// Sync Getters
int16_t Drone::i2cReadSync() { return i2c.readSync(); }

//
// Vitals Methods
//

void Drone::heartbeatLostCallback(void (*cb)(void)) { vitals.heartbeatLostCallback(cb); }

void Drone::getVoltage() { vitals.getVoltage(); }
int16_t Drone::getVoltageSync() { return vitals.getVoltageSync(); }
void Drone::voltageCallback(void (*cb)(int16_t)) { vitals.voltageCallback(cb); }

void Drone::getSignalStrength() { vitals.getSignalStrength(); }
int16_t Drone::getSignalStrengthSync() { return vitals.getSignalStrengthSync(); }
void Drone::signalStrengthCallback(void (*cb)(int16_t)) { vitals.signalStrengthCallback(cb); }
void Drone::setErrorHandler(void (*cb)(int16_t)) { vitals.setErrorHandler(cb); }
