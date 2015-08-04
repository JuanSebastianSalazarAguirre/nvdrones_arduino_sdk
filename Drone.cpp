#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "Drone.h"

#include <avr/io.h>


//
// Constructors
//

Drone::Drone() {
  _serialIO = SerialIO(serialPort0);
  _callback = Callback();
  _incomingPacketReader = IncomingPacketReader(&_serialIO);
  _vitals = Vitals(&_serialIO, &_incomingPacketReader, &_callback);
  _responseHandler = ResponseHandler(&_serialIO, &_incomingPacketReader, &_callback, &_vitals);
  _rc = RC(&_serialIO, &_callback, &_incomingPacketReader);
  _gpio = GPIO(&_serialIO, &_callback, &_incomingPacketReader);
  _i2c = I2C(&_serialIO, &_callback, &_incomingPacketReader);
  _pose = Pose(&_serialIO, &_callback, &_incomingPacketReader);
  _autopilot = Autopilot(&_serialIO, &_callback, &_incomingPacketReader);
  _transmitterSupport = TransmitterSupport(&_serialIO, &_callback, &_incomingPacketReader);
}

Drone::Drone(SerialPort serialPort) {
  _serialIO = SerialIO(serialPort);
  _callback = Callback();
  _incomingPacketReader = IncomingPacketReader(&_serialIO);
  _vitals = Vitals(&_serialIO, &_incomingPacketReader, &_callback);
  _responseHandler = ResponseHandler(&_serialIO, &_incomingPacketReader, &_callback, &_vitals);
  _rc = RC(&_serialIO, &_callback, &_incomingPacketReader);
  _gpio = GPIO(&_serialIO, &_callback, &_incomingPacketReader);
  _i2c = I2C(&_serialIO, &_callback, &_incomingPacketReader);
  _pose = Pose(&_serialIO, &_callback, &_incomingPacketReader);
  _autopilot = Autopilot(&_serialIO, &_callback, &_incomingPacketReader);
  _transmitterSupport = TransmitterSupport(&_serialIO, &_callback, &_incomingPacketReader);
}

Drone::Drone(int txPin, int rxPin) {
  _serialIO = SerialIO(txPin, rxPin);
  _callback = Callback();
  _incomingPacketReader = IncomingPacketReader(&_serialIO);
  _vitals = Vitals(&_serialIO, &_incomingPacketReader, &_callback);
  _responseHandler = ResponseHandler(&_serialIO, &_incomingPacketReader, &_callback, &_vitals);
  _rc = RC(&_serialIO, &_callback, &_incomingPacketReader);
  _gpio = GPIO(&_serialIO, &_callback, &_incomingPacketReader);
  _i2c = I2C(&_serialIO, &_callback, &_incomingPacketReader);
  _pose = Pose(&_serialIO, &_callback, &_incomingPacketReader);
  _autopilot = Autopilot(&_serialIO, &_callback, &_incomingPacketReader);
  _transmitterSupport = TransmitterSupport(&_serialIO, &_callback, &_incomingPacketReader);
}

//
// Initialization methods
//

void Drone::initialize() {
  _serialIO.softwareSerial.begin(57600);

  uint32_t t = millis();
  uint32_t now = 0;
  while (1) {
    IncomingPacket p = _incomingPacketReader.read();
    if (p.isHearbeat()) {
      _vitals.receiveHeartbeat();
      break;
    }

    if ((now = millis()) > 500 + t) {
      Serial.println("Waiting to initialize. Unable to contact drone.");
      t = now;
    }
  }

  Serial.println("Drone initialized.");
}

void Drone::arm() {
  _rc.setRudder(-100);
  _rc.setThrottle(-100);
  _rc.setElevator(-100);
  _rc.setAileron(-100);
  delay(500);
  _rc.setRudder(0);
  _rc.setThrottle(-100);
  _rc.setElevator(0);
  _rc.setAileron(0);
}


void Drone::listen() {
  _responseHandler.listen();

  _vitals.tick();
}


// 
// Pose Methods
//

// Async Getters
void Drone::getGPSData()                           { _pose.getGPSData(); }
void Drone::getLongitude()                         { _pose.getLongitude(); }
void Drone::getLatitude()                          { _pose.getLatitude(); }
void Drone::getAltitude()                          { _pose.getAltitude(); }
void Drone::getSatellites()                        { _pose.getSatellites(); }
void Drone::getSpeed()                             { _pose.getSpeed(); }
void Drone::getYaw()                               { _pose.getYaw(); }
void Drone::getPitchAngle()                        { _pose.getPitchAngle(); }
void Drone::getRollAngle()                         { _pose.getRollAngle(); }
void Drone::getAltitudeKp()                        { _autopilot.getAltitudeKp();}
void Drone::getAltitudeKi()                        { _autopilot.getAltitudeKi();}
void Drone::getAltitudeBase()                      { _autopilot.getAltitudeBase();}
void Drone::getAltitudeReference()                 { _autopilot.getAltitudeReference();}
void Drone::getAltitudeTolerance()                 { _autopilot.getAltitudeTolerance();}
void Drone::getXPositionKp()                       { _autopilot.getXPositionKp();}
void Drone::getXPositionKi()                       { _autopilot.getXPositionKi();}
void Drone::getXPositionKd()                       { _autopilot.getXPositionKd();}
void Drone::getXPositionReference()                { _autopilot.getXPositionReference();}
void Drone::getXPositionTolerance()                { _autopilot.getXPositionTolerance();}
void Drone::getYPositionKp()                       { _autopilot.getYPositionKp();}
void Drone::getYPositionKi()                       { _autopilot.getYPositionKi();}
void Drone::getYPositionKd()                       { _autopilot.getYPositionKd();}
void Drone::getYPositionReference()                { _autopilot.getYPositionReference();}
void Drone::getYPositionTolerance()                { _autopilot.getYPositionTolerance();}
void Drone::getSonarAltitude()                     { _autopilot.getSonarAltitude();}
void Drone::getSonarXPosition()                    { _autopilot.getSonarXPosition();}    
void Drone::getSonarYPosition()                    { _autopilot.getSonarYPosition();}

void Drone::longitudeCallback(void (*cb)(float))   { _pose.longitudeCallback(cb); }
void Drone::latitudeCallback(void (*cb)(float))    { _pose.latitudeCallback(cb); }
void Drone::altitudeCallback(void (*cb)(float))    { _pose.altitudeCallback(cb); }
void Drone::satelliteCallback(void (*cb)(int16_t)) { _pose.satelliteCallback(cb); }
void Drone::speedCallback(void (*cb)(float))       { _pose.speedCallback(cb); }
void Drone::yawCallback(void (*cb)(float))         { _pose.yawCallback(cb); }
void Drone::pitchAngleCallback(void (*cb)(float))  { _pose.pitchAngleCallback(cb); }
void Drone::rollAngleCallback(void (*cb)(float))   { _pose.rollAngleCallback(cb); }

// Sync Getters
float Drone::getLatitudeSync()      { return _pose.getLatitudeSync(); }
float Drone::getLongitudeSync()     { return _pose.getLongitudeSync(); }
float Drone::getAltitudeSync()      { return _pose.getAltitudeSync(); }
int16_t Drone::getSatellitesSync()  { return _pose.getSatellitesSync(); }
float Drone::getSpeedSync()         { return _pose.getSpeedSync(); }
float Drone::getYawSync()           { return _pose.getYawSync(); }
float Drone::getPitchAngleSync()    { return _pose.getPitchAngleSync(); }
float Drone::getRollAngleSync()     { return _pose.getRollAngleSync(); }


//
// TODO: rename RC
// RC Methods
//

// Setters
void Drone::setAileron(int16_t value)                 { _rc.setAileron(value); }
void Drone::setElevator(int16_t value)                { _rc.setElevator(value); }
void Drone::setThrottle(int16_t value)                { _rc.setThrottle(value); }
void Drone::setRudder(int16_t value)                  { _rc.setRudder(value); }
void Drone::setFlightMode(int16_t value)              { _rc.setFlightMode(value); }


// Async Getters
void Drone::getAileron()                             { _rc.getAileron(); }
void Drone::getElevator()                            { _rc.getElevator(); }
void Drone::getThrottle()                            { _rc.getThrottle(); }
void Drone::getRudder()                              { _rc.getRudder(); }
void Drone::getFlightMode()                          { _rc.getFlightMode(); }
void Drone::aileronCallback(void (*cb)(int16_t))     { _rc.aileronCallback(cb); }
void Drone::flightModeCallback(void (*cb)(int16_t))  { _rc.flightModeCallback(cb); }
void Drone::throttleCallback(void (*cb)(int16_t))    { _rc.throttleCallback(cb); }
void Drone::rudderCallback(void (*cb)(int16_t))      { _rc.rudderCallback(cb); }
void Drone::elevatorCallback(void (*cb)(int16_t))    { _rc.elevatorCallback(cb); }
void Drone::sonarAltitudeCallback(void (*cb)(float))        { _autopilot.sonarAltitudeCallback(cb); }
void Drone::sonarXPositionCallback(void (*cb)(float))       { _autopilot.sonarXPositionCallback(cb); }
void Drone::sonarYPositionCallback(void (*cb)(float))       { _autopilot.sonarYPositionCallback(cb); }
void Drone::AltitudeKpCallback(void (*cb)(int16_t))         { _autopilot.altitudeKpCallback(cb); }
void Drone::AltitudeKiCallback(void (*cb)(int16_t))         { _autopilot.altitudeKiCallback(cb); }
void Drone::AltitudeReferenceCallback(void (*cb)(float))    { _autopilot.altitudeReferenceCallback(cb); }
void Drone::AltitudeToleranceCallback(void (*cb)(float))    { _autopilot.altitudeToleranceCallback(cb); }
void Drone::AltitudeBaseCallback(void (*cb)(int16_t))       { _autopilot.altitudeBaseCallback(cb); }
void Drone::XPositionKpCallback(void (*cb)(int16_t))        { _autopilot.XPositionKpCallback(cb); }
void Drone::XPositionKiCallback(void (*cb)(int16_t))        { _autopilot.XPositionKiCallback(cb); }
void Drone::XPositionKdCallback(void (*cb)(int16_t))        { _autopilot.XPositionKdCallback(cb); }
void Drone::XPositionReferenceCallback(void (*cb)(float))   { _autopilot.XPositionReferenceCallback(cb); }
void Drone::XPositionToleranceCallback(void (*cb)(float))   { _autopilot.XPositionToleranceCallback(cb); } 
void Drone::YPositionKpCallback(void (*cb)(int16_t))        { _autopilot.YPositionKpCallback(cb); }
void Drone::YPositionKiCallback(void (*cb)(int16_t))        { _autopilot.YPositionKiCallback(cb); }
void Drone::YPositionKdCallback(void (*cb)(int16_t))        { _autopilot.YPositionKdCallback(cb); }
void Drone::YPositionReferenceCallback(void (*cb)(float))   { _autopilot.YPositionReferenceCallback(cb); }
void Drone::YPositionToleranceCallback(void (*cb)(float))   { _autopilot.YPositionToleranceCallback(cb); } 
void Drone::setAileronElevatorRudderThrottle(int16_t aileron,
  int16_t elevator, int16_t rudder, int16_t throttle) {
  _rc.setAileronElevatorRudderThrottle(aileron, elevator, rudder, throttle);
}

//
// Sync Getters
//
int16_t Drone::getAileronSync()                         { return _rc.getAileronSync(); }
int16_t Drone::getElevatorSync()                        { return _rc.getElevatorSync(); }
int16_t Drone::getThrottleSync()                        { return _rc.getThrottleSync(); }
int16_t Drone::getRudderSync()                          { return _rc.getRudderSync(); }
int16_t Drone::getFlightModeSync()                      { return _rc.getFlightModeSync(); }

int16_t Drone::getSonarAltitudeSync()                   { return _autopilot.getSonarAltitudeSync();}
int16_t Drone::getSonarPositionXSync()                  { return _autopilot.getSonarXPositionSync();}
int16_t Drone::getSonarPositionYSync()                  { return _autopilot.getSonarYPositionSync();}

int16_t Drone::getAltitudeKpSync()                      { return _autopilot.getAltitudeKpSync();}
int16_t Drone::getAltitudeKiSync()                      { return _autopilot.getAltitudeKiSync();}
int16_t Drone::getAltitudeBaseSync()                    { return _autopilot.getAltitudeBaseSync();}
float Drone::getAltitudeReferenceSync()                 { return _autopilot.getAltitudeReferenceSync();}
float Drone::getAltitudeToleranceSync()                 { return _autopilot.getAltitudeToleranceSync();}

int16_t Drone::getXPositionKpSync()                     { return _autopilot.getXPositionKpSync();}
int16_t Drone::getXPositionKiSync()                     { return _autopilot.getXPositionKiSync();}
int16_t Drone::getXPositionKdSync()                      { return _autopilot.getXPositionKdSync();}
float Drone::getXPositionReferenceSync()                { return _autopilot.getXPositionReferenceSync();}
float Drone::getXPositionToleranceSync()                { return _autopilot.getXPositionToleranceSync();}

int16_t Drone::getYPositionKpSync()                     { return _autopilot.getYPositionKpSync();}
int16_t Drone::getYPositionKiSync()                     { return _autopilot.getYPositionKiSync();}
int16_t Drone::getYPositionKdSync()                     { return _autopilot.getYPositionKdSync();}
float Drone::getYPositionReferenceSync()                { return _autopilot.getYPositionReferenceSync();}
float Drone::getYPositionToleranceSync()                { return _autopilot.getYPositionToleranceSync();}

//
// GPIO Methods
//

void Drone::pinMode(int16_t pin, int16_t logicLevel)              { _gpio.pinMode(pin, logicLevel); }
void Drone::digitalWrite(int16_t pin, bool logicLevel)            { _gpio.digitalWrite(pin, logicLevel); }
void Drone::analogWrite(int16_t pin, int16_t value)               { _gpio.analogWrite(pin, value); }
void Drone::pulseIn(int16_t pin, int16_t value)                   { _gpio.pulseIn(pin, value); }
void Drone::pulseIn(int16_t pin, int16_t value, uint32_t timeout) { _gpio.pulseIn(pin, value, timeout); }
void Drone::digitalRead(int16_t pin)                              { _gpio.digitalRead(pin); }
void Drone::analogRead(int16_t pin)                               { _gpio.analogRead(pin); }
void Drone::digitalReadCallback(void (*cb)(int16_t), int pin)     { _gpio.digitalReadCallback(cb, pin); }
void Drone::pulseInCallback(void (*cb)(uint32_t), int16_t pin)    { _gpio.pulseInCallback(cb, pin); }
void Drone::analogReadCallback(void (*cb)(int16_t), int16_t pin)  { _gpio.analogReadCallback(cb, pin); }
void Drone::attachServo(int16_t servoNumber, int16_t pin)         { _gpio.attachServo(servoNumber, pin); }
void Drone::detachServo(int16_t servoNumber)                      { _gpio.detachServo(servoNumber); }
void Drone::writeServo(int16_t servoNumber, int16_t data)         { _gpio.writeServo(servoNumber, data); }
void Drone::interruptCallback(void (*cb)(void), int16_t interrupt){ _gpio.interruptCallback(cb, interrupt); }


// Sync Getters
uint32_t Drone::pulseInSync(int16_t pin, int16_t value)     { return _gpio.pulseInSync(pin, value); }
uint32_t Drone::pulseInSync(int16_t pin, int16_t value, uint32_t timeout)     { return _gpio.pulseInSync(pin, value, timeout); }
int16_t Drone::digitalReadSync(int16_t pin) { return _gpio.digitalReadSync(pin); }
int16_t Drone::analogReadSync(int16_t pin)  { return _gpio.analogReadSync(pin); }



//
// I2C Methods
//

void Drone::i2cSetDeviceAddress(int16_t id)           { _i2c.setDeviceAddress(id); }
void Drone::i2cBeginTransmission()                    { _i2c.beginTransmission(); }
void Drone::i2cEndTransmission()                      { _i2c.endTransmission(); }
void Drone::i2cWrite(int16_t data)                    { _i2c.write(data); }
void Drone::i2cRead()                                 { _i2c.read(); }
void Drone::i2cRequestFrom(int16_t byteCount)         { _i2c.wireRequest(byteCount); }
void Drone::i2cReadCallback(void (*cb)(uint8_t))      { _i2c.readCallback(cb); }
void Drone::i2cAvailable()                            { _i2c.available(); }
void Drone::i2cAvailableCallback(void (*cb)(int16_t)) { _i2c.availableCallback(cb); }

// Sync Getters
int16_t Drone::i2cReadSync()      { return _i2c.readSync(); }
int16_t Drone::i2cAvailableSync() { return _i2c.availableSync(); }


//
// Vitals Methods
//

void Drone::heartbeatLostCallback(void (*cb)(void))       { _vitals.heartbeatLostCallback(cb); }
void Drone::heartbeatFoundCallback(void (*cb)(void))      { _vitals.heartbeatFoundCallback(cb); }
void Drone::getVoltage()                                  { _vitals.getVoltage(); }
int16_t Drone::getVoltageSync()                           { return _vitals.getVoltageSync(); }
void Drone::voltageCallback(void (*cb)(int16_t))          { _vitals.voltageCallback(cb); }
void Drone::getSignalStrength()                           { _vitals.getSignalStrength(); }
int16_t Drone::getSignalStrengthSync()                    { return _vitals.getSignalStrengthSync(); }
void Drone::signalStrengthCallback(void (*cb)(int16_t))   { _vitals.signalStrengthCallback(cb); }
void Drone::setErrorHandler(void (*cb)(int16_t))          { _vitals.setErrorHandler(cb); }


//
//Autopilot
//

void Drone::setAutopilotMode(int value)               {_autopilot.setAutopilotMode(value);}
void Drone::setAltitudeKp(int value)                  {_autopilot.setAltitudeKp(value);}
void Drone::setAltitudeKi(int value)                  {_autopilot.setAltitudeKi(value);}
void Drone::setAltitudeBase(int value)                {_autopilot.setAltitudeBase(value);}
void Drone::setAltitudeReference(float value)         {_autopilot.setAltitudeReference(value);}
void Drone::setAltitudeTolerance(float value)         {_autopilot.setAltitudeTolerance(value);}

void Drone::setXPositionKp(int value)                  {_autopilot.setXPositionKp(value);}
void Drone::setXPositionKi(int value)                  {_autopilot.setXPositionKi(value);}
void Drone::setXPositionKd(int value)                  {_autopilot.setXPositionKd(value);}
void Drone::setXPositionReference(float value)         {_autopilot.setXPositionReference(value);}
void Drone::setXPositionTolerance(float value)         {_autopilot.setXPositionTolerance(value);}

void Drone::setYPositionKp(int value)                  {_autopilot.setYPositionKp(value);}
void Drone::setYPositionKi(int value)                  {_autopilot.setYPositionKi(value);}
void Drone::setYPositionKd(int value)                  {_autopilot.setYPositionKd(value);}
void Drone::setYPositionReference(float value)         {_autopilot.setYPositionReference(value);}
void Drone::setYPositionTolerance(float value)         {_autopilot.setYPositionTolerance(value);}

//
//Transmitter Support Methods
//

void Drone::startTransmitterSupport()                    { _transmitterSupport.startTransmitterSupport(); }
void Drone::stopTransmitterSupport()                     { _transmitterSupport.stopTransmitterSupport(); }
void Drone::startTransmitterCalibration()                { _transmitterSupport.startTransmitterCalibration(); }
void Drone::stopTransmitterCalibration()                 { _transmitterSupport.stopTransmitterCalibration(); }

