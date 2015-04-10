/*

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <Avant.h>
#include <avr/io.h>

// ***********************************************
// Avant Class Implementation
// ***********************************************
Avant::Avant() {
  serialIO = SerialIO(0);
  avantRC = AvantRC(&serialIO, &callback);
  callback = Callback();
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantTransmitter = AvantTransmitter(&serialIO);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantXbee = AvantXbee(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantSPI = AvantSPI(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}

Avant::Avant(int hardwareSerialCode) {
  serialIO = SerialIO(hardwareSerialCode);
  avantRC = AvantRC(&serialIO, &callback);
  callback = Callback();
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantTransmitter = AvantTransmitter(&serialIO);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantXbee = AvantXbee(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantSPI = AvantSPI(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}
Avant::Avant(int txPin, int rxPin) {
  serialIO = SerialIO(txPin, rxPin);
  avantRC = AvantRC(&serialIO, &callback);
  callback = Callback();
  avantGPIO = AvantGPIO(&serialIO, &callback);
  responseHandler = AvantResponseHandler(&serialIO, &callback);
  avantTransmitter = AvantTransmitter(&serialIO);
  avantI2C = AvantI2C(&serialIO, &callback);
  avantXbee = AvantXbee(&serialIO, &callback);
  avantPose = AvantPose(&serialIO, &callback);
  avantSPI = AvantSPI(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}

AvantGPIO& Avant::GPIO() {return avantGPIO;} 
AvantResponseHandler& Avant::avantResponseHandler(){return responseHandler;}
AvantTransmitter& Avant::transmitter() {return avantTransmitter;} //sets the analog pins that 
AvantRC& Avant::RC() {return avantRC;} //functionality for sending RC data to the drone
AvantI2C& Avant::I2C() {return avantI2C;}
AvantXbee& Avant::xbee() {return avantXbee;}
AvantPose& Avant::pose() {return avantPose;}
AvantSPI& Avant::SPI() {return avantSPI;}
AvantAutoPilot& Avant::AutoPilot() {return avantAutoPilot;}

void Avant::initialize() {
  serialIO.softwareSerial.begin(57600);
}

void Avant::armDrone() {
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



// ***********************************************
// SerialIO Class Implementation
// ***********************************************
SerialIO::SerialIO() {}

SerialIO::SerialIO(int txPin , int rxPin) {
  softwareSerial = SoftwareSerial(txPin, rxPin);
  softwareSerial.begin(57600);
  selectedSerialPort = swSerialPort;
}

SerialIO::SerialIO(int hwSerialCode) {
  if (hwSerialCode == 0) {
    #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
      Serial.begin(57600);
    #endif
	selectedSerialPort = serialPort0;
  } else if (hwSerialCode == 1) {
    #if defined(UBRR1H)
      Serial1.begin(57600);
    #endif
	selectedSerialPort = serialPort1;
  } else if (hwSerialCode == 2) {
    #if defined(UBRR2H)
      Serial2.begin(57600);
    #endif
	selectedSerialPort = serialPort2;
  } else if (hwSerialCode == 3) {
    #if defined(UBRR3H)
      Serial3.begin(57600);
    #endif
	selectedSerialPort = serialPort3;
  }
}

void SerialIO::write(uint8_t data) {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        Serial.write(data);
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        Serial1.write(data);
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        Serial2.write(data);
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        Serial3.write(data);
      #endif
      break;
    case swSerialPort:
      softwareSerial.write(data);
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}

uint8_t SerialIO::serialRead() {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        return Serial.read();
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        return Serial1.read();
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        return Serial2.read();
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        return Serial3.read();
      #endif
      break;
    case swSerialPort:
      return softwareSerial.read();
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}

bool SerialIO::serialAvailable() {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        return Serial.available();
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        return Serial1.available();
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        return Serial2.available();
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        return Serial3.available();
      #endif
      break;
    case swSerialPort:
      return softwareSerial.available();
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}

void SerialIO::sendPacket(uint8_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(1);
  write(resourceID);
  write(actionID);
  write(data);
  write((1+resourceID+actionID+data)%256);
}

void SerialIO::sendPacket(int8_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(1);
  write(byte(resourceID));
  write(byte(actionID));
  write(data);
  // Cast data because the firmware interprets packets as unsigned for calculating checksum.
  write((1+resourceID+actionID+(uint8_t)data)%256);
}

void SerialIO::sendPacket(int16_t data, uint8_t resourceID, uint8_t actionID) {
  write('$');
  write(2);
  write(byte(resourceID));
  write(byte(actionID));
  write(highByte(data));
  write(lowByte(data));
  write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
}

void SerialIO::sendPacket(float data, uint8_t resourceID, uint8_t actionID) {
  union u_tag {
    uint8_t b[4];
    float dataFloat;
  } u;
  u.dataFloat = data;

  write('$');
  write(4);
  write(byte(resourceID));
  write(byte(actionID));
  write(u.b[0]);
  write(u.b[1]);
  write(u.b[2]);
  write(u.b[3]);
  write((4+resourceID+actionID+u.b[0]+u.b[1]+u.b[2]+u.b[3])%256);
}

void SerialIO::print(String data) {
  switch(selectedSerialPort) {
    case serialPort0:
      #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
        Serial.print(data);
      #endif
      break;
    case serialPort1:
      #if defined(UBRR1H)
        Serial1.print(data);
      #endif
      break;
    case serialPort2:
      #if defined(UBRR2H)
        Serial2.print(data);
      #endif
      break;
    case serialPort3:
      #if defined(UBRR3H)
        Serial3.print(data);
      #endif
      break;
    case swSerialPort:
      softwareSerial.print(data);
      break;
    default:
      Serial.println("Error: incorrectly configured serial settings.");
  }
}

void SerialIO::readBytes(char *buffer, int bytesToRead) {
/*
  if (isHwSerial0Used) {
    #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
      Serial.readBytes(buffer, bytesToRead);
    #endif
  } else if (isHwSerial1Used) {
    #if defined(UBRR1H)
      Serial1.readBytes(buffer, bytesToRead);
    #endif
  } else if (isHwSerial2Used) {
    #if defined(UBRR2H)
      Serial2.readBytes(buffer, bytesToRead);
    #endif
  } else if (isHwSerial3Used) {
    #if defined(UBRR3H)
        Serial3.readBytes(buffer, bytesToRead);
    #endif
  } else if (isSwSerialUsed) {
    softwareSerial.readBytes(buffer, bytesToRead);
  }
*/
}
// ***********************************************
// AvantRC Class Implementation
// ***********************************************
AvantRC::AvantRC() {};
AvantRC::AvantRC(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantRC::setAileron(int8_t value){
  service->sendPacket(value, 2, 4);
};
void AvantRC::setElevator(int8_t value){
  service->sendPacket(value, 2, 3);
};
void AvantRC::setThrottle(int8_t value){
  service->sendPacket(value, 2, 2);
};
void AvantRC::setRudder(int8_t value){
  service->sendPacket(value, 2, 1);
}; 
void AvantRC::setFlightMode(int8_t value){
  service->sendPacket(value, 2, 5);
};
void AvantRC::getAileron(){
  service->sendPacket(0, 2, 9);
}
void AvantRC::getElevator(){
  service->sendPacket(0, 2, 8);
}
void AvantRC::getThrottle(){
  service->sendPacket(0, 2, 7);
}
void AvantRC::getRudder(){
  service->sendPacket(0, 2, 6);
}
void AvantRC::getFlightMode(){
  service->sendPacket(0, 2, 10);
}

void AvantRC::flightModeCallback(void (*function)(byte)) {
  (*myCallback).flightMode = function;
}

void AvantRC::elevatorCallback(void (*function)(byte)) {
  (*myCallback).elevator= function;
}

void AvantRC::aileronCallback(void (*function)(byte)) {
  (*myCallback).aileron = function;
}

void AvantRC::rudderCallback(void (*function)(byte)) {
  (*myCallback).rudder = function;
}

void AvantRC::throttleCallback(void (*function)(byte)) {
  (*myCallback).throttle = function;
}

void AvantRC::sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron){
  long data = (long(rudder+100))+ (long(throttle+100) << 8) + (long(elevator+100) << 16) + (long(aileron+100) << 24);
}


// ***********************************************
// AvantTransmitter Class Implementation
// ***********************************************
AvantTransmitter::AvantTransmitter(){};
AvantTransmitter::AvantTransmitter(SerialIO *serialIO) {
  service = serialIO;
  elevatorMax = 777;
  elevatorMin = 136;
  aileronMax = 872;
  aileronMin = 124;
  throttleMax = 780;
  throttleMin = 118;
  rudderMax = 867;
  rudderMin = 97;
}

void AvantTransmitter::setElevatorPin(int pin) {
  elevatorPin = pin;
}
int AvantTransmitter::getElevatorPin(){
  return elevatorPin;
}
void AvantTransmitter::setAileronPin(int pin) {
  AileronPin = pin;
}
int AvantTransmitter::getAileronPin(){
  return AileronPin;
}
void AvantTransmitter::setThrottlePin(int pin) {
  throttlePin = pin;
}
int AvantTransmitter::getThrottlePin(){
  return throttlePin;
}
void AvantTransmitter::setRudderPin(int pin) {
  rudderPin = pin;
}
int AvantTransmitter::getRudderPin(){
  return rudderPin;
}
void AvantTransmitter::setFlightModePin(int pin) {
  flightModePin = pin;
}
int AvantTransmitter::getFlightModePin() {
  return flightModePin;
}

void AvantTransmitter::sendSticks(){
  int Elevator = analogRead(elevatorPin);
  int Aileron = analogRead(AileronPin);
  int Throttle = analogRead(throttlePin);
  int Rudder = analogRead(rudderPin);
  Elevator = map(Elevator, elevatorMax, elevatorMin, -100, 100);
  Aileron = map(Aileron, aileronMax, aileronMin, -100, 100);
  Throttle = map(Throttle, throttleMax, throttleMin, -100, 100);
  Rudder = map(Rudder, rudderMax, rudderMin, -100, 100);
  if (Elevator > 100) Elevator = 100;
  if (Elevator < -100) Elevator = -100;
  if (Aileron > 100) Aileron = 100;
  if (Aileron < -100) Aileron = -100;        
  if (Throttle > 100) Throttle = 100;
  if (Throttle < -100) Throttle = -100;  
  if (Rudder > 100) Rudder = 100;
  if (Rudder < -100) Rudder = -100;    

  service->sendPacket(Elevator, 2, 3);
  service->sendPacket(Aileron, 2, 4);
  service->sendPacket(Throttle, 2, 2);
  service->sendPacket(Rudder, 2, 1);
}

void AvantTransmitter::throttleEndpoints(uint8_t min, uint8_t max) {
  throttleMax = max;
  throttleMin = min;
}
void AvantTransmitter::rudderEndpoints(uint8_t min, uint8_t max) {
  rudderMax = max;
  rudderMin = min;
}
void AvantTransmitter::aileronEndpoints(uint8_t min, uint8_t max) {
  aileronMax = max;
  aileronMin = min;
}
void AvantTransmitter::elevatorEndpoints(uint8_t min, uint8_t max) {
  elevatorMax = max;
  elevatorMin = min;
}
//************************************************
//AvantGPIO Class Implementation
//************************************************
AvantGPIO::AvantGPIO() {};

AvantGPIO::AvantGPIO(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantGPIO::digitalWrite(uint8_t pin, bool logicLevel) {
  service->sendPacket(logicLevel, 6, pin);
}

void AvantGPIO::pinMode(uint8_t pin, int logicLevel) {
  service->sendPacket(logicLevel, 5, pin);
}

void AvantGPIO::digitalRead(uint8_t pin) {
  service->sendPacket(0, 8, pin);
}

void AvantGPIO::analogWrite(uint8_t pin, uint8_t value) {
  service->sendPacket(value, 7, pin);
}

void AvantGPIO::pulseIn(uint8_t pin) {
  service->sendPacket(0, 16, pin);
}

void AvantGPIO::analogRead(uint8_t pin) {
  service->sendPacket(0, 17, pin);
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
  service->sendPacket(0, 18, actionID);
}

void AvantGPIO::writeServo(uint8_t servoNumber, uint8_t data) {
  uint8_t actionID = ((servoNumber - 1) * 3) + 2;
  service->sendPacket(data, 18, actionID);
}

//**********************************
//AvantI2C Class Implementation
//**********************************
AvantI2C::AvantI2C(){}

AvantI2C::AvantI2C(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantI2C::deviceID(uint8_t ID){
  service->sendPacket(ID, 11, 7);
}
void AvantI2C::beginTransmission(void){
  service->sendPacket(0, 11, 8);
}

void AvantI2C::endTransmission(void){
  service->sendPacket(0, 11, 4);
}

void AvantI2C::write(uint8_t data){
  service->sendPacket(data, 11, 3);
}

void AvantI2C::read(void){
  service->sendPacket(0, 11, 5);
}
void AvantI2C::wireRequest(uint8_t bytes){
  service->sendPacket(bytes, 11, 6);
}

void AvantI2C::readCallback(void (*function)(byte)) {
  (*myCallback).i2cRead = function;
}


//********************************************
//AvantXbee Class Implementation
//********************************************
AvantXbee::AvantXbee(){}

AvantXbee::AvantXbee(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}
   
void AvantXbee::id(uint8_t id) {
  char acknowledge[2];
  service->print("+++");
  delay(1200);
  service->print("ATID");
  //service->write(id);
  service->write(15);
  service->readBytes(&acknowledge[0], 2);
  service->print("ATCN");
  Serial.println(acknowledge);
}

//********************************************
//AvantPose Class Implementation
//********************************************
AvantPose::AvantPose(){}

AvantPose::AvantPose(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantPose::getGPSData(void) {
  service->sendPacket(0, 9, 1);
}

void AvantPose::getLatitude(void) {
  service->sendPacket(0, 9, 2);
}

void AvantPose::getLongitude(void) {
  service->sendPacket(0, 9, 3);
}

void AvantPose::getAltitude(void ) {
  service->sendPacket(0, 9, 4);
}

void AvantPose::getSatellites(void) {
  service->sendPacket(0, 9, 5);
}

void AvantPose::getSpeed(void) {
  service->sendPacket(0, 9, 6);
}

void AvantPose::getOrientation(void){
  service->sendPacket(0, 9, 7);
}

void AvantPose::longitudeCallback(void (*function)(float)) {
  (*myCallback).longitude = function;
}

void AvantPose::latitudeCallback(void (*function)(float)) {
  (*myCallback).latitude = function;
}

void AvantPose::altitudeCallback(void (*function)(float)) {
  (*myCallback).altitude = function;
}

void AvantPose::speedCallback(void (*function)(float)) {
  (*myCallback).speed = function;
}

void AvantPose::satelliteCallback(void (*function)(byte)) {
  (*myCallback).satellite = function;
}

void AvantPose::orientationCallback(void (*function)(float)) {
  (*myCallback).orientation = function;
}


//*******************************************
//AvantSPI Class Implementation
//*******************************************
AvantSPI::AvantSPI(){}
AvantSPI::AvantSPI(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantSPI::transfer(uint8_t data){
  service->sendPacket(data, 10, 1);
}

void AvantSPI::setBitOrder(uint8_t data){
  service->sendPacket(data, 10, 2);
}

void AvantSPI::setClockDivider(uint8_t data){
  service->sendPacket(data, 10, 3);
}

void AvantSPI::setDataMode(uint8_t data){
  service->sendPacket(data, 10, 4);
}

void AvantSPI::transferCallback(void (*function)(byte)) {
  (*myCallback).transfer = function;
}

//*******************************************
//AvantAutoPilot Class Implementation
//*******************************************
AvantAutoPilot::AvantAutoPilot(){}
AvantAutoPilot::AvantAutoPilot(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

void AvantAutoPilot::gpsExecute() {
  service->sendPacket(0, 3, 1);
}

void AvantAutoPilot::compassExecute() {
  service->sendPacket(0, 3, 2);
}

void AvantAutoPilot::setYawError(float error) {
  service->sendPacket(error, 3, 3);
}

void AvantAutoPilot::setThrottleError(float error) {
  service->sendPacket(error, 3, 4);
}

void AvantAutoPilot::setElevatorError(float error) {
  service->sendPacket(error, 3, 5);
}

void AvantAutoPilot::setAileronError(float error) {
  service->sendPacket(error, 3, 6);
}

void AvantAutoPilot::setWaypointLatitude(float latitude) {
  service->sendPacket(latitude, 15, 1);
}

void AvantAutoPilot::setWaypointLongitude(float longitude) {
  service->sendPacket(longitude, 15, 2);
}

void AvantAutoPilot::setWaypointAltitude(float altitude) {
  service->sendPacket(altitude, 15, 3);
}

void AvantAutoPilot::setWaypointOrientation(float orientation) {
  service->sendPacket(orientation, 15, 4);
}

void AvantAutoPilot::setYawKP(float kp) {
  service->sendPacket(kp, 15, 5);
}

void AvantAutoPilot::setYawKD(float kd) {
  service->sendPacket(kd, 15, 6);
}

void AvantAutoPilot::setYawKI(float ki) {
  service->sendPacket(ki, 15, 7);
}

void AvantAutoPilot::setThrottleKP(float kp) {
  service->sendPacket(kp, 15, 8);
}

void AvantAutoPilot::setThrottleKD(float kd) {
  service->sendPacket(kd, 15, 9);
}

void AvantAutoPilot::setThrottleKI(float ki) {
  service->sendPacket(ki, 15, 10);
}

void AvantAutoPilot::setElevatorKP(float kp) {
  service->sendPacket(kp, 15, 11);
}

void AvantAutoPilot::setElevatorKD(float kd) {
  service->sendPacket(kd, 15, 12);
}

void AvantAutoPilot::setElevatorKI(float ki) {
  service->sendPacket(ki, 15, 13);
}

void AvantAutoPilot::setAileronKP(float kp) {
  service->sendPacket(kp, 15, 14);
}

void AvantAutoPilot::setAileronKD(float kd) {
  service->sendPacket(kd, 15, 15);
}

void AvantAutoPilot::setAileronKI(float ki) {
  service->sendPacket(ki, 15, 16);
}

void AvantAutoPilot::getWaypointLatitude(void (*function)(float)) {
  (*myCallback).getWaypointLatitude = function;
  service->sendPacket(0, 15, 22);
}

void AvantAutoPilot::getWaypointLongitude(void (*function)(float)) {
  (*myCallback).getWaypointLongitude = function;
  service->sendPacket(0, 15, 23);
}

void AvantAutoPilot::getWaypointAltitude(void (*function)(float)) {
  (*myCallback).getWaypointAltitude = function;
  service->sendPacket(0, 15, 24);
}

void AvantAutoPilot::getWaypointOrientation(void (*function)(float)) {
  (*myCallback).getWaypointOrientation = function;
  service->sendPacket(0, 15, 25);
}

void AvantAutoPilot::getYawKP(void (*function)(float)) {
  (*myCallback).getYawKP = function;
  service->sendPacket(0, 15, 26);
}

void AvantAutoPilot::getYawKD(void (*function)(float)) {
  (*myCallback).getYawKD = function;
  service->sendPacket(0, 15, 27);
}

void AvantAutoPilot::getYawKI(void (*function)(float)) {
  (*myCallback).getYawKI = function;
  service->sendPacket(0, 15, 28);
}

void AvantAutoPilot::getThrottleKP(void (*function)(float)) {
  (*myCallback).getThrottleKP = function;
  service->sendPacket(0, 15, 29);
}

void AvantAutoPilot::getThrottleKD(void (*function)(float)) {
  (*myCallback).getThrottleKD = function;
  service->sendPacket(0, 15, 30);
}

void AvantAutoPilot::getThrottleKI(void (*function)(float)) {
  (*myCallback).getThrottleKI = function;
  service->sendPacket(0, 15, 31);
}

void AvantAutoPilot::getElevatorKP(void (*function)(float)) {
  (*myCallback).getElevatorKP = function;
  service->sendPacket(0, 15, 32);
}

void AvantAutoPilot::getElevatorKD(void (*function)(float)) {
  (*myCallback).getElevatorKD = function;
  service->sendPacket(0, 15, 33);
}

void AvantAutoPilot::getElevatorKI(void (*function)(float)) {
  (*myCallback).getElevatorKI = function;
  service->sendPacket(0, 15, 34);
}

void AvantAutoPilot::getAileronKP(void (*function)(float)) {
  (*myCallback).getAileronKP = function;
  service->sendPacket(0, 15, 35);
}

void AvantAutoPilot::getAileronKD(void (*function)(float)) {
  (*myCallback).getAileronKD = function;
  service->sendPacket(0, 15, 36);
}

void AvantAutoPilot::getAileronKI(void (*function)(float)) {
  (*myCallback).getAileronKI = function;
  service->sendPacket(0, 15, 37);
}

//*******************************************
//AvantResponseHandler Class Implementation
//*******************************************
AvantResponseHandler::AvantResponseHandler(){};
AvantResponseHandler::AvantResponseHandler(SerialIO *rcTservice, Callback *callback) {
  service = rcTservice;
  myCallback = callback;
}

float AvantResponseHandler::dataToFloat(byte data[]) {
  union u_tag {
    byte b[4];
    float data_float;
  } u;
  u.b[0] = byte(data[0]);
  u.b[1] = byte(data[1]);
  u.b[2] = byte(data[2]);
  u.b[3] = byte(data[3]);
  return u.data_float;
}

long AvantResponseHandler::dataToLong(byte data[]) {
  long data_long = 0;
  data_long = data[0] << 8;
  data_long = (data_long + data[1]) << 8;
  data_long = (data_long + data[2]) << 8;
  data_long = data_long + data[3]; 
  return data_long;
}

void AvantResponseHandler::responseHandler() {
  uint8_t buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
  uint8_t length = 0;
  uint8_t resourceID = 0;
  uint8_t actionID = 0;
  uint16_t datasum = 0;
  uint8_t i = 0;
  while (service->serialAvailable() > 0) {
    if (service->serialRead() == '$') {  //this is the start of a Data Packet
      while((actionID == -1 || resourceID == -1 || length == -1 ) && (i < 250)) {
        uint8_t s = service->serialRead();
        if (length == -1 && s != -1) {
          length = s;         //set the length
        } else if (resourceID == -1 && s != -1) {
          resourceID = s;     //set the resourceID
        } else if (actionID == -1 && s != -1) {
          actionID = s;       //set the actionID
        }
        i++;
      }
      uint8_t data[length+1]; // create a buffer to store the data and checksum info 
      i = 0;    //reset the i to zero to begin a new iteration process
      // TODO: explain `80` better
      for (int iterate = 0; iterate < 80*length; iterate++) { //iterate for certain amount of times based on number of desired bytes
        if (i == length+1) {               //if all desired bytes have been read then break
          break;
        }
        uint8_t s = service->serialRead();
        if (s != -1) {
          data[i] = s;
          i++;
        }
      }
      for(int i=0;i<length;i++){
        datasum = datasum+data[i];
      }
      
      if (data[length] == (length+resourceID+actionID+datasum)%256) {  //check the checksum
        //route the data to the appropriate place here
        switch(resourceID) {
          case 1:
            //Flight_Setup
            Serial.println("got Flight Setup");
            break;
          case 2:
            //Manual_Controls
            break;
          case 3:
            //Mission_Planner
            break;
          case 4:
            //Battery_Status
            break;
          case 5:
            //Pin_Mode
            break;
          case 6:
            //Digital_Write
            break;
          case 7:
            //Analog_Write
            break;
          case 8:
            //Digital_Read
            if(actionID == 1)
              myCallback->digitalRead1(data[0]);
            if(actionID == 2)
              myCallback->digitalRead2(data[0]);
            if(actionID == 3)
              myCallback->digitalRead3(data[0]);
            if(actionID == 4)
              myCallback->digitalRead4(data[0]);
            if(actionID == 5)
              myCallback->digitalRead5(data[0]);
            if(actionID == 6)
              myCallback->digitalRead6(data[0]);
            if(actionID == 7)
              myCallback->digitalRead7(data[0]);
            if(actionID == 8)
              myCallback->digitalRead8(data[0]);
            if(actionID == 9)
              myCallback->digitalRead9(data[0]);
            if(actionID == 10)
              myCallback->digitalRead10(data[0]);
            break;
          case 9:
            //Pose_Data
            if(actionID == 2)
              myCallback->longitude(dataToFloat(data));
            if(actionID == 3)
              myCallback->latitude(dataToFloat(data));
            if(actionID == 4)
              myCallback->altitude(dataToFloat(data));
            if(actionID == 5)
              myCallback->satellite(data[0]);
            if(actionID == 6)
              myCallback->speed(dataToFloat(data));
            if(actionID == 7)
              myCallback->orientation(dataToFloat(data));
            break;
          case 10:
            //SPI_Communication
            break;
          case 11:
            //I2C_Communication
            break;
          case 12:
            //Uart_Communication
            break;
          case 13:
            //Transmission_And_Security
            break;
          case 14:
            //Scripting
            break;
          case 15:
            //SimpleAP
            break;
          case 16:
            if(actionID == 1)
              myCallback->pulseIn1(dataToLong(data));
            if(actionID == 2)
              myCallback->pulseIn2(dataToLong(data));
            if(actionID == 3)
              myCallback->pulseIn3(dataToLong(data));
            if(actionID == 4)
              myCallback->pulseIn4(dataToLong(data));
            if(actionID == 5)
              myCallback->pulseIn5(dataToLong(data));
            if(actionID == 6)
              myCallback->pulseIn6(dataToLong(data));
            if(actionID == 7)
              myCallback->pulseIn7(dataToLong(data));
            if(actionID == 8)
              myCallback->pulseIn8(dataToLong(data));
            if(actionID == 9)
              myCallback->pulseIn9(dataToLong(data));
            if(actionID == 10)
              myCallback->pulseIn10(dataToLong(data));
            break;
          case 17:
            if(actionID == 1)
              myCallback->analogRead1(data[0]);
            if(actionID == 2)
              myCallback->analogRead2(data[0]);
            if(actionID == 3)
              myCallback->analogRead3(data[0]);
            if(actionID == 4)
              myCallback->analogRead4(data[0]);
            break;
      }//end of data packet router
    }//end of checksum
    else
      Serial.println("checksum failed");
    }//end of if Serial available
  }//end of Serial.read
}

//*****************************************************
//SoftwareSerial Code
//
// Lookup table
//
//\cond
typedef struct _DELAY_TABLE
//\endcond
{
  long baud;
  unsigned short rx_delay_centering;
  unsigned short rx_delay_intrabit;
  unsigned short rx_delay_stopbit;
  unsigned short tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
  //  baud    rxcenter   rxintra    rxstop    tx
  { 115200,   1,         17,        17,       12,    },
  { 57600,    10,        37,        37,       33,    },
  { 38400,    25,        57,        57,       54,    },
  { 31250,    31,        70,        70,       68,    },
  { 28800,    34,        77,        77,       74,    },
  { 19200,    54,        117,       117,      114,   },
  { 14400,    74,        156,       156,      153,   },
  { 9600,     114,       236,       236,      233,   },
  { 4800,     233,       474,       474,      471,   },
  { 2400,     471,       950,       950,      947,   },
  { 1200,     947,       1902,      1902,     1899,  },
  { 600,      1902,      3804,      3804,     3800,  },
  { 300,      3804,      7617,      7617,     7614,  },
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         5,      3,      },
  { 57600,    1,          15,        15,     13,     },
  { 38400,    2,          25,        26,     23,     },
  { 31250,    7,          32,        33,     29,     },
  { 28800,    11,         35,        35,     32,     },
  { 19200,    20,         55,        55,     52,     },
  { 14400,    30,         75,        75,     72,     },
  { 9600,     50,         114,       114,    112,    },
  { 4800,     110,        233,       233,    230,    },
  { 2400,     229,        472,       472,    469,    },
  { 1200,     467,        948,       948,    945,    },
  { 600,      948,        1895,      1895,   1890,   },
  { 300,      1895,       3805,      3805,   3802,   },
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        21,     18,     },
  { 57600,    20,         43,        43,     41,     },
  { 38400,    37,         73,        73,     70,     },
  { 31250,    45,         89,        89,     88,     },
  { 28800,    46,         98,        98,     95,     },
  { 19200,    71,         148,       148,    145,    },
  { 14400,    96,         197,       197,    194,    },
  { 9600,     146,        297,       297,    294,    },
  { 4800,     296,        595,       595,    592,    },
  { 2400,     592,        1189,      1189,   1186,   },
  { 1200,     1187,       2379,      2379,   2376,   },
  { 600,      2379,       4759,      4759,   4755,   },
  { 300,      4759,       9523,      9523,   9520,   },
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of SoftwareSerial supports only 20, 16 and 8MHz processors

#endif

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count) {
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--) {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) { 
  uint8_t tmp=0;

  asm volatile("sbiw    %0, 0x01 \n\t"
    "ldi %1, 0xFF \n\t"
    "cpi %A0, 0xFF \n\t"
    "cpc %B0, %1 \n\t"
    "brne .-10 \n\t"
    : "+r" (delay), "+a" (tmp)
    : "0" (delay)
    );
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen() {
  if (active_object != this) {
    _buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    SREG = oldSREG;
    return true;
  }

  return false;
}

//
// The receive routine called by the interrupt handler
//
void SoftwareSerial::recv() {

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read()) {
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1) {
      tunedDelay(_rx_delay_intrabit);
      DebugPulse(_DEBUG_PIN2, 1);
      uint8_t noti = ~i;
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN2, 1);

    if (_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } else {
    #if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
    #endif
      _buffer_overflow = true;
    }
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

void SoftwareSerial::tx_pin_write(uint8_t pin_state) {
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}

uint8_t SoftwareSerial::rx_pin_read() {
  return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt() {
  if (active_object) {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect) {
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect) {
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect) {
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect) {
  SoftwareSerial::handle_interrupt();
}
#endif

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic)
{
  setTX(transmitPin);
  setRX(receivePin);
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial() {
  end();
}

void SoftwareSerial::setTX(uint8_t tx) {
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(uint8_t rx) {
  pinMode(rx, INPUT);
  if (!_inverse_logic)
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

//
// Public methods
//

void SoftwareSerial::begin(long speed) {
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i) {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed) {
      _rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
      _rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
      _rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
      _tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }

  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (_rx_delay_stopbit) {
    if (digitalPinToPCICR(_receivePin)) {
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
      *digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
    }
    tunedDelay(_tx_delay); // if we were low this establishes the end
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  listen();
}

void SoftwareSerial::end() {
  if (digitalPinToPCMSK(_receivePin))
    *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
}


// Read data from buffer
int SoftwareSerial::read() {
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available() {
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b) {
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  uint8_t oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  tx_pin_write(_inverse_logic ? HIGH : LOW);
  tunedDelay(_tx_delay + XMIT_START_ADJUSTMENT);

  // Write each of the 8 bits
  if (_inverse_logic) {
    for (byte mask = 0x01; mask; mask <<= 1) {
      if (b & mask) // choose bit
        tx_pin_write(LOW); // send 1
      else
        tx_pin_write(HIGH); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(LOW); // restore pin to natural state
  } else {
    for (byte mask = 0x01; mask; mask <<= 1) {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(HIGH); // restore pin to natural state
  }

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(_tx_delay);
  
  return 1;
}

void SoftwareSerial::flush() {
  if (!isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  SREG = oldSREG;
}

int SoftwareSerial::peek() {
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
