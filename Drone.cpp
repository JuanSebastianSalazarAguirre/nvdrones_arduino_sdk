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
  avantSPI = AvantSPI(&serialIO, &callback);
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
  avantSPI = AvantSPI(&serialIO, &callback);
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
  avantSPI = AvantSPI(&serialIO, &callback);
  avantAutoPilot = AvantAutoPilot(&serialIO, &callback);
}

AvantGPIO& Drone::GPIO() {return avantGPIO;} 
AvantResponseHandler& Drone::avantResponseHandler(){return responseHandler;}
AvantRC& Drone::RC() {return avantRC;} //functionality for sending RC data to the drone
AvantI2C& Drone::I2C() {return avantI2C;}
AvantPose& Drone::pose() {return avantPose;}
AvantSPI& Drone::SPI() {return avantSPI;}
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
  service->sendPacket((int16_t)0, 11, 8);
}

void AvantI2C::endTransmission(void){
  service->sendPacket((int16_t)0, 11, 4);
}

void AvantI2C::write(uint8_t data){
  service->sendPacket((uint8_t)data, 11, 3);
}

void AvantI2C::read(void){
  service->sendPacket((int16_t)0, 11, 5);
}
void AvantI2C::wireRequest(uint8_t bytes){
  service->sendPacket(bytes, 11, 6);
}

void AvantI2C::readCallback(void (*function)(byte)) {
  (*myCallback).i2cRead = function;
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
  service->sendPacket((int16_t)0, 9, 1);
}

void AvantPose::getLatitude(void) {
  service->sendPacket((int16_t)0, 9, 2);
}

void AvantPose::getLongitude(void) {
  service->sendPacket((int16_t)0, 9, 3);
}

void AvantPose::getAltitude(void ) {
  service->sendPacket((int16_t)0, 9, 4);
}

void AvantPose::getSatellites(void) {
  service->sendPacket((int16_t)0, 9, 5);
}

void AvantPose::getSpeed(void) {
  service->sendPacket((int16_t)0, 9, 6);
}

void AvantPose::getOrientation(void){
  service->sendPacket((int16_t)0, 9, 7);
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
  service->sendPacket((int16_t)0, 3, 1);
}

void AvantAutoPilot::compassExecute() {
  service->sendPacket((int16_t)0, 3, 2);
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
  service->sendPacket((int16_t)0, 15, 22);
}

void AvantAutoPilot::getWaypointLongitude(void (*function)(float)) {
  (*myCallback).getWaypointLongitude = function;
  service->sendPacket((int16_t)0, 15, 23);
}

void AvantAutoPilot::getWaypointAltitude(void (*function)(float)) {
  (*myCallback).getWaypointAltitude = function;
  service->sendPacket((int16_t)0, 15, 24);
}

void AvantAutoPilot::getWaypointOrientation(void (*function)(float)) {
  (*myCallback).getWaypointOrientation = function;
  service->sendPacket((int16_t)0, 15, 25);
}

void AvantAutoPilot::getYawKP(void (*function)(float)) {
  (*myCallback).getYawKP = function;
  service->sendPacket((int16_t)0, 15, 26);
}

void AvantAutoPilot::getYawKD(void (*function)(float)) {
  (*myCallback).getYawKD = function;
  service->sendPacket((int16_t)0, 15, 27);
}

void AvantAutoPilot::getYawKI(void (*function)(float)) {
  (*myCallback).getYawKI = function;
  service->sendPacket((int16_t)0, 15, 28);
}

void AvantAutoPilot::getThrottleKP(void (*function)(float)) {
  (*myCallback).getThrottleKP = function;
  service->sendPacket((int16_t)0, 15, 29);
}

void AvantAutoPilot::getThrottleKD(void (*function)(float)) {
  (*myCallback).getThrottleKD = function;
  service->sendPacket((int16_t)0, 15, 30);
}

void AvantAutoPilot::getThrottleKI(void (*function)(float)) {
  (*myCallback).getThrottleKI = function;
  service->sendPacket((int16_t)0, 15, 31);
}

void AvantAutoPilot::getElevatorKP(void (*function)(float)) {
  (*myCallback).getElevatorKP = function;
  service->sendPacket((int16_t)0, 15, 32);
}

void AvantAutoPilot::getElevatorKD(void (*function)(float)) {
  (*myCallback).getElevatorKD = function;
  service->sendPacket((int16_t)0, 15, 33);
}

void AvantAutoPilot::getElevatorKI(void (*function)(float)) {
  (*myCallback).getElevatorKI = function;
  service->sendPacket((int16_t)0, 15, 34);
}

void AvantAutoPilot::getAileronKP(void (*function)(float)) {
  (*myCallback).getAileronKP = function;
  service->sendPacket((int16_t)0, 15, 35);
}

void AvantAutoPilot::getAileronKD(void (*function)(float)) {
  (*myCallback).getAileronKD = function;
  service->sendPacket((int16_t)0, 15, 36);
}

void AvantAutoPilot::getAileronKI(void (*function)(float)) {
  (*myCallback).getAileronKI = function;
  service->sendPacket((int16_t)0, 15, 37);
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
