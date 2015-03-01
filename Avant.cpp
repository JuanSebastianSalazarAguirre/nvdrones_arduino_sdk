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
    rcService = RCTransmitService(0);
    avantRC = AvantRC(&rcService, &callback);
	callback = Callback();
	avantGPIO = AvantGPIO(&rcService, &callback);
	responseHandler = AvantResponseHandler(&rcService, &callback);
    avantTransmitter = AvantTransmitter(&rcService);
	avantI2C = AvantI2C(&rcService, &callback);
	avantXbee = AvantXbee(&rcService, &callback);
	avantPose = AvantPose(&rcService, &callback);
	avantSPI = AvantSPI(&rcService, &callback);
}

Avant::Avant(int hardwareSerialCode) {
    rcService = RCTransmitService(hardwareSerialCode);
    avantRC = AvantRC(&rcService, &callback);
	callback = Callback();
	avantGPIO = AvantGPIO(&rcService, &callback);
	responseHandler = AvantResponseHandler(&rcService, &callback);
    avantTransmitter = AvantTransmitter(&rcService);
	avantI2C = AvantI2C(&rcService, &callback);
	avantXbee = AvantXbee(&rcService, &callback);
	avantPose = AvantPose(&rcService, &callback);
	avantSPI = AvantSPI(&rcService, &callback);
}
Avant::Avant(int txPin, int rxPin) {
   rcService = RCTransmitService(txPin, rxPin);
   avantRC = AvantRC(&rcService, &callback);
   callback = Callback();
   avantGPIO = AvantGPIO(&rcService, &callback);
   responseHandler = AvantResponseHandler(&rcService, &callback);
   avantTransmitter = AvantTransmitter(&rcService);
   avantI2C = AvantI2C(&rcService, &callback);
   avantXbee = AvantXbee(&rcService, &callback);
   avantPose = AvantPose(&rcService, &callback);
   avantSPI = AvantSPI(&rcService, &callback);
}

AvantGPIO& Avant::GPIO() {return avantGPIO;} 
AvantResponseHandler& Avant::avantResponseHandler(){return responseHandler;}
AvantTransmitter& Avant::transmitter() {return avantTransmitter;} //sets the analog pins that 
AvantRC& Avant::RC() {return avantRC;} //functionality for sending RC data to the drone
AvantI2C& Avant::I2C() {return avantI2C;}
AvantXbee& Avant::xbee() {return avantXbee;}
AvantPose& Avant::pose() {return avantPose;}
AvantSPI& Avant::SPI() {return avantSPI;}

        
void Avant::armDrone() {
    rcService.sendData(-100, 2, 1);
    rcService.sendData(-100, 2, 2);
    rcService.sendData(-100, 2, 3);
    rcService.sendData(-100, 2, 4);
    delay(500);
    rcService.sendData(0, 2, 1);
    rcService.sendData(-100, 2, 2);
    rcService.sendData(0, 2, 3);
    rcService.sendData(0, 2, 4);
}



// ***********************************************
// RCTransmitService Class Implementation
// ***********************************************
RCTransmitService::RCTransmitService() {}

RCTransmitService::RCTransmitService(int txPin , int rxPin) {
    softwareSerial = SoftwareSerial(txPin, rxPin);
	softwareSerial.begin(57600);
    isHwSerial0Used = false;
    isHwSerial1Used = false;
    isHwSerial2Used = false;
    isHwSerial3Used = false;
    isSwSerialUsed = true;
}

RCTransmitService::RCTransmitService(int hwSerialCode) {
    if (hwSerialCode == 0) {
        #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
            Serial.begin(57600);
        #endif
        isHwSerial0Used = true;
        isHwSerial1Used = false;
        isHwSerial2Used = false;
        isHwSerial3Used = false;
        isSwSerialUsed = false;
    } else if (hwSerialCode == 1) {
        #if defined(UBRR1H)
            Serial1.begin(57600);
        #endif
        isHwSerial0Used = false;
        isHwSerial1Used = true;
        isHwSerial2Used = false;
        isHwSerial3Used = false;
        isSwSerialUsed = false;
    } else if (hwSerialCode == 2) {
        #if defined(UBRR2H)
            Serial2.begin(57600);
        #endif
        isHwSerial0Used = false;
        isHwSerial1Used = false;
        isHwSerial2Used = true;
        isHwSerial3Used = false;
        isSwSerialUsed = false;
    }  else if (hwSerialCode == 3) {
        #if defined(UBRR3H)
            Serial3.begin(57600);
        #endif
        isHwSerial0Used = false;
        isHwSerial1Used = false;
        isHwSerial2Used = false;
        isHwSerial3Used = true;
        isSwSerialUsed = false;
    }
}

void RCTransmitService::sendData(int data, uint8_t resourceID, uint8_t actionID) {
    if (isHwSerial0Used) {
        #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
            Serial.write('$');
            Serial.write(2);
            Serial.write(byte(resourceID));
            Serial.write(byte(actionID));
            Serial.write(highByte(data));
            Serial.write(lowByte(data));
            Serial.write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
        #endif
    } else if (isHwSerial1Used) {
        #if defined(UBRR1H)
            Serial1.write('$');
            Serial1.write(2);
            Serial1.write(byte(resourceID));
            Serial1.write(byte(actionID));
            Serial1.write(highByte(data));
            Serial1.write(lowByte(data));
            Serial1.write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
        #endif
    }
    else if (isHwSerial2Used) {
        #if defined(UBRR2H)
            Serial2.write('$');
            Serial2.write(2);
            Serial2.write(byte(resourceID));
            Serial2.write(byte(actionID));
            Serial2.write(highByte(data));
            Serial2.write(lowByte(data));
            Serial2.write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
        #endif
    } else if (isHwSerial3Used) {
        #if defined(UBRR3H)
            Serial3.write('$');
            Serial3.write(2);
            Serial3.write(byte(resourceID));
            Serial3.write(byte(actionID));
            Serial3.write(highByte(data));
            Serial3.write(lowByte(data));
            Serial3.write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
        #endif
    } 
    else if (isSwSerialUsed) {
        softwareSerial.write('$');
        softwareSerial.write(2);
        softwareSerial.write(byte(resourceID));
        softwareSerial.write(byte(actionID));
        softwareSerial.write(highByte(data));
        softwareSerial.write(lowByte(data));
        softwareSerial.write((2+resourceID+actionID+highByte(data)+lowByte(data))%256);
    }
}

void RCTransmitService::print(String data) {
    if (isHwSerial0Used) {
        #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
            Serial.print(data);
        #endif
    } else if (isHwSerial1Used) {
        #if defined(UBRR1H)
            Serial1.print(data);
        #endif
    }
    else if (isHwSerial2Used) {
        #if defined(UBRR2H)
            Serial2.print(data);
        #endif
    } else if (isHwSerial3Used) {
        #if defined(UBRR3H)
            Serial3.print(data);
        #endif
    } 
    else if (isSwSerialUsed) {
        softwareSerial.print(data);
    }
}

void RCTransmitService::write(byte data) {
    if (isHwSerial0Used) {
        #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
            Serial.write(data);
        #endif
    } else if (isHwSerial1Used) {
        #if defined(UBRR1H)
            Serial1.write(data);
        #endif
    }
    else if (isHwSerial2Used) {
        #if defined(UBRR2H)
            Serial2.write(data);
        #endif
    } else if (isHwSerial3Used) {
        #if defined(UBRR3H)
            Serial3.write(data);
        #endif
    } 
    else if (isSwSerialUsed) {
        softwareSerial.write(data);
    }
}

void RCTransmitService::readBytes(char *buffer, int bytesToRead) {
    if (isHwSerial0Used) {
        #if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H)
            Serial.readBytes(buffer, bytesToRead);
        #endif
    } else if (isHwSerial1Used) {
        #if defined(UBRR1H)
            Serial1.readBytes(buffer, bytesToRead);
        #endif
    }
    else if (isHwSerial2Used) {
        #if defined(UBRR2H)
            Serial2.readBytes(buffer, bytesToRead);
        #endif
    } else if (isHwSerial3Used) {
        #if defined(UBRR3H)
            Serial3.readBytes(buffer, bytesToRead);
        #endif
    } 
    else if (isSwSerialUsed) {
        softwareSerial.readBytes(buffer, bytesToRead);
    }
}
// ***********************************************
// AvantRC Class Implementation
// ***********************************************
AvantRC::AvantRC() {};
AvantRC::AvantRC(RCTransmitService *rcTservice, Callback *callback) {
    service = rcTservice;
	myCallback = callback;
}

void AvantRC::setAileron(int value){
	service->sendData(value, 2, 4);
};
void AvantRC::setElevator(int value){
	service->sendData(value, 2, 3);
};
void AvantRC::setThrottle(int value){
	service->sendData(value, 2, 2);
};
void AvantRC::setRudder(int value){
	service->sendData(value, 2, 1);
}; 
void AvantRC::setFlightMode(int value){
	service->sendData(value, 2, 5);
};
void AvantRC::getAileron(){
	service->sendData(0, 2, 9);
}
void AvantRC::getElevator(){
	service->sendData(0, 2, 8);
}
void AvantRC::getThrottle(){
	service->sendData(0, 2, 7);
}
void AvantRC::getRudder(){
	service->sendData(0, 2, 6);
}
void AvantRC::getFlightMode(){
	service->sendData(0, 2, 10);
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
AvantTransmitter::AvantTransmitter(RCTransmitService *rcService) {
    service = rcService;
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

    service->sendData(Elevator, 2, 3);
    service->sendData(Aileron, 2, 4);
    service->sendData(Throttle, 2, 2);
    service->sendData(Rudder, 2, 1);
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

AvantGPIO::AvantGPIO(RCTransmitService *rcTservice, Callback *callback) {
	service = rcTservice;
	myCallback = callback;
}

void AvantGPIO::digitalWrite(uint8_t pin, bool logicLevel) {
	service->sendData(logicLevel, 6, pin);
}

void AvantGPIO::pinMode(uint8_t pin, bool logicLevel) {
	service->sendData(logicLevel, 5, pin);
}

void AvantGPIO::digitalRead(uint8_t pin) {
	service->sendData(0, 8, pin);
}

void AvantGPIO::analogWrite(uint8_t pin, uint8_t value) {
	service->sendData(value, 8, pin);
}

void AvantGPIO::digitalReadCallback(void (*function)(byte)) {
   (*myCallback).digitalRead = function;
}

//**********************************
//AvantI2C Class Implementation
//**********************************
AvantI2C::AvantI2C(){}

AvantI2C::AvantI2C(RCTransmitService *rcTservice, Callback *callback) {
	service = rcTservice;
	myCallback = callback;
}

void AvantI2C::deviceID(uint8_t ID){
	service->sendData(ID, 11, 7);
}
void AvantI2C::beginTransmission(void){
	service->sendData(0, 11, 8);
}

void AvantI2C::endTransmission(void){
	service->sendData(0, 11, 4);
}

void AvantI2C::write(uint8_t data){
	service->sendData(data, 11, 3);
}

void AvantI2C::read(void){
	service->sendData(0, 11, 5);
}
void AvantI2C::wireRequest(uint8_t bytes){
	service->sendData(bytes, 11, 6);
}

void AvantI2C::readCallback(void (*function)(byte)) {
   (*myCallback).i2cRead = function;
}


//********************************************
//AvantXbee Class Implementation
//********************************************
AvantXbee::AvantXbee(){}

AvantXbee::AvantXbee(RCTransmitService *rcTservice, Callback *callback) {
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

AvantPose::AvantPose(RCTransmitService *rcTservice, Callback *callback) {
	service = rcTservice;
	myCallback = callback;
}

void AvantPose::getGPSData(void) {
	service->sendData(0, 9, 1);
}

void AvantPose::getLatitude(void) {
	service->sendData(0, 9, 2);
}

void AvantPose::getLongitude(void) {
	service->sendData(0, 9, 3);
}

void AvantPose::getAltitude(void ) {
	service->sendData(0, 9, 4);
}

void AvantPose::getSatellites(void) {
	service->sendData(0, 9, 5);
}

void AvantPose::getSpeed(void) {
	service->sendData(0, 9, 6);
}

void AvantPose::getOrientation(void){
	service->sendData(0, 9, 7);
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
	(*myCallback).satallite = function;
}

void AvantPose::orientationCallback(void (*function)(float)) {
	(*myCallback).orientation = function;
}


//*******************************************
//AvantSPI Class Implementation
//*******************************************
AvantSPI::AvantSPI(){}
AvantSPI::AvantSPI(RCTransmitService *rcTservice, Callback *callback) {
    service = rcTservice;
	myCallback = callback;
}

void AvantSPI::transfer(uint8_t data){
	service->sendData(data, 10, 1);
}

void AvantSPI::setBitOrder(uint8_t data){
	service->sendData(data, 10, 2);
}

void AvantSPI::setClockDivider(uint8_t data){
	service->sendData(data, 10, 3);
}

void AvantSPI::setDataMode(uint8_t data){
	service->sendData(data, 10, 4);
}

void AvantSPI::transferCallback(void (*function)(byte)) {
	(*myCallback).transfer = function;
}
//*******************************************
//AvantResponseHandler Class Implementation
//*******************************************
AvantResponseHandler::AvantResponseHandler(){};

AvantResponseHandler::AvantResponseHandler(RCTransmitService *rcTservice, Callback *callback) {
    service = rcTservice;
	myCallback = callback;
}

float AvantResponseHandler::dataToFloat(char data[]) {
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

void AvantResponseHandler::responseHandler() {
if(service->isHwSerial0Used) {
  char buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
  int length = 0;
  int resourceID = 0;
  int actionID = 0;
  int datasum = 0;
  while (Serial.available() > 0) { 
    if (Serial.read() == '$'){  //this is the start of a Data Packet
      if (Serial.readBytes(&buffer[0], 3) == 3){ //read the next three bytes of the stream and store them into buffer[]
        length = byte(buffer[0]); //get the length of the data
        resourceID = byte(buffer[1]); //get the resourceID
        actionID = byte(buffer[2]); //get the actionID
        char data[length+1]; // create a buffer to store the data and checksum info 
        if(Serial.readBytes(&data[0], length+1) == length+1) { //get the data and checksum from Serial Stream and store into buffer
          for(int i=0;i<length;i++){
            datasum = datasum+byte(data[i]);
          }
          
          if(byte(data[length])==(length+resourceID+actionID+datasum)%256){  //check the checksum
            ///////route the data to the appropriate place here
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
                break;
               case 9:
                //Pose_Data
				if(actionID == 2)
					(*myCallback).longitude(dataToFloat(data));
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
            }//end of data packet router
          }//end of checksum
          else
            Serial.println("checksum failed");
        }//end of Serial.readBytes for data and checksum
        else
          Serial.println("Data Wrong Size");
      }//End of Serial.readBytes for buffer
      else 
        Serial.println("Packet Missing Header");
    }//end of if Serial available
  }//end of Serial.read
}

#if defined(UBRR1H)
	if(service->isHwSerial1Used) {
	  char buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
	  int length = 0;
	  int resourceID = 0;
	  int actionID = 0;
	  int datasum = 0;
	  while (Serial1.available() > 0) { 
		if (Serial1.read() == '$'){  //this is the start of a Data Packet
		  Serial.print("got data");
		  if (Serial1.readBytes(&buffer[0], 3) == 3){ //read the next three bytes of the stream and store them into buffer[]
			length = byte(buffer[0]); //get the length of the data
			resourceID = byte(buffer[1]); //get the resourceID
			actionID = byte(buffer[2]); //get the actionID
			char data[length+1]; // create a buffer to store the data and checksum info 
			if(Serial1.readBytes(&data[0], length+1) == length+1) { //get the data and checksum from Serial Stream and store into buffer
			  for(int i=0;i<length;i++){
				datasum = datasum+byte(data[i]);
			  }
			  
			  if(byte(data[length])==(length+resourceID+actionID+datasum)%256){  //check the checksum
				///////route the data to the appropriate place here
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
					break;
				   case 9:
					//Pose_Data
					if(actionID == 2){
						(*myCallback).longitude(dataToFloat(data));
						Serial.println("hi");
						}
					break;
				   case 10:
					//SPI_Communication
					break;
				   case 11:
					//I2C_Communication
					//Avant::i2cRead(data);
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
				}//end of data packet router
			  }//end of checksum
			  else
				Serial.println("checksum failed");
			}//end of Serial.readBytes for data and checksum
			else
			  Serial.println("Data Wrong Size");
		  }//End of Serial.readBytes for buffer
		  else 
			Serial.println("Packet Missing Header");
		}//end of if Serial available
	  }//end of Serial.read
	}
#endif
#if defined(UBRR2H)
	if(service->isHwSerial2Used) {
	  char buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
	  int length = 0;
	  int resourceID = 0;
	  int actionID = 0;
	  int datasum = 0;
	  while (Serial2.available() > 0) { 
		if (Serial2.read() == '$'){  //this is the start of a Data Packet
		  if (Serial2.readBytes(&buffer[0], 3) == 3){ //read the next three bytes of the stream and store them into buffer[]
			length = byte(buffer[0]); //get the length of the data
			resourceID = byte(buffer[1]); //get the resourceID
			actionID = byte(buffer[2]); //get the actionID
			char data[length+1]; // create a buffer to store the data and checksum info 
			if(Serial2.readBytes(&data[0], length+1) == length+1) { //get the data and checksum from Serial Stream and store into buffer
			  for(int i=0;i<length;i++){
				datasum = datasum+byte(data[i]);
			  }
			  
			  if(byte(data[length])==(length+resourceID+actionID+datasum)%256){  //check the checksum
				///////route the data to the appropriate place here
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
					break;
				   case 9:
					//Pose_Data
					if(actionID == 2)
						(*myCallback).longitude(dataToFloat(data));
					break;
				   case 10:
					//SPI_Communication
					break;
				   case 11:
					//I2C_Communication
					//Avant::i2cRead(data);
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
				}//end of data packet router
			  }//end of checksum
			  else
				Serial.println("checksum failed");
			}//end of Serial.readBytes for data and checksum
			else
			  Serial.println("Data Wrong Size");
		  }//End of Serial.readBytes for buffer
		  else 
			Serial.println("Packet Missing Header");
		}//end of if Serial available
	  }//end of Serial.read
	}
#endif
#if defined(UBRR3H)
	if(service->isHwSerial3Used) {
	  char buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
	  int length = 0;
	  int resourceID = 0;
	  int actionID = 0;
	  int datasum = 0;
	  while (Serial3.available() > 0) { 
		if (Serial3.read() == '$'){  //this is the start of a Data Packet
		  if (Serial3.readBytes(&buffer[0], 3) == 3){ //read the next three bytes of the stream and store them into buffer[]
			length = byte(buffer[0]); //get the length of the data
			resourceID = byte(buffer[1]); //get the resourceID
			actionID = byte(buffer[2]); //get the actionID
			char data[length+1]; // create a buffer to store the data and checksum info 
			if(Serial3.readBytes(&data[0], length+1) == length+1) { //get the data and checksum from Serial Stream and store into buffer
			  for(int i=0;i<length;i++){
				datasum = datasum+byte(data[i]);
			  }
			  
			  if(byte(data[length])==(length+resourceID+actionID+datasum)%256){  //check the checksum
				///////route the data to the appropriate place here
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
					break;
				   case 9:
					//Pose_Data
					if(actionID == 2)
						(*myCallback).longitude(dataToFloat(data));
					break;
				   case 10:
					//SPI_Communication
					break;
				   case 11:
					//I2C_Communication
					//Avant::i2cRead(data);
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
				}//end of data packet router
			  }//end of checksum
			  else
				Serial.println("checksum failed");
			}//end of Serial.readBytes for data and checksum
			else
			  Serial.println("Data Wrong Size");
		  }//End of Serial.readBytes for buffer
		  else 
			Serial.println("Packet Missing Header");
		}//end of if Serial available
	  }//end of Serial.read
	}
#endif
if(service->isSwSerialUsed) {
  char buffer[2];  //this is a buffer to store the length, resourceID, ActionID coming in through serial
  int length = 0;
  int resourceID = 0;
  int actionID = 0;
  int datasum = 0;
  while (service->softwareSerial.available() > 0) { 
	if (service->softwareSerial.read() == '$'){  //this is the start of a Data Packet
	  if (service->softwareSerial.readBytes(&buffer[0], 3) == 3){ //read the next three bytes of the stream and store them into buffer[]
		length = byte(buffer[0]); //get the length of the data
		resourceID = byte(buffer[1]); //get the resourceID
		actionID = byte(buffer[2]); //get the actionID
		char data[length+1]; // create a buffer to store the data and checksum info 
		if(service->softwareSerial.readBytes(&data[0], length+1) == length+1) { //get the data and checksum from Serial Stream and store into buffer
		  for(int i=0;i<length;i++){
			datasum = datasum+byte(data[i]);
		  }
		  
		  if(byte(data[length])==(length+resourceID+actionID+datasum)%256){  //check the checksum
			///////route the data to the appropriate place here
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
				break;
			   case 9:
				//Pose_Data
				if(actionID == 2)
					(*myCallback).longitude(dataToFloat(data));
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
			}//end of data packet router
		  }//end of checksum
		  else
			Serial.println("checksum failed");
		}//end of Serial.readBytes for data and checksum
		else
		  Serial.println("Data Wrong Size");
	  }//End of Serial.readBytes for buffer
	  else 
		Serial.println("Packet Missing Header");
	}//end of if Serial available
  }//end of Serial.read
}
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
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
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
bool SoftwareSerial::listen()
{
  if (active_object != this)
  {
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
void SoftwareSerial::recv()
{

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
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
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
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else 
    {
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

void SoftwareSerial::tx_pin_write(uint8_t pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}

uint8_t SoftwareSerial::rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
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
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(uint8_t rx)
{
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

void SoftwareSerial::begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
  {
    long baud = pgm_read_dword(&table[i].baud);
    if (baud == speed)
    {
      _rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
      _rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
      _rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
      _tx_delay = pgm_read_word(&table[i].tx_delay);
      break;
    }
  }

  // Set up RX interrupts, but only if we have a valid RX baud rate
  if (_rx_delay_stopbit)
  {
    if (digitalPinToPCICR(_receivePin))
    {
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

void SoftwareSerial::end()
{
  if (digitalPinToPCMSK(_receivePin))
    *digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
}


// Read data from buffer
int SoftwareSerial::read()
{
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

int SoftwareSerial::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
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
  if (_inverse_logic)
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(LOW); // send 1
      else
        tx_pin_write(HIGH); // send 0
    
      tunedDelay(_tx_delay);
    }

    tx_pin_write(LOW); // restore pin to natural state
  }
  else
  {
    for (byte mask = 0x01; mask; mask <<= 1)
    {
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

void SoftwareSerial::flush()
{
  if (!isListening())
    return;

  uint8_t oldSREG = SREG;
  cli();
  _receive_buffer_head = _receive_buffer_tail = 0;
  SREG = oldSREG;
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}