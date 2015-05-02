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

*/

#ifndef __ArduinoSDK__Drone__
#define __ArduinoSDK__Drone__

#include <inttypes.h>
#include <Stream.h>
#include <Arduino.h>
#include "RC.h"
#include "SerialIO.h"
#include "Callback.h"
#include "Pose.h"
#include "GPIO.h"
#include "I2C.h"
#include "ResponseHandler.h"
#include "Vitals.h"

class Drone 
{
private:
  RC rc;
  SerialIO serialIO;
  GPIO gpio;
  IncomingPacketReader incomingPacketReader;
  Vitals vitals;
  ResponseHandler responseHandler;
  I2C i2c;
  Callback callback;
  Pose pose;

public:
  Drone();
  Drone(SerialPort serialPort);
  Drone(int txPin, int rxPin);

  void listen();
  void arm();
  void initialize();

  /**
       
  This sends only one message to request all of the POSE data. 
  The Extender will reply with 6 messages containing Long, Lat, number of Sat, Speed, Orientation, and Altitude
       
  */
  void getGPSData();

  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Longitude
  The response gets handled through the callback function longitudeCallbackCallback().
       
  */
  void getLongitude();

  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Latitude.
  The response gets handled through the callback function latitudeCallback().
       
  */
  void getLatitude();

  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Altitude.
  The response gets handled through the callback function altitudeCallback().
       
  */
  void getAltitude();

  /**
       
  Sends a request to the Extender asking for it to reply with the current number of satellites the vehicles
  GPS is locked onto.
  The response gets handled through the callback function satelliteCallback().
  The more satellites you have the higher the accuracy is.  There needs to be a minimum of 4 satellites to get a lock.  
       
  */
  void getSatellites();

  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Land Speed given by the GPS.
  The response gets handled through the callback function speedCallback().
       
  */
  void getSpeed();

  /**
       
  Sends a request to the Extender asking for it to reply with the vehicles current Orientation given by the Compass.
  The response gets handled through the callback function orientationCallback().
       
  */
  void getOrientation();

  /**
       
  Callback function which passes longitude information to the function that it is passed.
       
  */
  void longitudeCallback(void (*cb)(float));

  /**
       
  Callback function which passes latitude information to the function that it is passed.
       
  */
  void latitudeCallback(void (*cb)(float));

  /**
       
  Callback function which passes altitude information to the function that it is passed.
       
  */
  void altitudeCallback(void (*cb)(float));

  /**
       
  Callback function which passes number of locked satellites to the function that it is passed.
       
  */
  void satelliteCallback(void (*cb)(int16_t));

  /**
       
  Callback function which passes speed information to the function that it is passed.
       
  */
  void speedCallback(void (*cb)(float));

  /**
       
  Callback function which passes orientation information to the function that it is passed
       
  */
  void orientationCallback(void (*cb)(float));

  /**

  Synchronous version of `getLatitude`.

  */
  float getLatitudeSync();

  /**

  Synchronous version of `getLongitude`.

  */
  float getLongitudeSync();

  /**

  Synchronous version of `getAltitude`.

  */
  float getAltitudeSync();

  /**

  Synchronous version of `getSatellites`.

  */
  int16_t getSatellitesSync();

  /**

  Synchronous version of `getSpeed`.

  */
  float getSpeedSync();

  /**

  Synchronous version of `getOrientation`.

  */
  float getOrientationSync();

  /**

  Sends a request to the extender to set the Aileron channel to a specific channel between -100 and 100.

  @param value The value to set the Aileron channel to

  */
  void setAileron(int8_t value);

  /**

  Sends a request to the extender to set the Extender channel to a specific channel between -100 and 100.

  @param value The value to set the Extender channel to

  */
  void setElevator(int8_t value);

  /**

  Sends a request to the extender to set the Throttle channel to a specific channel between -100 and 100.

  @param value The value to set the Throttle channel to

  */
  void setThrottle(int8_t value);

  /**

  Sends a request to the extender to set the Rudder channel to a specific channel between -100 and 100.

  @param value The value to set the Rudder channel to

  */
  void setRudder(int8_t value);

  /**

  Sends a request to the extender to set the Flight Mode channel to a specific channel between -100 and 100.

  @param value The value to set the Flight Mode channel to

  */
  void setFlightMode(int8_t value);

  /**

  Sends a request to the extender to reply with message containing the current Aileron value.

  */
  void getAileron();

  /**

  Sends a request to the extender to reply with message containing the current Elevator value.

  */
  void getElevator();

  /**

  Sends a request to the extender to reply with message containing the current Throttle value.

  */
  void getThrottle();

  /**

  Sends a request to the extender to reply with message containing the current Rudder value.

  */
  void getRudder();

  /**

  Sends a request to the extender to reply with message containing the current Flight Mode value.

  */
  void getFlightMode();

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void aileronCallback(void (*cb)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void flightModeCallback(void (*cb)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void throttleCallback(void (*cb)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void rudderCallback(void (*cb)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void elevatorCallback(void (*cb)(int16_t));

  /**

  Execute funtion in a thread to not freez the drone.

  */
  void sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron);

  /**

  Synchronous version of `getAileron`.

  */
  int16_t getAileronSync();

  /**

  Synchronous version of `getElevator`.

  */
  int16_t getElevatorSync();

  /**

  Synchronous version of `getThrottle`.

  */
  int16_t getThrottleSync();

  /**

  Synchronous version of `getRudder`.

  */
  int16_t getRudderSync();

  /**

  Synchronous version of `getFlightMode`.

  */
  int16_t getFlightModeSync();
  
  /**
   
  Configures the specified pin on the App Extender to behave either as an input or an output.

   
  @param pin value expected 0 to 10
  @param logicLevel expected 0 or 1
   
  */
  void pinMode(uint8_t pin, int logicLevel);

  /**

  Write a HIGH or LOW value to a digital pin on the App Extender.

  @param pin value expected 0 to 10
  @param logicLevel expected 0 or 1

  */
  void digitalWrite(uint8_t pin, bool logicLevel);

  /**

  Writes an analog value (PWM wave) to the specified pin on the App Extender. 

  @param pin sets pins 1-10 on the App Extender to the PWM value specified by value
  @param value sets the PWM values duty cycle ranging from 0 and 255 

  */
  void analogWrite(uint8_t pin, uint8_t value);

  /**
   
  Writes an a request to the Extender to check the duration of a pulse on the specified pin.

  @param pin sets pins 1-10 on the App Extender to the PWM value specified by value 

  */
  void pulseIn(uint8_t pin);

  /**

  TODO: add documentation.

  */
  int pulseInSync(uint8_t pin);


  /**

  Sends a request to the App Extender to reply with the logical state of the specified pin.  

  @param pin Selects which GPIO pin on the App Extender it should return.

  */
  void digitalRead(uint8_t pin);

  /**

  TODO: add documentation.

  */
  int digitalReadSync(uint8_t pin);

  /**
   
  Sends a request to the App Extender to reply with the analog value of the specified pin.  

  @param pin Selects which analog pin on the App Extender it should return.

  */
  void analogRead(uint8_t pin);

  /**

  TODO: add documentation.

  */
  int analogReadSync(uint8_t pin);

  /**

  This callback function is executed everytime it gets digitalRead information.  It passes
  this information to the function specified in the argument.

  @param function function name of the function to pass received information to
  @param pin the desired pin to sample logic level information from 

  */
  void digitalReadCallback(void (*cb)(byte), int pin);

  /**

  This callback function is executed every time it gets pulseIn information.  It passes
  this information to the function specified in the argument.
   
  @param function name of the function to pass received information to
  @param pin the desired pin to sample pulseIn information from 
   
  */
  void pulseInCallback(void (*cb)(long), uint8_t pin);

  /**
   
  This callback function is executed every time it gets analogRead information.  It passes
  this information to the function specified in the argument.
   
  @param function function name of the function to pass received information to
  @param pin the desired pin to sample analog information from 
   
  */
  void analogReadCallback(void (*cb)(byte), uint8_t pin);

  /**

  Associates a servo with a pin.

  @param servoNumber which of the three servos to attach. Values should be 1-3.
  @param pin the desired pin to associate with the servo.

  */
  void attachServo(uint8_t servoNumber, uint8_t pin);

  /**

  Clear a previously set assocation between the given servo and pin.

  @param servoNumber the desired servo to detach. Values should be 1-3.

  */
  void detachServo(uint8_t servoNumber);

  /**

  Write data to a servo. This servo should already be attached via the `attachServo` method.

  @param servoNumber the servo to send data too. Values should be 1-3.
  @param data the data to send to the servo.

  */
  void writeServo(uint8_t servoNumber, uint8_t data);

  /**
       
  This sets the address of the device that the App Extender sends data too.
  This address will be continuously used by beginTransmission, write, read, and wireRequest
  until it is reset to a different value.  
      
  @param ID range from 0 to 255
      
  */
  void setDeviceAddress(uint8_t ID);

  /**

  Begin a transmission to the I2C slave device with the address specified by deviceID(ID)
  Subsequently, queue bytes for transmission with the write() function and transmit them
  by calling endTransmission().
      
  */
  void beginTransmission();

  /**

  Ends a transmission to a slave device that was begun by beginTransmission() and transmits
  the bytes that were queued by write().
       
  */
  void endTransmission();

  /**
       
  Queues bytes for transmission from the App Extender to a slave device, in between calls to 
  beginTransmission() and endTransmission().  
      
  @param data Adds one byte of information to the queue
      
  */
  void write(uint8_t data);

  /**

  Sends a request to the App Extender to read one byte of information from the I2C buffer and return 
  the information to the arduino.  The relpies are handled by the readCallback() function.  
     
  */
  void read();

  /**

  TODO: add documentation.

  */
  int16_t readSync();

  /**

  Sends a request to the App Extender to read multiple bytes of information from the I2C buffer and return 
  the information to the arduino. The relpies are handled by the readCallback() function.  
      
  @param byteCount Sets the number of bytes to read from the I2C buffer
      
  */
  void wireRequest(uint8_t byteCount);

  /**
       
  This callback function is executed everytime it gets I2C information.  It passes
  this information to the function specified in the argument.
      
  @param function name of the method

  */
  void readCallback(void (*cb)(byte));

  /**

  TODO: add documentation.

  */
  void heartbeatLostCallback(void (*cb)(void));

  /**

  TODO: add documentation.

  */
  void getVoltage();

  /**

  TODO: add documentation.

  */
  int16_t getVoltageSync();

  /**

  TODO: add documentation.

  */
  void voltageCallback(void (*cb)(uint8_t));

  /**

  TODO: add documentation.

  */
  void getSignalStrength();

  /**

  TODO: add documentation.

  */
  int16_t getSignalStrengthSync();

  /**

  TODO: add documentation.

  */
  void signalStrengthCallback(void (*cb)(uint8_t));

};

#endif /* defined __ArduinoSDK__Drone__ */