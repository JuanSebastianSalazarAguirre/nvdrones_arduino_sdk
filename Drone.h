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
#include "AvantRC.h"
#include "SerialIO.h"
#include "Callback.h"

/******************************************************************************
* Definitions
******************************************************************************/



//*****************************************
//Avant Classes
//*****************************************
//\cond

  

//\cond
class AvantResponseHandler
//\endcond
{
  private:
    SerialIO *service;
    Callback *myCallback;
    float dataToFloat(byte data[]);
    long  dataToLong(byte data[]);
  public:
    AvantResponseHandler();
    AvantResponseHandler(SerialIO *rcTservice, Callback *callback);
    void responseHandler();
    void callbackTest(byte test){
    myCallback->i2cRead(test);
    }   
};

class AvantPose
/**
 
 The pose class allow you to gather from one place all the data related to position. 
 In this case we are talking about GPS location and compass orientation.
 
 */
{
  private:
     //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantPose();
    
    AvantPose(SerialIO *rcTservice, Callback *callback);
    //\endcond

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
    void longitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes latitude information to the function that it is passed.
         
    */
    void latitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes altitude information to the function that it is passed.
         
    */
    void altitudeCallback(void (*function)(float));
    
    /**
         
    Callback function which passes number of locked satellites to the function that it is passed.
         
    */
    void satelliteCallback(void (*function)(byte));
    
    /**
         
    Callback function which passes speed information to the function that it is passed.
         
    */
    void speedCallback(void (*function)(float));
    
    /**
         
    Callback function which passes orientation information to the function that it is passed
         
    */
    void orientationCallback(void (*function)(float));
};

class AvantGPIO
/**

 This class controls all functionality related to the general purpose IO pins on the drone, 
 includes turning pins high/low, setting PWM values, reading if pins are high/low ect.
 
 Basically all the extra hardware that you connected to the 10 pins on the NVextenderd can be 
 accessed and controlled using this Class.
 
*/
{
  private:
    //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantGPIO();
    
    AvantGPIO(SerialIO *rcTservice, Callback *callback);
    //\endcond


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
 
    Sends a request to the App Extender to reply with the logical state of the specified pin.  
 
    @param pin Selects which GPIO pin on the App Extender it should return.
 
    */
    void digitalRead(uint8_t pin);
    
    /**
     
    Sends a request to the App Extender to reply with the analog value of the specified pin.  
 
    @param pin Selects which analog pin on the App Extender it should return.
 
    */
    void analogRead(uint8_t pin);

    /**
 
    This callback function is executed everytime it gets digitalRead information.  It passes
    this information to the function specified in the argument.
 
    @param function function name of the function to pass received information to
    @param pin the desired pin to sample logic level information from 
 
    */
    void digitalReadCallback(void (*function)(byte), int pin);
    
    /**
    
    This callback function is executed every time it gets pulseIn information.  It passes
    this information to the function specified in the argument.
     
    @param function name of the function to pass received information to
    @param pin the desired pin to sample pulseIn information from 
     
    */
    void pulseInCallback(void (*function)(long), uint8_t pin);
    
    /**
     
    This callback function is executed every time it gets analogRead information.  It passes
    this information to the function specified in the argument.
     
    @param function function name of the function to pass received information to
    @param pin the desired pin to sample analog information from 
     
    */
    void analogReadCallback(void (*function)(byte), uint8_t pin);

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
};

class AvantI2C
/**

 This class allow you to gain access to the I2C serial bus, allowing you to communicate with
 sensors and actuators that you connected to the NVextender.
 
 # Example

 @htmlonly
    <script src="https://gist.github.com/davidgatti/790c8fdcd27385b12043.js"></script>
 @endhtmlonly
 
 */
{
  private:
    //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantI2C();
    
    AvantI2C(SerialIO *rcTservice, Callback *callback);
    //\endcond

    /**
         
    This sets the address of the device that the App Extender sends data too.
    This address will be continuously used by beginTransmission, write, read, and wireRequest
    until it is reset to a different value.  
        
    @param ID range from 0 to 255
        
    */
    void deviceID(uint8_t ID);
    
    /**
    
    Begin a transmission to the I2C slave device with the address specified by deviceID(ID)
    Subsequently, queue bytes for transmission with the write() function and transmit them
    by calling endTransmission().
        
    */
    void beginTransmission(void);
    
    /**
    
    Ends a transmission to a slave device that was begun by beginTransmission() and transmits
    the bytes that were queued by write().
         
    */
    void endTransmission(void);
    
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
    void read(void);
    
    /**
    
    Sends a request to the App Extender to read multiple bytes of information from the I2C buffer and return 
    the information to the arduino. The relpies are handled by the readCallback() function.  
        
    @param bytes Sets the number of bytes to read from the I2C buffer
        
    */
    void wireRequest(uint8_t bytes);
    
    /**
         
    This callback function is executed everytime it gets I2C information.  It passes
    this information to the function specified in the argument.
        
    @param function name of the method
    
    */
    void readCallback(void (*function)(byte));
};

class AvantSPI 
/**

This class allow you to gain acess to the SPI serial bus, allowing you to communicate with
sensors and actuators that you connected to the NVextender using the exposed PINs.
 
To learn more about SPI, visit the followign line: http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
 
*/
{
  private:
    //\cond
    SerialIO *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantSPI();
    
    AvantSPI(SerialIO *rcTservice, Callback *callback);
    //\endcond

    /**

    Transfers one byte of information from the App Extender over the SPI bus.

    @param data the information to pass to the slave device.

    */
    void transfer(byte data);

    /**
    
    Sets the order of the bits shifter out of the SPI bus, either LSBFIRST or MSBFIRST

    @param data Specifies either LSBFIRST or MSBFIRST

    */
    void setBitOrder(byte data);

    /**

    Sets the SPI clock divider relative to the system clock.  Available dividers include 2, 4, 8, 16, 32, 64, and 128.
    The default setting is 4.

    @param data Determines which divider to use

    */
    void setClockDivider(byte data);

    /**

    Sets the SPI data mode: that is, clock polarity and phase.  There are 4 possible modes 0-3.  
    
    @param data Sets the SPI mode.

    */
    void setDataMode(byte data);

    /**
         
    This callback function is executed everytime it gets SPI information.  It passes
    this information to the function specified in the argument.
         
    */
    void transferCallback(void (*function)(byte));
};

class AvantAutoPilot
{
private:
    SerialIO *service;
    Callback *myCallback;

public:
    AvantAutoPilot();
    AvantAutoPilot(SerialIO *rcTservice, Callback *callback);
    void gpsExecute();
    void compassExecute();
    void setYawError(float error);
    void setThrottleError(float error);
    void setElevatorError(float error);
    void setAileronError(float error);
    void setWaypointLatitude(float latitude);
    void setWaypointLongitude(float longitude);
    void setWaypointAltitude(float altitude);
    void setWaypointOrientation(float orientation);
    void setYawKP(float kp);
    void setYawKD(float kd);
    void setYawKI(float ki);
    void setThrottleKP(float kp);
    void setThrottleKD(float kd);
    void setThrottleKI(float ki);
    void setElevatorKP(float kp);
    void setElevatorKD(float kd);
    void setElevatorKI(float ki);
    void setAileronKP(float kp);
    void setAileronKD(float kd);
    void setAileronKI(float ki);
    void getWaypointLatitude(void (*function)(float));
    void getWaypointLongitude(void (*function)(float));
    void getWaypointAltitude(void (*function)(float));
    void getWaypointOrientation(void (*function)(float));
    void getYawKP(void (*function)(float));
    void getYawKD(void (*function)(float));
    void getYawKI(void (*function)(float));
    void getThrottleKP(void (*function)(float));
    void getThrottleKD(void (*function)(float));
    void getThrottleKI(void (*function)(float));
    void getElevatorKP(void (*function)(float));
    void getElevatorKD(void (*function)(float));
    void getElevatorKI(void (*function)(float));
    void getAileronKP(void (*function)(float));
    void getAileronKD(void (*function)(float));
    void getAileronKI(void (*function)(float));
};

//\cond
class Drone 
//\endcond
{
    private:
      AvantRC avantRC;
      SerialIO serialIO;
      AvantGPIO avantGPIO;
      AvantResponseHandler responseHandler;
      AvantI2C avantI2C;
      Callback callback;
      AvantPose avantPose;
      AvantSPI avantSPI;
      AvantAutoPilot avantAutoPilot;
    public:
      Drone();
      Drone(SerialPort serialPort);
      Drone(int txPin, int rxPin);
      AvantRC& RC();
      AvantGPIO& GPIO();
      AvantI2C& I2C();
      AvantPose& pose();
      AvantResponseHandler& avantResponseHandler();
      AvantSPI& SPI();
      AvantAutoPilot& AutoPilot();
      void arm();
      void initialize();
};

#endif
