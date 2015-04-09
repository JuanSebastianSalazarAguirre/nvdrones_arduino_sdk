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
#ifndef Avant_h
#define Avant_h

#include <inttypes.h>
#include <Stream.h>
#include <Arduino.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 256 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

//********************************************
//SoftwareSerial Code
//*****************************************
//\cond
class SoftwareSerial : public Stream
//\endcond
{
  private:
    // per object data
    uint8_t _receivePin;
    uint8_t _receiveBitMask;
    volatile uint8_t *_receivePortRegister;
    uint8_t _transmitBitMask;
    volatile uint8_t *_transmitPortRegister;

    uint16_t _rx_delay_centering;
    uint16_t _rx_delay_intrabit;
    uint16_t _rx_delay_stopbit;
    uint16_t _tx_delay;

    uint16_t _buffer_overflow:1;
    uint16_t _inverse_logic:1;

    // static data
    static char _receive_buffer[_SS_MAX_RX_BUFF]; 
    static volatile uint8_t _receive_buffer_tail;
    static volatile uint8_t _receive_buffer_head;
    static SoftwareSerial *active_object;

    // private methods
    void recv();
    uint8_t rx_pin_read();
    void tx_pin_write(uint8_t pin_state);
    void setTX(uint8_t transmitPin);
    void setRX(uint8_t receivePin);

    // private static method for timing
    static inline void tunedDelay(uint16_t delay);

  public:
    // public methods
    SoftwareSerial(){};
    SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
    ~SoftwareSerial();
    void begin(long speed);
    bool listen();
    void end();
    bool isListening() { return this == active_object; }
    bool overflow() { bool ret = _buffer_overflow; _buffer_overflow = false; return ret; }
    int peek();

    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int available();
    virtual void flush();

    using Print::write;

    // public only for easy access by interrupt handlers
    static inline void handle_interrupt();
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif

//*****************************************
//Avant Classes
//*****************************************
//\cond
class Callback 
//\endcond
{
  public:
    //SPI Callbacks
    void (*transfer)(byte);
    //I2C Callbacks
    void (*i2cRead)(byte); 
    //GPS callbacks
    void (*longitude)(float);
    void (*latitude)(float);
    void (*altitude)(float);
    void (*speed)(float);
    void (*satellite)(byte);
    void (*orientation)(float);
    //RC Callbacks
    void (*flightMode)(byte);
    void (*elevator)(byte);
    void (*aileron)(byte);
    void (*throttle)(byte);
    void (*rudder)(byte);
    //pulseIn Callbacks
    void (*pulseIn1)(long);
    void (*pulseIn2)(long);
    void (*pulseIn3)(long);
    void (*pulseIn4)(long);
    void (*pulseIn5)(long);
    void (*pulseIn6)(long);
    void (*pulseIn7)(long);
    void (*pulseIn8)(long);
    void (*pulseIn9)(long);
    void (*pulseIn10)(long);
    //digital Read Callbacks
    void (*digitalRead1)(byte);
    void (*digitalRead2)(byte);
    void (*digitalRead3)(byte);
    void (*digitalRead4)(byte);
    void (*digitalRead5)(byte);
    void (*digitalRead6)(byte);
    void (*digitalRead7)(byte);
    void (*digitalRead8)(byte);
    void (*digitalRead9)(byte);
    void (*digitalRead10)(byte);
    //analog Read Callbacks
    void (*analogRead1)(byte);
    void (*analogRead2)(byte);
    void (*analogRead3)(byte);
    void (*analogRead4)(byte);
    //Xbee Callbacks
    //AutoPilot Callbacks
    void (*getWaypointLatitude)(float);
    void (*getWaypointLongitude)(float);
    void (*getWaypointAltitude)(float);
    void (*getWaypointOrientation)(float);
    void (*getElevatorKP)(float);
    void (*getElevatorKD)(float);
    void (*getElevatorKI)(float);
    void (*getThrottleKP)(float);
    void (*getThrottleKD)(float);
    void (*getThrottleKI)(float);
    void (*getAileronKP)(float);
    void (*getAileronKD)(float);
    void (*getAileronKI)(float);
    void (*getYawKP)(float);
    void (*getYawKD)(float);
    void (*getYawKI)(float);
    
};
  

//\cond
class RCTransmitService
//\endcond
{
  friend class AvantResponseHandler;
    private:
      bool isHwSerial0Used;
      bool isHwSerial1Used;
      bool isHwSerial2Used;
      bool isHwSerial3Used;
      bool isSwSerialUsed;

    public:
      SoftwareSerial softwareSerial;
      RCTransmitService();

      /**
      TODO

      @param txPin
      @param rxPin
      */
      RCTransmitService(int txPin , int rxPin);

      /**
      TODO

      @param hwSerialCode
      */
      RCTransmitService(int hwSerialCode);


      void sendData(int data, uint8_t resourceID, uint8_t actionID);
  

      void sendData(uint8_t data, uint8_t resourceID, uint8_t actionID);
      

      void sendData(float data, uint8_t resourceID, uint8_t actionID);

      /**
      Method description
      */
      void print(String data);
      
      /**
      Method description
      */
      void write(byte data);

      /**
      Method description
      */
      void readBytes(char *buffer, int bytesToRead);
};

//\cond
class AvantResponseHandler
//\endcond
{
  private:
    RCTransmitService *service;
    Callback *myCallback;
    float dataToFloat(byte data[]);
    long  dataToLong(byte data[]);
  public:
    AvantResponseHandler();
    AvantResponseHandler(RCTransmitService *rcTservice, Callback *callback);
    void responseHandler();
    void callbackTest(byte test){
    myCallback->i2cRead(test);
    }   
};

class AvantTransmitter 
/**
 This class allows you to do initial setup of the hardware after you connected it to the drone. 
 You will have to tell the NVextender what pin is connected to what drone controller. 
 Based on the naming convention from the methods in this class you should be able 
 to easily understand what type of information the hardware needs.

 # Example

 @htmlonly
    <script src="https://gist.github.com/davidgatti/cf3331742d53f5f80609.js"></script>
 @endhtmlonly

*/
{
  private:
    int elevatorPin;
    int AileronPin;
    int throttlePin;
    int rudderPin;
    int flightModePin;
    
    int elevatorMax;
    int elevatorMin;
    int aileronMax;
    int aileronMin;
    int throttleMax;
    int throttleMin;
    int rudderMax;
    int rudderMin;
    
    //\cond
    RCTransmitService *service;
    //\endcond
  public:
    //\cond
    AvantTransmitter();
    
    AvantTransmitter(RCTransmitService *rcService);
    //\endcond

    /**
    Sets the analogue pin that the Elevator potentiometer is connected to.

    @param pin Hardware pin number.
    */
    void setElevatorPin(int pin);
  
    /**
    Returns the analogue pin that the Aileron potentiometer is connected to.
    */
    int getElevatorPin();
  
    /**
    Sets the analogue pin that the Aileron potentiometer is connected to.
     
    @param pin number of the hardware pin you connected it to.
    */
    void setAileronPin(int pin);
  
    /**
    Returns the analogue pin that the Aileron potentiometer is connected to.
     
    */
    int getAileronPin();
  
    /**
    Sets the analogue pin that the Throttle potentiometer is connected to.
     
    @param pin number of the hardware pin you connected it to.
    */
    void setThrottlePin(int pin);
  
    /**
    Returns the analogue pin that the Elevator potentiometer is connected to.

    */
    int getThrottlePin();
  
    /**
    Sets the analogue pin that the Rudder potentiometer is connected to.
 
    @param pin number of the hardware pin you connected it to.
    */
    void setRudderPin(int pin);
  
    /**
    Returns the analogue pin that the Rudder potentiometer is connected to.

    */
    int getRudderPin();
  
    /**
    Sets the analogue pin that the Flight Mode potentiometer is connected to.
 
    @param pin number of the hardware pin you connected it to.
    */
    void setFlightModePin(int pin);
  
    /**
    Returns the analogue pin that the Flight Mode potentiometer is connected to.

    */
    int getFlightModePin();
  
    /**
    Reads all of the analogue values, maps the value to between -100 and 100, and sends the 
    values using the sendRTEA function.
    */
    void sendSticks();
  
    /**
    Sets the values that sendSticks() uses for mapping.  
   
    @param min The analogue value that the arduino reads when the stick position is all the way down.
    @param max The analogue value that the arduino reads when the stick position is all the way up. 
    */
    void throttleEndpoints(uint8_t min, uint8_t max);
  
    /**
    Sets the values that sendSticks() uses for mapping.  
   
    @param min The analogue value that the arduino reads when the stick position is all the way down.
    @param max The analogue value that the arduino reads when the stick position is all the way up. 
    */
    void rudderEndpoints(uint8_t min, uint8_t max);
  
    /**
    Sets the values that sendSticks() uses for mapping.  
   
    @param min The analogue value that the arduino reads when the stick position is all the way down.
    @param max The analogue value that the arduino reads when the stick position is all the way up. 
    */
    void aileronEndpoints(uint8_t min, uint8_t max);
  
    /**
    Sets the values that sendSticks() uses for mapping.  
   
    @param min The analogue value that the arduino reads when the stick position is all the way down.
    @param max The analogue value that the arduino reads when the stick position is all the way up. 
    */
    void elevatorEndpoints(uint8_t min, uint8_t max);
};

class AvantXbee
/**

 This class configure the Xbee connected to both the drone and the arduino board. 
 With this calls you can pair both Xbee in code instead of the hardware button.
 This approach gives you much more flexibility if you are in noisy environment 
 and need to change frequencies etc. Also allows you to reconnect in code to a drone
 if the paring between the two Xbee get lost.

*/
 {
  private:
    //\cond
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantXbee();
    
    AvantXbee(RCTransmitService *rcTservice, Callback *callback);
    //\endcond

    /// modulation rate in symbols per second, default is 115200
    int baud;
    
    /**
    Sets the PanID of the Xbee connected to the arduino to (int id). It accepts values between 0 and 0xFF.
    
    @param id int
    @returns -1 if there was an error with setting the PanID and 1 if it was successful.
    */
    void id(uint8_t id);
};

class AvantPose
/**
 
 The pose class allow you to gather from one place all the data related to position. 
 In this case we are talking about GPS location and compass orientation.
 
 */
{
  private:
     //\cond
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantPose();
    
    AvantPose(RCTransmitService *rcTservice, Callback *callback);
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

class AvantRC //handles sending values to the PWM/PPM port(s) 
/**
 
 This class allows you to control your drone: set the throttle, rudder, elevator, and aileron channels on the drone.
 Essentially controlling up down left right ect.
 
*/
{
  private:
    //\cond
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantRC();
    
    AvantRC(RCTransmitService *rcTservice, Callback *callback);
    //\endcond
    
    /**
    
    Sends a request to the extender to set the Aileron channel to a specific channel between -100 and 100.  
    
    @param value The value to set the Aileron channel to

    */
    void setAileron(int value);
    
    /**
         
    Sends a request to the extender to set the Extender channel to a specific channel between -100 and 100.  
    
    @param value The value to set the Extender channel to
        
    */
    void setElevator(int value);
    
    /**
        
    Sends a request to the extender to set the Throttle channel to a specific channel between -100 and 100.  
    
    @param value The value to set the Throttle channel to
         
    */
    void setThrottle(int value);
    
    /**
         
    Sends a request to the extender to set the Rudder channel to a specific channel between -100 and 100.  
    
    @param value The value to set the Rudder channel to
         
    */
    void setRudder(int value);
    
    /**
         
    Sends a request to the extender to set the Flight Mode channel to a specific channel between -100 and 100.  
    
    @param value The value to set the Flight Mode channel to
         
    */
    void setFlightMode(int value);
    
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
    void aileronCallback(void (*function)(byte));
    
    /**

    Execute funtion in a thread to not freez the drone.
         
    */
    void flightModeCallback(void (*function)(byte));
    
    /**
         
    Execute funtion in a thread to not freez the drone.
         
    */
    void throttleCallback(void (*function)(byte));
    
    /**
         
    Execute funtion in a thread to not freez the drone.
         
    */
    void rudderCallback(void (*function)(byte));
    
    /**
     
    Execute funtion in a thread to not freez the drone.
     
    */
    void elevatorCallback(void (*function)(byte));
    
    /**
     
    Execute funtion in a thread to not freez the drone.
     
    */
    void sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron);
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
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantGPIO();
    
    AvantGPIO(RCTransmitService *rcTservice, Callback *callback);
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
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantI2C();
    
    AvantI2C(RCTransmitService *rcTservice, Callback *callback);
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
    RCTransmitService *service;
    Callback *myCallback;
    //\endcond
  public:
    //\cond
    AvantSPI();
    
    AvantSPI(RCTransmitService *rcTservice, Callback *callback);
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
    RCTransmitService *service;
    Callback *myCallback;

public:
    AvantAutoPilot();
    AvantAutoPilot(RCTransmitService *rcTservice, Callback *callback);
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
class Avant 
//\endcond
{
    private:
      AvantTransmitter avantTransmitter;
      AvantRC avantRC;
      AvantXbee avantXbee;
      RCTransmitService rcService;
      AvantGPIO avantGPIO;
      AvantResponseHandler responseHandler;
      AvantI2C avantI2C;
      Callback callback;
      AvantPose avantPose;
      AvantSPI avantSPI;
      AvantAutoPilot avantAutoPilot;
    public:
      Avant();
      Avant(int hardwareSerialCode);
      Avant(int txPin, int rxPin);
      AvantTransmitter& transmitter();
      AvantRC& RC();
      AvantGPIO& GPIO();
      AvantI2C& I2C();
      AvantXbee& xbee();
      AvantPose& pose();
      AvantResponseHandler& avantResponseHandler();
      AvantSPI& SPI();
      AvantAutoPilot& AutoPilot();
      void armDrone();
      void initialize();
};