
#ifndef __ArduinoSDK__AvantI2C__
#define __ArduinoSDK__AvantI2C__


#include "SerialIO.h"
#include "Callback.h"

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

#endif