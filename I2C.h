#ifndef __ArduinoSDK__I2C__
#define __ArduinoSDK__I2C__

#include "SerialIO.h"
#include "Callback.h"
#include "ResponseHandler.h"

class I2C
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
    SerialIO *serialIO;
    Callback *callbacks;
    ResponseHandler *responseHandler;
    //\endcond
  public:
    //\cond
    I2C();
    
    I2C(SerialIO *_serialIO, Callback *_callback, ResponseHandler *_responseHandler);
    //\endcond

    /**
         
    This sets the address of the device that the App Extender sends data too.
    This address will be continuously used by beginTransmission, write, read, and wireRequest
    until it is reset to a different value.  
        
    @param ID range from 0 to 255
        
    */
    void setDeviceAddress(uint8_t address);

    /**
    
    Sends a request to the App Extender to read multiple bytes of information from the I2C buffer and return
    the information to the arduino. The relpies are handled by the readCallback() function.

    @param bytes Sets the number of bytes to read from the I2C buffer

    */
    void wireRequest(uint8_t bytes);

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
         
    This callback function is executed everytime it gets I2C information.  It passes
    this information to the function specified in the argument.
        
    @param cb name of the method
    
    */
    void readCallback(void (*cb)(byte));
};

#endif /* defined __ArduinoSDK__I2C__ */
