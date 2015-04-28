#ifndef __ArduinoSDK__GPIO__
#define __ArduinoSDK__GPIO__

#include "SerialIO.h"
#include "Callback.h"

class GPIO
/**

 This class controls all functionality related to the general purpose IO pins on the drone, 
 includes turning pins high/low, setting PWM values, reading if pins are high/low ect.
 
 Basically all the extra hardware that you connected to the 10 pins on the NVextenderd can be 
 accessed and controlled using this Class.
 
*/
{
  private:
    //\cond
    SerialIO *serialIO;
    Callback *callbacks;
    //\endcond
  public:
    //\cond
    GPIO();
    
    GPIO(SerialIO *_serialIO, Callback *_callbacks);
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
 
    @param cb function name of the function to pass received information to
    @param pin the desired pin to sample logic level information from 
 
    */
    void digitalReadCallback(void (*cb)(byte), int pin);
    
    /**
    
    This callback function is executed every time it gets pulseIn information.  It passes
    this information to the function specified in the argument.
     
    @param cb name of the function to pass received information to
    @param pin the desired pin to sample pulseIn information from 
     
    */
    void pulseInCallback(void (*cb)(long), uint8_t pin);
    
    /**
     
    This callback function is executed every time it gets analogRead information.  It passes
    this information to the function specified in the argument.
     
    @param cb function name of the function to pass received information to
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
};

#endif /* defined __ArduinoSDK__GPIO__ */
