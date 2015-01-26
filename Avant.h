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
class SoftwareSerial : public Stream
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
class Callback {
	public:
		void (*i2cRead)(byte); 
		void (*digitalRead)(byte);
		void (*longitude)(float);
		void (*latitude)(float);
		void (*altitude)(float);
		void (*speed)(float);
		void (*satallite)(byte);
		void (*orientation)(float);
		void (*transfer)(byte);
		void (*flightMode)(byte);
		void (*elevator)(byte);
		void (*aileron)(byte);
		void (*throttle)(byte);
		void (*rudder)(byte);
		
};

class RCTransmitService
{
	/**
	This class allows you to...
	@pre Don't forget to first connect your Arduino to the Avant platform.
	# Example
	Below you can see how this class shiuld be used.
	~~~{.ino}
		println("This is where your write example code.")
		println("Have a nice day.")
		println("NVdrones Developer Relations")
	~~~
	*/
	friend class AvantResponseHandler;
    private:
        bool isHwSerial0Used;
        bool isHwSerial1Used;
        bool isHwSerial2Used;
        bool isHwSerial3Used;
        bool isSwSerialUsed;
        SoftwareSerial softwareSerial;
    public:
	    /**
        Method description
        */
        RCTransmitService();
		/**
        Method description
        @param txPin Parameter description
        @param rxPin Parameter description
        */
        RCTransmitService(int txPin , int rxPin);
		/**
        Method description
        @param hwSerialCode Parameter description
        */
        RCTransmitService(int hwSerialCode);
		/**
        Method description
        @param data Parameter description
        @param resourceID Parameter description
        @param actionID Parameter description
        @returns Return description
        */
        void sendData(int data, uint8_t resourceID, uint8_t actionID);
		void print(String data);
		void write(byte data);
		void readBytes(char *buffer, int bytesToRead);
		
};


class AvantResponseHandler {
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantResponseHandler();
		AvantResponseHandler(RCTransmitService *rcTservice, Callback *callback);
		void responseHandler();
		void callbackTest(byte test){
			myCallback->i2cRead(test);
		}
		
};

class AvantTransmitter 
{
/**
This class allows you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class shiuld be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
    private:
		int elevatorPin;
        int AileronPin;
        int throttlePin;
        int rudderPin;
        int flightModePin;
		RCTransmitService *service;
    public:
		/**
        Description of the method
        */
        AvantTransmitter();
		AvantTransmitter(RCTransmitService *rcService);
		/**
        Description of the method
        @param pin Parameter description
        */
		void setElevatorPin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getElevatorPin();
		/**
        Description of the method
        @param pin Parameter description
        */
		void setAileronPin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getAileronPin();
		/**
        Description of the method
        @param pin Parameter description
        */
		void setThrottlePin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getThrottlePin();
		/**
        Description of the method
        @param pin Parameter description
        */
		void setRudderPin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getRudderPin();
		/**
        Description of the method
        @param pin Parameter description
        */
		void setFlightModePin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getFlightModePin();
		
		void sendSticks();
};

class AvantXbee  //handles configuring the Xbee
{
/**
This class allows you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class should be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
	private:
		RCTransmitService *service;
		Callback *myCallback;
    public:
		AvantXbee();
		AvantXbee(RCTransmitService *rcTservice, Callback *callback);
        int baud;
		/**
        Description of the method
        @param id Parameter description
        @returns Return description
        */
        void id(uint8_t id);
};

class AvantPose
{
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantPose();
		AvantPose(RCTransmitService *rcTservice, Callback *callback);
		void getGPSData();
		void getLongitude();
		void getLatitude();
		void getAltitude();
		void getSatellites();
		void getSpeed();
		void getOrientation();
		void longitudeCallback(void (*function)(float));
		void latitudeCallback(void (*function)(float));
		void altitudeCallback(void (*function)(float));
		void satelliteCallback(void (*function)(byte));
		void speedCallback(void (*function)(float));
		void orientationCallback(void (*function)(float));
};

class AvantRC //handles sending values to the PWM/PPM port(s) 
{
/**
This class allows you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class should be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
    private:
        RCTransmitService *service;
		Callback *myCallback;
		
    public:
		/**
        Method description
        */
        AvantRC();
		/**
        Method description
        @param rcTservice Parameter description
        */
        AvantRC(RCTransmitService *rcTservice, Callback *callback);
		/**
        Method description
        @param value Parameter description
        */
        void setAileron(int value);
		/**
        Method description
        @param value Parameter description
        */
        void setElevator(int value);
		/**
        Method description
        @param value Parameter description
        */
        void setThrottle(int value);
		/**
        Method description
        @param value Parameter description
        */
        void setRudder(int value);
		/**
        Method description
        @param value Parameter description
        */
        void setFlightMode(int value);
		/**
        Method description
        @param value Parameter description
        */
        void getAileron();
		/**
        Method description
        @returns Return description
        */
        void getElevator();
		/**
        Method description
        @returns Return description
        */
        void getThrottle();
		/**
        Method description
        @returns Return description
        */
        void getRudder();
		/**
        Method description
        @returns Return description
        */
        void getFlightMode();
		
		void aileronCallback(void (*function)(byte));
		
		void flightModeCallback(void (*function)(byte));
		
		void throttleCallback(void (*function)(byte));
		
		void rudderCallback(void (*function)(byte));
		
		void elevatorCallback(void (*function)(byte));
		
		void sendRTEA(uint8_t rudder, uint8_t throttle, uint8_t elevator, uint8_t aileron);
};

class AvantGPIO {
/**
This class allows you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class shiuld be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantGPIO();
		AvantGPIO(RCTransmitService *rcTservice, Callback *callback);
		void pinMode(uint8_t pin, bool logicLevel);
		void digitalWrite(uint8_t pin,bool logicLevel);
		void analogWrite(uint8_t pin, uint8_t value);
		void digitalRead(uint8_t pin);
		void digitalReadCallback(void (*function)(byte));
};

class AvantI2C {
/**
This class allows you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class shiuld be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		/**
        Method description
        */
		AvantI2C();
		/**
        Method description
        @param rcTservice Parameter description
        */
		AvantI2C(RCTransmitService *rcTservice, Callback *callback);
		/**
        Method description
        @param ID Parameter description
        */
		void deviceID(uint8_t ID);
		/**
        Method description
        */
		void beginTransmission(void);
		/**
        Method description
        */
		void endTransmission(void);
		/**
        Method description
        @param data Parameter description
        */
		void write(uint8_t data);
		/**
        Method description
        */
		void read(void);
		/**
        Method description
        @param bytes Parameter description
        */
		void wireRequest(uint8_t bytes);
		/**
        Method description
        @param function Parameter description
        */
		void readCallback(void (*function)(byte));
};

class AvantSPI {
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantSPI();
		AvantSPI(RCTransmitService *rcTservice, Callback *callback);
		void transfer(byte data);
		void setBitOrder(byte data);
		void setClockDivider(byte data);
		void setDataMode(byte data);
		void transferCallback(void (*function)(byte));
};


class Avant {
/**
This class allow you to...
@pre Don't forget to first connect your Arduino to the Avant platform.
# Example
Below you can see how this class should be used.
~~~{.ino}
    println("This is where your write example code.")
    println("Have a nice day.")
    println("NVdrones Developer Relations")
~~~
*/
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
    public:
		/**
        Method description
        */
        Avant();
		/**
        Method description
        @param hardwareSerialCode Parameter description
        */
        Avant(int hardwareSerialCode);
		/**
        Method description
        @param txPin Parameter description
        @param rxPin Parameter description
        */
        Avant(int txPin, int rxPin);
		/**
        Method description
        @returns Return description
        */
        AvantTransmitter& transmitter();
		/**
        Method description
        @returns Return description
        */
        AvantRC& RC();
		/**
        Method description
        @returns Return description
        */
		AvantGPIO& GPIO();
		/**
        Method description
        @returns Return description
        */
		AvantI2C& I2C();
		/**
        Method description
        @returns Return description
        */
		AvantXbee& xbee();
		/**
        Method description
        @returns Return description
        */
		AvantPose& pose();
		/**
        Method description
        @returns Return description
        */
		AvantResponseHandler& avantResponseHandler();
		/**
        Method description
        @returns Return description
        */
		AvantSPI& SPI();
		/**
        Method description
        */
        void armDrone();
};

