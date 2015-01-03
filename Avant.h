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
		void (*i2cRead)(int); 
		
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
        int sendData(int data, uint8_t resourceID, uint8_t actionID);
};


class AvantResponseHandler {
	private:
		RCTransmitService service;
		Callback myCallback;
	public:
		AvantResponseHandler();
		AvantResponseHandler(RCTransmitService& rcTservice);
		AvantResponseHandler(RCTransmitService& rcTservice, Callback& callback);
		void responseHandler();
		
};

class AvantSetup 
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
        int ailronPin;
        int throttlePin;
        int rudderPin;
        int flightModePin;
		RCTransmitService service;
    public:
		/**
        Description of the method
        */
        AvantSetup();
		AvantSetup(RCTransmitService& rcService);
        ~AvantSetup();
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
		void setAilronPin(int pin);
		/**
        Method descriptionDescription of the method
        @returns Return description
        */
		int getAilronPin();
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
		void (*callback)(float);
		
    public:
		/// Variable Description
        uint8_t rxPin;
		/// Variable Description
        uint8_t txPin;
		/// Variable Description
        int baud;
		/**
        Description od the method
        @param id Parameter description
        @returns Return description
        */
        uint8_t id(uint8_t id);
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
        RCTransmitService service;
    public:
		/**
        Method description
        */
        AvantRC();
		/**
        Method description
        @param rcTservice Parameter description
        */
        AvantRC(RCTransmitService& rcTservice);
		/**
        Method description
        @param value Parameter description
        */
        void setAilron(int value);
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
        int getAilron();
		/**
        Method description
        @returns Return description
        */
        int getElevator();
		/**
        Method description
        @returns Return description
        */
        int getThrottle();
		/**
        Method description
        @returns Return description
        */
        int getRudder();
		/**
        Method description
        @returns Return description
        */
        int getFlightMode();
		/**
        Method description
        @returns Return description
        */
        int readSensorReading();
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
		RCTransmitService service;
		void (*callback)(float);
	public:
		AvantGPIO();
		AvantGPIO(RCTransmitService& rcTservice);
		void pinMode(uint8_t pin, bool logicLevel);
		void digitalWrite(uint8_t pin,bool logicLevel);
		void analogWrite(uint8_t pin, uint8_t value);
		void digitalRead(uint8_t pin);
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
		RCTransmitService service;
		Callback myCallback;
	public:
		/**
        Method description
        */
		AvantI2C();
		/**
        Method description
        @param rcTservice Parameter description
        */
		AvantI2C(RCTransmitService& rcTservice);
		
		
		AvantI2C(RCTransmitService& rcTservice, Callback& callback);
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
		void readCallback(void (*function)(int));
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
        AvantSetup setup;
        AvantRC rc;
        RCTransmitService rcService;
		AvantGPIO gpio;
		AvantResponseHandler responseHandler;
		AvantI2C i2c;
		Callback callback;
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
        AvantSetup& avantSetup();
		/**
        Method description
        @returns Return description
        */
        AvantRC& avantRC();
		/**
        Method description
        @returns Return description
        */
		AvantGPIO& avantGPIO();
		/**
        Method description
        @returns Return description
        */
		AvantI2C& avantI2C();
		/**
        Method description
        @returns Return description
        */
		AvantResponseHandler& avantResponseHandler();
		/**
        Method description
        */
        void armDrone();
		/**
        Method description
        */
        void disarmDrone();
};

