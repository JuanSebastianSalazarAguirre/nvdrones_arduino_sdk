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
/**
This class allows you to...
*/
{
	friend class AvantResponseHandler;
    private:
        bool isHwSerial0Used;
        bool isHwSerial1Used;
        bool isHwSerial2Used;
        bool isHwSerial3Used;
        bool isSwSerialUsed;
        SoftwareSerial softwareSerial;
    public:
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

		/**
        Method description

        @param data Parameter description
        @param resourceID Parameter description
        @param actionID Parameter description
        */
        void sendData(int data, uint8_t resourceID, uint8_t actionID);

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
 You will have to tell the NVextender waht pinn is connected to waht drone controller. 
 Based on the naming convention from the methods in this class you should be able 
 to esly udnerstand what type of information the hardware needs.

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
		RCTransmitService *service;
    public:
        AvantTransmitter();

        /**
        TODO

        @param rcService
        */
		AvantTransmitter(RCTransmitService *rcService);
		
        /**
        Setting the hardwer pin so the system knows what type of data to expect.

        @param pin Hardware pin number.
        */
		void setElevatorPin(int pin);
		
        /**
        Retrive the hardware pin number
         
        @returns Returns the pin value
        */
		int getElevatorPin();
		
        /**
        Setting the hardwer pin so the system knows what type of data to expect.
         
        @param pin number of the hardwer pin you connected it to.
        */
		void setAileronPin(int pin);
		
        /**
        Retrive the hardware pin number
         
        @returns Returns the pin value
        */
		int getAileronPin();
		
        /**
        Setting the hardwer pin so the system knows what type of data to expect.
         
        @param pin number of the hardwer pin you connected it to.
        */
		void setThrottlePin(int pin);
		
        /**
        Retrive the hardware pin number
     
        @returns Returns the pin value
        */
		int getThrottlePin();
		
        /**
        Setting the hardwer pin so the system knows what type of data to expect.
     
        @param pin number of the hardwer pin you connected it to.
        */
		void setRudderPin(int pin);
		
        /**
        Retrive the hardware pin number
     
        @returns Returns the pin value
        */
		int getRudderPin();
		
        /**
        Setting the hardwer pin so the system knows what type of data to expect.
     
        @param pin number of the hardwer pin you connected it to.
        */
		void setFlightModePin(int pin);
		
        /**
        Retrive the hardware pin number
     
        @returns Returns the pin value
        */
		int getFlightModePin();
		
        /**
        Retrive the hardware pin number
        */
		void sendSticks();
};

class AvantXbee
/**

 This class configure the Xbee connected to both the drone and the arduino board. 
 With this calls you can pair both Xbee in code instead of the hardware button.
 This aproache gives you much more flexibility if you are in noisy enviroment 
 and need to cheng frequencies etc. Alos allwos you to recconnect in code to a drone
 if the paring betwen the two Xbee get lost.

*/
 {
	private:
		RCTransmitService *service;
		Callback *myCallback;
    public:
		AvantXbee();

        /**
        TODO

        @param rcTservice
        @param callback
        */
		AvantXbee(RCTransmitService *rcTservice, Callback *callback);

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
 In this case we are takign about GPS location and compass orientation.
 
 */
{
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantPose();

        /**
        TODO

        @param rcTservice
        @param callback
        */
		AvantPose(RCTransmitService *rcTservice, Callback *callback);

        /**
         
        Get GPS data
         
        */
		void getGPSData();
		
        /**
         
         Call back method that needs to be implemented to recive the actual data from: longitudeCallback()
         
         */
        void getLongitude();
		
        /**
         
         Call back method that needs to be implemented to recive the actual data from: altitudeCallback()
         
         */
        void getLatitude();
		
        /**
         
         Call back method that needs to be implemented to recive the actual data from: altitudeCallback()
         
         */
        void getAltitude();
		
        /**
         
         Call back method that needs to be implemented to recive the actual data from: satelliteCallback().
         
         The more satelites you have the higher precinon yuo will get, thus if you want to be sure you are
         gettign a precise mesuremnt, you shiuld wait untill youget more then 4 satelites. Sicne 4 is the 
         bair minimum to get a 3D fix
         
         */
        void getSatellites();
		
        /**
         
         Get drone speed based on GPS data
         
         */
        void getSpeed();
		
        /**
         
         Call back method that needs to be implemented to recive the actual data from: speedCallback()
         
         */
        void getOrientation();
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void longitudeCallback(void (*function)(float));
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void latitudeCallback(void (*function)(float));
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void altitudeCallback(void (*function)(float));
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void satelliteCallback(void (*function)(byte));
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void speedCallback(void (*function)(float));
		
        /**
         
         Execute funtion in a thread to not freez the drone.
         
         */
        void orientationCallback(void (*function)(float));
};

class AvantRC //handles sending values to the PWM/PPM port(s) 
/**
 
 This class allows you to controll your drone: set the throttle, rudder, elevator, and aileron channels on the drone.
 Essentially controlling up down left right ect.
 
*/
{
    private:
        RCTransmitService *service;
		Callback *myCallback;
    public:
        AvantRC();

        /**
        TODO

        @param rcTservice
        @param callback
        */
        AvantRC(RCTransmitService *rcTservice, Callback *callback);
		
        /**
         Basically the roll movnent that the drone have to do. The expected range is betwen -10 and 10.
        
         @param value roll value
        */
        void setAileron(int value);
		
        /**
         
         The pitch of the drone or in other words, how high you want you drone to go up. The expected range is betwen 0 and 1000
         
         @param value elevation value
        
         */
        void setElevator(int value);
		
        /**
        
         This vuale will make your adrenalien rise up, isnce it is the speed of your drone. The expected values are from 0 to 100.
         The accelration is based on the technical specification fo your drone
        
         @param value speed value
         
         */
        void setThrottle(int value);
		
        /**
         
         The yaw parameter allows you to set the dirrection at whithc it is pointing, allowing you to turn. The expected range is betwne 0 and 360.
         The yaw is based on the technical specification fo your drone
        
         @param value yaw value
         
         */
        void setRudder(int value);
		
        /**
         
         This method allow to set the flight mode of the drone ...
        
         @param value flight mode value
         
         */
        void setFlightMode(int value);
		
        /**
         
         Get the exact roll position of the drone
        
         @returns int value
         
         */
        void getAileron();
		
        /**

         Get the elevation position of the drone
        
         @returns int value
        
         */
        void getElevator();
		
        /**
         
         Get the speed value of the drone
        
         @returns int value
        
         */
        void getThrottle();
		
        /**
        
         Get the dirrection of the drone
        
         @returns int value
        
         */
        void getRudder();
		
        /**
        
         Get the flight mode
        
         @returns int value
        
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
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantGPIO();

        /**
        TODO

        @param rcTservice
        @param callback
        */
		AvantGPIO(RCTransmitService *rcTservice, Callback *callback);

        /**
     
         Set the pin mode in whihc the pin should operate.
     
         @param pin value expected 0 to 10
         @param logicLevel expected 0 or 1
     
         */
		void pinMode(uint8_t pin, bool logicLevel);

        /**
     
         Send to the digital binary data
     
         @param pin value expected 0 to 10
         @param logicLevel expected 0 or 1
     
         */
		void digitalWrite(uint8_t pin,bool logicLevel);

        /**
     
         Send analog data to the selected pin
     
         @param pin value expected 0 to 10
         @param value expected somethins
     
         */
		void analogWrite(uint8_t pin, uint8_t value);

        /**
     
         Read data form selected pin in binary.
     
         @param pin number to listent to.
     
         */
		void digitalRead(uint8_t pin);

        /**
     
         This method allows you to set a listner for new data and call a metod that you set to be 
         called from your code once there is data to be returned. This way your code won't freez 
         the done while waiting for data to come back grom the NVextender
     
         @param function name in you code to be called
     
         */
		void digitalReadCallback(void (*function)(byte));
};

class AvantI2C
/**

 This class allow you to gain acess to the I2C serial bus, allowing you to communicate with
 sensors and actuators that you connected to the NVextender using the exposed PINs.
 
 # Example

 @htmlonly
    <script src="https://gist.github.com/davidgatti/790c8fdcd27385b12043.js"></script>
 @endhtmlonly
 
 */
{
	private:
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantI2C();

        /**
        TODO

        @param rcTservice
        @param callback
        */
		AvantI2C(RCTransmitService *rcTservice, Callback *callback);

		/**
         
         Since the I2C allows for many devices to communicate over the same channe. 
         Each device distinqueshi itselfe with a Device ID. This ID can be sed by the 
         factory and should be described in the documentation, or you should be able 
         to set the ID manually to avoid conflicts
        
         @param ID range from 0 to 255
        
         */
		void deviceID(uint8_t ID);
		
        /**
    
         Some sensors ned a nudge before they will start transmittign data, and this 
         method allows you to inform the sensor to start trasmitig.
        
         */
		void beginTransmission(void);
		
        /**
    
         Similalry to beginTransmission(), once you don't need recivign data fro ma 
         selcted snsor anymore. Use this method to stop the data sampling
         
         */
		void endTransmission(void);
		
        /**
         
         This methd allows you to send commands to the actuator connected to your 
         drone. For example, you can tell your motor how many revolution per minute should do.
        
         @param data range betwen 0 and 10000
        
         */
		void write(uint8_t data);
		
        /**
        
         This method will tell the NVextender to sample data only once. A good example would 
         be to read from time to time the temeprature, sinic this is not a peach of 
         information that changes quickly over time.
         
         The data will be reutnru to a colback method taht you specified with readCallback()
        
         */
		void read(void);
		
        /**
    
         Method description
        
         @param bytes Parameter description
        
         */
		void wireRequest(uint8_t bytes);
		
        /**
         
         This method let you set which method to call back for the data that you request from the NVextender
        
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
		RCTransmitService *service;
		Callback *myCallback;
	public:
		AvantSPI();

        /**
        TODO

        @param rcTservice
        @param callback
        */
		AvantSPI(RCTransmitService *rcTservice, Callback *callback);

        /**

        Read data form selected pin in binary.

        @param data pin number to listent to.

        */
		void transfer(byte data);

        /**

        Read data form selected pin in binary.

        @param data pin number to listent to.

        */
		void setBitOrder(byte data);

        /**

        Read data form selected pin in binary.

        @param data pin number to listent to.

        */
		void setClockDivider(byte data);

        /**

        Read data form selected pin in binary.

        @param data pin number to listent to.

        */
		void setDataMode(byte data);

        /**
         
         Read data form selected pin in binary.
         
         */
		void transferCallback(void (*function)(byte));
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
        void armDrone();
};

float readFloat() {
    float data;
    Serial.readBytes((char*)&data, sizeof(data));
    return data;
}
