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

class AvantSetup //stores which analogPins to get stick data from
{                 //This is only used for the function sendSticks
    private:
        static uint8_t elevatorPin;
        static uint8_t ailronPin;
        static uint8_t throttlePin;
        static uint8_t rudderPin;
        static uint8_t flightModePin;
    public:
        AvantSetup();
        ~AvantSetup();
        void setElevatorPin(uint8_t pin);
        uint8_t getElevatorPin();
        void setAilronPin(uint8_t pin);
        uint8_t getAilronPin();
        void setThrottlePin(uint8_t pin);
        uint8_t getThrottlePin();
        void setRudderPin(uint8_t pin);
        uint8_t getRudderPin();
};

class AvantXbee  //handles configuring the Xbee
{
    public:
        uint8_t rxPin;
        uint8_t txPin;
        int baud;
        uint8_t id(uint8_t id);
};


class RCTransmitService
{
    private:
        bool isHwSerial0Used;
        bool isHwSerial1Used;
        bool isHwSerial2Used;
        bool isHwSerial3Used;
        bool isSwSerialUsed;
        SoftwareSerial softwareSerial;
    public:
        RCTransmitService();
        RCTransmitService(int txPin , int rxPin);
        RCTransmitService(int hwSerialCode);
        int sendData(int data, uint8_t resourceID, uint8_t actionID);
};


class AvantRC //handles sending values to the PWM/PPM port(s) 
{
    private:
        RCTransmitService service;
    public:
        AvantRC();
        AvantRC(RCTransmitService rcTservice);
        void setAilron(int value);
        void setElevator(int value);
        void setThrottle(int value);
        void setRudder(int value);
        void setFlightMode(int value);
        int getAilron();
        int getElevator();
        int getThrottle();
        int getRudder();
        int getFlightMode();
        void sendSticks();
        int readSensorReading();
};

class Avant
{
    private:
        AvantSetup setup;
        AvantRC rc;
        RCTransmitService rcService;
        void (*callback)(float);
        
    public:
        Avant();
        Avant(int hardwareSerialCode);
        Avant(int txPin, int rxPin);
        AvantSetup avantSetup();
        AvantRC avantRC();
        void armDrone();
        void disarmDrone();

        void setCallbackFunction(void (*function)(float));
        void readData();
};

